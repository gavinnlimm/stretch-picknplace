#!/usr/bin/env python3
"""
Robot client for a trained ACT policy - runs ON THE ROBOT (needs ROS + cv2, nothing else).
Reads the head camera, joint states and odometry, asks policy_server.py (GPU PC) for the next chunk of actions,
and moves the robot toward the chunk's target, then asks again. The policy only drives base_x (forward),
lift, arm, wrist yaw and gripper. The head is NOT controlled: set the head pose first (see HEAD below).

    # robot tab, driver launched WITHOUT mode:=gamepad, camera running, head set, area clear, hand on the run-stop
    /usr/bin/python3 ~/policy_client.py --server http://<gpu ip>:8000 --module drop            # DRY RUN: only prints
    /usr/bin/python3 ~/policy_client.py --server http://<gpu ip>:8000 --module drop --go       # moves, runs all steps without stopping

Safety: dry run by default; --go needed to move; with --go it runs continuously (add --confirm to be asked for Enter before every move);
targets are clipped to the joint ranges seen in the demos and to a small change per step; it stops if the camera or
joint states go stale, the head pose is wrong, the server fails, or after --seconds. Ctrl+C stops it.
base_x is measured from where the robot stands when this script starts (the same as in the demos).
STATUS: tested on the robot (drop module); results in docs/experiments.md.
"""
import argparse, base64, json, math, os, sys, time, urllib.request
import numpy as np

SIZE = (320, 240)                                   # same as demo_logger.py
JOINTS = {'lift': 'joint_lift', 'arm': 'wrist_extension', 'wrist_yaw': 'joint_wrist_yaw',
          'gripper': 'joint_gripper_finger_left', 'head_pan': 'joint_head_pan', 'head_tilt': 'joint_head_tilt'}
ACTION_JOINTS = ['lift', 'arm', 'wrist_yaw', 'gripper']             # order of actions 1..4 (action 0 is base_x)
# head pose each module was recorded at (rad); the client refuses to run if the head is further than HEAD_TOL away
HEAD = {'pick': (-1.29, -0.75), 'drop': (-1.38, -0.60)}; HEAD_TOL = 0.08
# joint ranges seen in the usable demos (+/- 0.05) - targets are clipped to these (arm >= 1 cm so it never presses its end stop)
LIMITS = {'pick': {'lift': (0.61, 0.97), 'arm': (0.01, 0.56), 'wrist_yaw': (-0.05, 0.05), 'gripper': (-0.30, 0.09)},
          'drop': {'lift': (0.68, 1.01), 'arm': (0.01, 0.52), 'wrist_yaw': (-0.05, 0.05), 'gripper': (-0.40, 0.05)}}
MAX_DELTA = {'lift': 0.10, 'arm': 0.10, 'wrist_yaw': 0.05, 'gripper': 0.15, 'base': 0.10}   # max move per step from now
SPEED = {'lift': 0.10, 'arm': 0.10, 'wrist_yaw': 0.5, 'gripper': 0.3, 'base': 0.10}          # m/s or rad/s (slower than the demos)


# ---------- pure helpers (no ROS; tested offline) ----------
def to_bgr(m):
    """ROS Image (rgb8/bgr8) -> BGR array, or None."""
    import cv2
    if m.encoding not in ('rgb8', 'bgr8'): return None
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a


def encode_frame(bgr):
    """BGR frame -> base64 JPEG, resized and compressed exactly like demo_logger.py saved the training frames."""
    import cv2
    small = cv2.resize(bgr, SIZE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return base64.b64encode(buf.tobytes()).decode()


def base_forward(o0, o):
    """Forward distance (m) from the start pose o0 along the start heading. o = [x, y, qz, qw]."""
    yaw0 = 2 * math.atan2(o0[2], o0[3])
    return (o[0] - o0[0]) * math.cos(yaw0) + (o[1] - o0[1]) * math.sin(yaw0)


def state_vector(js, bx):
    """[base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt] - same order as the converter."""
    return [bx] + [float(js.get(JOINTS[k], 0.0)) for k in ('lift', 'arm', 'wrist_yaw', 'gripper', 'head_pan', 'head_tilt')]


def plan_step(actions, state, module, n_exec, base_mode='absolute'):
    """Pick the target of the n_exec-th action of the chunk, clip it, and work out how long the move should take.
    state is always the full 7-value state (base_x first). base_mode 'relative': action[:, 0] is the base move per
    0.1 s step, so the base moves by the sum over the first n_exec actions.
    Returns dict(joints={name: absolute target}, dx=base move in m, secs_joints, secs_base, notes=[...])."""
    a = np.asarray(actions, float)
    if a.ndim != 2 or a.shape[1] != 5 or len(a) == 0 or not np.all(np.isfinite(a)): raise ValueError('bad action chunk')
    tgt = a[min(n_exec, len(a)) - 1]; notes = []; joints = {}; secs = max(1.0, 0.1 * n_exec)
    for i, k in enumerate(ACTION_JOINTS, start=1):
        cur = state[i]; lo, hi = LIMITS[module][k]; want = float(tgt[i])
        v = min(max(want, lo), hi)
        d = min(max(v - cur, -MAX_DELTA[k]), MAX_DELTA[k]); v = cur + d
        if abs(v - want) > 1e-3: notes.append(f'{k}: wanted {want:+.3f}, using {v:+.3f}')
        if abs(d) > 0.005: joints[JOINTS[k]] = v; secs = max(secs, abs(d) / SPEED[k])
    dx = float(a[:min(n_exec, len(a)), 0].sum()) if base_mode == 'relative' else float(tgt[0]) - state[0]
    dxc = min(max(dx, -MAX_DELTA['base']), MAX_DELTA['base'])
    if abs(dxc - dx) > 1e-3: notes.append(f'base: wanted {dx:+.3f} m, using {dxc:+.3f}')
    if abs(dxc) < 0.01: dxc = 0.0
    return {'joints': joints, 'dx': dxc, 'secs_joints': secs, 'secs_base': abs(dxc) / SPEED['base'] + 1.0, 'notes': notes}


RELEASE_OPEN = -0.02      # gripper position the drop demos reached when the can fell (-0.02 .. -0.12)
RELEASE_MAX = 0.10        # widest scripted release; same opening drop_reset.py uses (the policy itself is limited to 0.05)


def holding_still_over_container(state, actions, n_exec, dx):
    """True when the arm is out, the base has stopped and the policy is no longer opening the gripper: the stall
    seen with the relative-base model at the end of the drop (gripper stuck near -0.08, can still held)."""
    a = np.asarray(actions, float); want_grip = float(a[min(n_exec, len(a)) - 1][4])
    return state[2] >= 0.38 and dx == 0.0 and state[4] > -0.15 and abs(want_grip - state[4]) < 0.02


def head_ok(state, module):
    pan, tilt = HEAD[module]
    return abs(state[5] - pan) <= HEAD_TOL and abs(state[6] - tilt) <= HEAD_TOL


def server_mode(url, timeout=5.0):
    with urllib.request.urlopen(url.rstrip('/') + '/health', timeout=timeout) as r: return json.loads(r.read()).get('base_mode', 'absolute')


def ask_server(url, b64, state, timeout=5.0):
    req = urllib.request.Request(url.rstrip('/') + '/act', json.dumps({'image': b64, 'state': state}).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r: out = json.loads(r.read())
    if 'actions' not in out: raise RuntimeError(f'server: {out}')
    return out['actions'], out.get('ms')


# ---------- ROS part ----------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--server', required=True, help='e.g. http://128.253.224.8:8000')
    ap.add_argument('--module', choices=['pick', 'drop'], required=True)
    ap.add_argument('--go', action='store_true', help='actually move the robot (default: dry run, prints only)')
    ap.add_argument('--confirm', action='store_true', help='ask for Enter before every move (default: run all steps without stopping)')
    ap.add_argument('--no-confirm', action='store_true', help=argparse.SUPPRESS)   # old name; now the default
    ap.add_argument('--seconds', type=float, default=20.0, help='stop after this long')
    ap.add_argument('--n-exec', type=int, default=10, help='move toward the n-th action of each chunk, then ask again')
    ap.add_argument('--base-mode', choices=['absolute', 'relative'], default='absolute',
                    help='must match how the checkpoint was converted (relative = base_x not in the state)')
    ap.add_argument('--auto-release', action='store_true',
                    help='drop only: if the policy stalls with the arm out and the gripper still mostly closed for 3 steps, '
                         f'open the gripper to {RELEASE_OPEN} yourself and stop. A scripted fallback; report results with and without it')
    ap.add_argument('--release-open', type=float, default=RELEASE_OPEN,
                    help=f'gripper position used by --auto-release (default {RELEASE_OPEN}; larger = wider open, max {RELEASE_MAX})')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from sensor_msgs.msg import Image, JointState
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    class C(Node):
        def __init__(s):
            super().__init__('policy_client'); s.img = s.js = s.odom = None; s.t = {}
            def keep(name, attr): return lambda m: (setattr(s, attr, m), s.t.__setitem__(name, time.time()))
            s.create_subscription(Image, '/camera/color/image_raw', keep('img', 'img'), 1)
            s.create_subscription(JointState, '/stretch/joint_states', keep('js', 'js'), 1)
            s.create_subscription(Odometry, '/odom', keep('odom', 'odom'), 1)
            s.traj = ActionClient(s, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
            s.pos = s.create_client(Trigger, '/switch_to_position_mode')
        def spin(s, sec=0.1):
            """process ROS messages for `sec` seconds (keeps camera / joints / odometry fresh)"""
            end = time.time() + sec
            while True:
                rclpy.spin_once(s, timeout_sec=0.02)
                if time.time() >= end: break
        def fresh(s, max_age=1.0):
            return all(k in s.t and time.time() - s.t[k] < max_age for k in ('img', 'js', 'odom'))
        def position_mode(s):
            s.pos.wait_for_service(timeout_sec=5.0); f = s.pos.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(s, f, timeout_sec=5.0)
            if not f.done() or not f.result().success: raise SystemExit('could not switch to position mode')
        def go(s, joints, secs):
            g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(joints)
            p = JointTrajectoryPoint(); p.positions = [float(v) for v in joints.values()]
            p.time_from_start.sec = int(secs); p.time_from_start.nanosec = int((secs % 1) * 1e9)
            g.trajectory.points = [p]
            if not s.traj.wait_for_server(timeout_sec=5.0): raise SystemExit('no trajectory action server (is the driver running?)')
            f = s.traj.send_goal_async(g); rclpy.spin_until_future_complete(s, f); h = f.result()
            if not h.accepted: raise SystemExit(f'goal rejected {list(joints)} (driver in gamepad mode?)')
            r = h.get_result_async(); rclpy.spin_until_future_complete(s, r)
            print(f'    {r.result().result.error_string}')

    try: sm = server_mode(a.server)
    except Exception as e: sys.exit(f'cannot reach the server: {e!r}')
    if sm != a.base_mode: sys.exit(f'the server model uses base mode "{sm}" but you gave --base-mode {a.base_mode}')
    rclpy.init(); n = C()
    t0 = time.time()
    while not n.fresh() and time.time() - t0 < 10: n.spin()
    if not n.fresh(): sys.exit('no fresh camera / joint states / odometry - are the driver and camera running?')
    o = n.odom.pose.pose; o0 = [o.position.x, o.position.y, o.orientation.z, o.orientation.w]
    js = dict(zip(n.js.name, n.js.position)); st = state_vector(js, 0.0)
    if not head_ok(st, a.module):
        sys.exit(f'head is at pan {st[5]:.2f}, tilt {st[6]:.2f}; the {a.module} demos used {HEAD[a.module]} (+/-{HEAD_TOL}). Set the head first.')
    if a.go: n.position_mode()
    print(f'{a.module}: {"MOVING" if a.go else "dry run"}; {a.base_mode} base; server {a.server}; start base pose recorded; {a.seconds:.0f} s max')
    start = time.time(); step = 0; still = 0; released = False
    try:
        while time.time() - start < a.seconds:
            n.spin(0.3)                                   # catch up on all topics before reading them
            if not n.fresh(): sys.exit('STOP: camera / joint states / odometry went stale')
            o = n.odom.pose.pose; js = dict(zip(n.js.name, n.js.position))
            st = state_vector(js, base_forward(o0, [o.position.x, o.position.y, o.orientation.z, o.orientation.w]))
            if not head_ok(st, a.module): sys.exit('STOP: head moved away from the demo pose')
            bgr = to_bgr(n.img)
            if bgr is None: sys.exit('STOP: unsupported camera encoding ' + n.img.encoding)
            sent = st if a.base_mode == 'absolute' else st[1:]          # relative: the model never sees base_x
            try: chunk, ms = ask_server(a.server, encode_frame(bgr), [round(v, 5) for v in sent])
            except Exception as e: sys.exit(f'STOP: server problem: {e!r}')
            pl = plan_step(chunk, st, a.module, a.n_exec, a.base_mode); step += 1
            print(f'[{time.time()-start:5.1f}s] step {step} ({ms} ms)  state base {st[0]:+.3f} lift {st[1]:.3f} arm {st[2]:.3f} grip {st[4]:+.3f}')
            print('   plan: ' + ', '.join(f'{k.replace("joint_", "")} -> {v:+.3f}' for k, v in pl['joints'].items()) + f'  base {pl["dx"]:+.3f} m')
            for note in pl['notes']: print('   clipped - ' + note)
            still = still + 1 if holding_still_over_container(st, chunk, a.n_exec, pl['dx']) else 0
            if a.auto_release and a.module == 'drop' and still >= 3:
                rel = min(max(a.release_open, LIMITS['drop']['gripper'][0]), RELEASE_MAX)
                print(f'   auto-release: the policy has stalled for {still} steps; opening the gripper to {rel:+.2f}')
                pl['joints'] = {JOINTS['gripper']: rel}; pl['secs_joints'] = 1.5; pl['dx'] = 0.0; released = True
            if not a.go: n.spin(1.0); continue           # dry run: wait, but keep reading the topics
            if a.confirm and input('   Enter = move, q = quit: ').strip().lower() == 'q': break
            if pl['joints']: n.go(pl['joints'], pl['secs_joints'])
            if pl['dx']: n.go({'translate_mobile_base': pl['dx']}, pl['secs_base'])
            if released: print('   released - stopping'); break
    except KeyboardInterrupt:
        print('\nstopped (Ctrl+C)')
    print(f'done after {step} steps')


if __name__ == '__main__':
    main()
