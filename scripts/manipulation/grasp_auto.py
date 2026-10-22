#!/usr/bin/env python3
# usage: python3 ~/grasp_auto.py t1 [plan|show|go]
#   plan (default): scan with the head and print the plan (only the head moves)
#   show: scan, drive, move gripper around the can (open), wait for Enter, pull back
#   go:   scan, drive, full grasp (lower, extend, close, lift, retract)
# Run after:  python3 ~/aruco_align.py align t1
import sys, os, json, time
sys.path.insert(0, os.path.expanduser('~'))
import numpy as np, rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from can_detect import N, CAL

GRIP_OPEN, GRIP_CLOSE, YAW = 0.13, -0.01, -0.0652
LIFT = {'t1': 0.7447}          # grasp lift per table (table height is fixed)
LIFT_CARRY, RETRACT = 0.86, 0.02
REACH_OFS = 0.07               # measured: gripper stopped 7 cm short
REACH_MIN, REACH_MAX = 0.05, 0.50
SCAN_PANS = (0.0, 0.35, -0.35, 0.7, -0.7)   # head pan offsets (rad) from the calibrated view
SCAN_XW = 0.60                 # search +-60 cm along the table while scanning
MAX_DRIVE = 0.45               # max base drive along the table (m)

class G(N):
    def __init__(s):
        super().__init__(); s.js = {}
        s.create_subscription(JointState, '/stretch/joint_states', lambda m: s.js.update(zip(m.name, m.position)), 1)
        s.traj = ActionClient(s, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        s.pos = s.create_client(Trigger, '/switch_to_position_mode')
        s.nav = s.create_client(Trigger, '/switch_to_navigation_mode')
    def mode(s, cli):
        cli.wait_for_service(timeout_sec=5.0); f = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(s, f, timeout_sec=5.0)
    def go(s, secs=4, **j):
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(j)
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in j.values()]; p.time_from_start.sec = int(secs)
        g.trajectory.points = [p]; s.traj.wait_for_server()
        f = s.traj.send_goal_async(g); rclpy.spin_until_future_complete(s, f)
        h = f.result()
        if not h.accepted: raise SystemExit('rejected ' + str(list(j)))
        r = h.get_result_async(); rclpy.spin_until_future_complete(s, r)
        print('  done', ', '.join(f'{k} {v:+.3f}' for k, v in j.items()))
    def grasp_tf(s):
        for _ in range(20):
            try: return s.tfb.lookup_transform('base_link', 'link_grasp_center', Time()).transform.translation
            except Exception: s.spin(0.3)
        raise SystemExit('no TF for link_grasp_center')
    def detect(s, tz, roi, xw, frames=5):
        r = []
        for _ in range(frames):
            f = s.frame(tz, roi, xw)
            if f is None: return None
            r.append(f)
        return {k: float(np.median([q[k] for q in r])) for k in ('x', 'y', 'top', 'ext')}
    def scan(s, cal):
        hp, ht = cal['head']; tag = cal['tag']
        for off in SCAN_PANS:
            s.go(secs=2, joint_head_pan=hp + off, joint_head_tilt=ht); s.spin(1.0)
            c = s.detect(tag[2], tag[:2], SCAN_XW, frames=2)
            if c: print(f"scan: can found at pan {off:+.2f} (x {c['x']:+.3f}, y {c['y']:+.3f})"); break
        s.go(secs=2, joint_head_pan=hp, joint_head_tilt=ht); s.spin(1.0)
        if not c: raise SystemExit('scan: no can found within ~60 cm of the tag')
        return c
    def plan(s, c):
        s.spin(0.5); g = s.grasp_tf(); ext0 = s.js['wrist_extension']
        dx = c['x'] - g.x                                   # base drives along x
        reach = ext0 + (g.y - c['y']) + REACH_OFS           # arm extends along -y
        print(f"can     x {c['x']:+.3f}  y {c['y']:+.3f}  (height {c['top']*100:.1f} cm, width {c['ext']*100:.1f} cm)")
        print(f"gripper x {g.x:+.3f}  y {g.y:+.3f}  (arm at {ext0:.3f})")
        print(f"PLAN: drive {dx*100:+.1f} cm, reach {reach:.3f}")
        return dx, reach

def main():
    key = sys.argv[1]; cmd = sys.argv[2] if len(sys.argv) > 2 else 'plan'
    cal = json.load(open(CAL))[key]; tag = cal['tag']
    rclpy.init(); a = G(); a.spin(1.5)
    while 'wrist_extension' not in a.js: a.spin(0.2)
    a.mode(a.pos)
    if cmd in ('show', 'go'):
        a.go(joint_lift=LIFT_CARRY); a.go(wrist_extension=RETRACT)      # arm up and in, out of the camera view
        a.go(joint_wrist_yaw=YAW); a.go(joint_gripper_finger_left=GRIP_OPEN); a.spin(1.0)
    c = a.scan(cal)
    dx, reach = a.plan(c)
    if cmd == 'plan':
        print('plan only - only the head moved'); a.mode(a.nav); rclpy.shutdown(); return
    if abs(dx) > MAX_DRIVE: raise SystemExit(f'can is {dx*100:+.0f} cm along - more than {MAX_DRIVE*100:.0f} cm, not driving')
    if abs(dx) > 0.05:          # far: drive close, then measure again from the new spot
        a.go(secs=abs(dx) / 0.05 + 3, translate_mobile_base=dx); a.spin(1.0)
        g = a.grasp_tf()
        c = a.detect(tag[2], (g.x, tag[1]), 0.30)
        if not c: raise SystemExit('lost the can after driving')
        print('re-measured after driving:'); dx, reach = a.plan(c)
    if abs(dx) > 0.15 or not (REACH_MIN < reach < REACH_MAX): raise SystemExit('outside safe limits -> not moving')
    if abs(dx) > 0.005: a.go(secs=abs(dx) / 0.05 + 3, translate_mobile_base=dx)
    a.go(joint_lift=LIFT[key]); a.go(secs=6, wrist_extension=reach)
    if cmd == 'show':
        input('Gripper should be open around the can. Check centering, then press Enter to pull back...')
        a.go(wrist_extension=RETRACT); a.go(joint_lift=LIFT_CARRY)
    else:
        a.go(joint_gripper_finger_left=GRIP_CLOSE); a.spin(1.0)
        a.go(joint_lift=LIFT_CARRY); a.go(wrist_extension=RETRACT)
        print('GRASPED')
    a.mode(a.nav)
    rclpy.shutdown()

if __name__ == '__main__': main()
