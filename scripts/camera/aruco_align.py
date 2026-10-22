#!/usr/bin/env python3
# usage: python3 ~/aruco_align.py calib t1|t2   or   align t1|t2
import sys, os, json, time
import numpy as np, cv2, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

TAGS = {'t1': 8, 't2': 7}
SIZE = 0.08
CAL = os.path.expanduser('~/aruco_cal.json')

def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class A(Node):
    def __init__(s):
        super().__init__('aruco_align')
        s.img = None; s.info = None; s.js = {}
        s.create_subscription(Image, '/camera/color/image_raw', lambda m: setattr(s, 'img', m), 1)
        s.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: setattr(s, 'info', m), 1)
        s.create_subscription(JointState, '/stretch/joint_states', lambda m: s.js.update(zip(m.name, m.position)), 1)
        s.tfb = tf2_ros.Buffer(); s.tfl = tf2_ros.TransformListener(s.tfb, s)
        s.traj = ActionClient(s, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        s.pos = s.create_client(Trigger, '/switch_to_position_mode')
        s.nav = s.create_client(Trigger, '/switch_to_navigation_mode')
    def spin(s, t):
        t0 = time.time()
        while time.time() - t0 < t: rclpy.spin_once(s, timeout_sec=0.05)
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
    def tag(s, tid, n=10):
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        h = SIZE / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        pts = []
        for _ in range(n * 5):
            s.img = None
            while s.img is None or s.info is None: rclpy.spin_once(s, timeout_sec=0.1)
            m = s.img
            gray = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1), cv2.COLOR_RGB2GRAY)
            if hasattr(cv2.aruco, 'ArucoDetector'): c, ids, _ = cv2.aruco.ArucoDetector(dic).detectMarkers(gray)
            else: c, ids, _ = cv2.aruco.detectMarkers(gray, dic)
            if ids is None or tid not in ids.ravel(): continue
            k = list(ids.ravel()).index(tid)
            K = np.array(s.info.k).reshape(3, 3); D = np.array(s.info.d)
            ok, rv, tv = cv2.solvePnP(obj, c[k].reshape(4, 2).astype(np.float32), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: continue
            try: t = s.tfb.lookup_transform('base_link', m.header.frame_id, Time())
            except Exception as e: print('waiting for TF...'); s.spin(0.3); continue
            T = t.transform.translation
            pts.append(quat_R(t.transform.rotation) @ tv.ravel() + np.array([T.x, T.y, T.z]))
            if len(pts) >= n: break
        if not pts: return None
        return np.median(np.array(pts), axis=0)

def main():
    cmd, key = sys.argv[1], sys.argv[2]; tid = TAGS[key]
    rclpy.init(); a = A(); a.spin(1.5)
    cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
    if cmd == 'calib':
        ext, lift = a.js['wrist_extension'], a.js['joint_lift']
        hp, ht = a.js['joint_head_pan'], a.js['joint_head_tilt']
        input('Gripper is around the can. If the can covers the tag, take it away now. Press Enter...')
        a.mode(a.pos); a.go(joint_gripper_finger_left=0.13); a.go(wrist_extension=0.02); a.go(joint_lift=0.86)
        a.spin(1.0); m = a.tag(tid)
        if m is None: raise SystemExit('tag not seen - point the head at it')
        cal[key] = {'tag': m.tolist(), 'ext': ext, 'lift': lift, 'head': [hp, ht]}
        json.dump(cal, open(CAL, 'w'), indent=1)
        print(f'saved {key}: tag x {m[0]:.3f} y {m[1]:.3f}, reach {ext:.3f}, lift {lift:.3f}')
        a.mode(a.nav)
    elif cmd == 'align':
        c = cal[key]
        a.mode(a.pos); a.go(wrist_extension=0.02); a.go(joint_lift=0.76 if key == 't2' else 0.86)
        m = None
        for dp, dt in [(0,0),(-0.15,0),(0.15,0),(0,-0.15),(-0.3,0),(0.3,0),(-0.15,-0.15),(0.15,-0.15)]:
            a.go(secs=2, joint_head_pan=c['head'][0]+dp); a.go(secs=2, joint_head_tilt=c['head'][1]+dt); a.spin(0.8)
            m = a.tag(tid)
            if m is not None: break
            print('tag not seen, moving head...')
        if m is None: raise SystemExit(f'tag {tid} not seen after head search')
        dx, dy = m[0] - c['tag'][0], m[1] - c['tag'][1]
        reach = c['ext'] - dy
        print(f'camera: along {dx*100:+.1f} cm, sideways {dy*100:+.1f} cm -> drive {dx*100:+.1f} cm, reach {reach:.3f}')
        if abs(dx) > 0.40 or not (0.0 < reach < 0.50): raise SystemExit('too far off - not moving')
        if abs(dx) > 0.005: a.go(secs=abs(dx) / 0.10 + 1, translate_mobile_base=dx)
        json.dump({'reach': reach, 'lift': c['lift']}, open(os.path.expanduser(f'~/align_{key}.json'), 'w'))
        a.mode(a.nav)
    rclpy.shutdown()

if __name__ == '__main__': main()
