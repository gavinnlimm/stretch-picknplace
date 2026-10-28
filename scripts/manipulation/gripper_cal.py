#!/usr/bin/env python3
# usage: python3 ~/gripper_cal.py
# Moves ONLY the gripper to a few positions. At each one, measure the gap between the finger pads (cm)
# with a ruler and type it. Saves ~/gripper_cal.json, which grasp_auto.py uses to close to the
# object's measured width (minus a small squeeze) instead of closing all the way.
import os, json, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

POS = [0.13, 0.10, 0.07, 0.04, 0.02, 0.0]     # gripper positions to measure (open -> closed)
J = 'joint_gripper_finger_left'
OUT = os.path.expanduser('~/gripper_cal.json')

class C(Node):
    def __init__(s):
        super().__init__('gripper_cal'); s.js = {}
        s.create_subscription(JointState, '/stretch/joint_states', lambda m: s.js.update(zip(m.name, m.position)), 1)
        s.traj = ActionClient(s, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        s.pos = s.create_client(Trigger, '/switch_to_position_mode')
        s.nav = s.create_client(Trigger, '/switch_to_navigation_mode')
    def spin(s, t):
        t0 = time.time()
        while time.time() - t0 < t: rclpy.spin_once(s, timeout_sec=0.05)
    def mode(s, cli):
        cli.wait_for_service(timeout_sec=5.0); f = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(s, f, timeout_sec=5.0)
    def grip(s, v):
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = [J]
        p = JointTrajectoryPoint(); p.positions = [float(v)]; p.time_from_start.sec = 2
        g.trajectory.points = [p]; s.traj.wait_for_server()
        f = s.traj.send_goal_async(g); rclpy.spin_until_future_complete(s, f)
        h = f.result()
        if not h.accepted: raise SystemExit(f'gripper move to {v} rejected')
        r = h.get_result_async(); rclpy.spin_until_future_complete(s, r); s.spin(0.5)

def main():
    rclpy.init(); n = C(); n.spin(1.0); n.mode(n.pos)
    gaps, poss = [], []
    try:
        for v in POS:
            n.grip(v); actual = n.js.get(J, v)
            t = input(f'gripper at {actual:+.3f} (asked {v:+.3f}). Gap between finger pads in cm (Enter = skip): ').strip()
            if t: gaps.append(float(t) / 100); poss.append(float(v))   # save the commanded position (the one grasp_auto.py sends)
    finally:
        n.grip(POS[0]); n.mode(n.nav)
    if len(gaps) >= 3:
        json.dump({'gap': gaps, 'pos': poss, 'time': time.time()}, open(OUT, 'w'), indent=1)
        print(f'saved {len(gaps)} points to {OUT}:'); [print(f'  gap {g*100:4.1f} cm  <->  position {p:+.3f}') for g, p in zip(gaps, poss)]
    else: print('need at least 3 measurements - nothing saved')
    rclpy.shutdown()

if __name__ == '__main__': main()
