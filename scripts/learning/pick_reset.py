#!/usr/bin/env python3
"""
Put the robot in the start pose of the pick demos - no gamepad, no driver restart.
Driver must be running WITHOUT mode:=gamepad (the same driver the policy client uses).

    /usr/bin/python3 ~/pick_reset.py

Steps (asks for Enter before each one; s = skip, q = quit):
  1. arm in, wrist straight (first, so nothing is hit), then lift to the demo start height
  2. gripper half open (the demos started anywhere from -0.12 to +0.05)
  3. head to the pick demo pose (pan -1.29, tilt -0.75)
It does NOT move the base: put the robot at the start position along the table yourself, with the can on the table
in reach (the demos started within arm reach). Then run policy_client.py --module pick.
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

LIFT, ARM, YAW, GRIP = 0.75, 0.03, 0.0, -0.02   # typical start of the pick demos (lift 0.71-0.79, arm 0.01-0.12)
HEAD = (-1.29, -0.75)


class R(Node):
    def __init__(s):
        super().__init__('pick_reset')
        s.traj = ActionClient(s, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        s.pos = s.create_client(Trigger, '/switch_to_position_mode')

    def position_mode(s):
        if not s.pos.wait_for_service(timeout_sec=5.0): sys.exit('driver not running?')
        f = s.pos.call_async(Trigger.Request()); rclpy.spin_until_future_complete(s, f, timeout_sec=5.0)

    def go(s, secs, **joints):
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(joints)
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in joints.values()]
        p.time_from_start.sec = int(secs); p.time_from_start.nanosec = int((secs % 1) * 1e9)
        g.trajectory.points = [p]
        if not s.traj.wait_for_server(timeout_sec=5.0): sys.exit('no trajectory server (driver running?)')
        f = s.traj.send_goal_async(g); rclpy.spin_until_future_complete(s, f); h = f.result()
        if not h.accepted: sys.exit(f'rejected {list(joints)} - is the driver in gamepad mode?')
        r = h.get_result_async(); rclpy.spin_until_future_complete(s, r)
        print('   ', r.result().result.error_string)


def ask(msg):
    """True = do it, False = skip; q stops the script."""
    a = input(f'{msg}  [Enter = do it, s = skip, q = quit] ').strip().lower()
    if a == 'q': sys.exit('stopped')
    return a != 's'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grip', type=float, default=GRIP,
                    help=f'start opening of the gripper (default {GRIP}; the pick demos started between -0.12 and +0.05; up to 0.10 allowed, larger = wider)')
    a = ap.parse_args()
    if not -0.12 <= a.grip <= 0.10: sys.exit('--grip must be between -0.12 and 0.10')
    if a.grip > 0.06: print('note: wider than any pick demo started (max 0.054); the policy has not seen this opening, so watch the first plan')
    rclpy.init(); n = R(); n.position_mode()
    if ask('1. Arm in and wrist straight first, then lift to %.2f?' % LIFT):
        n.go(4, wrist_extension=ARM, joint_wrist_yaw=YAW)
        n.go(3, joint_lift=LIFT)
    if ask('2. Gripper to %+.2f (open)?' % a.grip): n.go(3, joint_gripper_finger_left=a.grip)
    if ask(f'3. Head to pan {HEAD[0]}, tilt {HEAD[1]}?'): n.go(3, joint_head_pan=HEAD[0], joint_head_tilt=HEAD[1])
    print('Ready. Place the robot at the start, can on the table, then: /usr/bin/python3 ~/policy_client.py --server http://localhost:8000 --module pick --go --seconds 90 --base-mode relative')


if __name__ == '__main__':
    main()
