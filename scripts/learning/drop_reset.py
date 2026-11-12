#!/usr/bin/env python3
"""
Reset the robot for the next drop trial - no gamepad, no driver restart.
Driver must be running WITHOUT mode:=gamepad (the same driver the policy client uses).

    /usr/bin/python3 ~/drop_reset.py --back 0.35        # back up 35 cm (vary 0.25 - 0.50 between trials)

Steps (asks for Enter before each one; Ctrl+C to stop):
  1. open the gripper, you put the can between the fingers, Enter, it closes on the can
  2. lift to carry height, arm retracted, wrist straight
  3. drive the base straight back by --back metres (check behind the robot first)
  4. head to the drop demo pose (pan -1.38, tilt -0.60)
Then run policy_client.py again.
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

GRIP_OPEN, GRIP_HOLD = 0.10, -0.21      # -0.21 = how tightly the can was held in the drop demos
LIFT, ARM, YAW = 0.78, 0.02, 0.0        # start pose like the drop demos (arm 2 cm out, not against its end stop)
HEAD = (-1.38, -0.60)


class R(Node):
    def __init__(s):
        super().__init__('drop_reset')
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
    ap.add_argument('--back', type=float, default=0.35, help='metres to drive straight back (0 = do not move the base)')
    ap.add_argument('--hold', type=float, default=GRIP_HOLD,
                    help=f'gripper position when closing on the can (default {GRIP_HOLD}; a value nearer 0, e.g. -0.19, squeezes less)')
    ap.add_argument('--regrip', action='store_true',
                    help='the can is already held (e.g. after the learned pick): tighten the gripper to --hold WITHOUT opening it first, '
                         'so the drop starts with the grip the drop demos had (-0.18 to -0.21; the learned pick holds at about -0.04)')
    a = ap.parse_args()
    if not 0.0 <= a.back <= 0.6: sys.exit('--back must be between 0 and 0.6 m')
    rclpy.init(); n = R(); n.position_mode()

    if a.regrip:
        if ask(f'1. Tighten the gripper on the held can to {a.hold:+.3f} (no opening)?'):
            n.go(3, joint_gripper_finger_left=min(max(a.hold, -0.30), -0.10))
    elif ask('1. Open the gripper?'):
        n.go(3, joint_gripper_finger_left=GRIP_OPEN)
        input('   Put the can between the fingers and hold it there, then press Enter to close...')
        n.go(3, joint_gripper_finger_left=min(max(a.hold, -0.30), -0.10))
    if ask(f'2. Arm in and wrist straight first, then lift to {LIFT}?'):
        n.go(4, wrist_extension=ARM, joint_wrist_yaw=YAW)       # arm out of the way first, at the height it dropped from
        n.go(3, joint_lift=LIFT)                                 # then lower the lift (was one combined move that hit the container)
    if a.back > 0 and ask(f'3. Drive straight BACK {a.back:.2f} m? (behind the robot clear?)'):
        n.go(a.back / 0.10 + 1.0, translate_mobile_base=-a.back)
    if ask(f'4. Head to pan {HEAD[0]}, tilt {HEAD[1]}?'): n.go(3, joint_head_pan=HEAD[0], joint_head_tilt=HEAD[1])
    print('Ready. Run: /usr/bin/python3 ~/policy_client.py --server http://localhost:8000 --module drop --go --seconds 90')


if __name__ == '__main__':
    main()
