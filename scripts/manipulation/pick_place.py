#!/usr/bin/env python3
# usage: python3 ~/pick_place.py pick | place | show | stow | full
import sys, os, json, time, subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

GRIP_OPEN, GRIP_CLOSE = 0.13, -0.01
YAW = -0.0652
LIFT_GRASP, LIFT_CARRY = 0.7447, 0.86
REACH_PICK = 0.336
LIFT_PLACE, REACH_PLACE, LIFT_CLEAR = 0.748, 0.336, 0.86
RETRACT = 0.02

def grasp_offset():
    """how much higher than LIFT_GRASP the last grasp was (size-based grasp); 0 if none in the last 15 min or ADAPT=0"""
    p = os.path.expanduser('~/grasp_last.json')
    if os.environ.get('ADAPT') == '0' or not os.path.exists(p) or time.time() - os.path.getmtime(p) > 900: return 0.0
    return max(0.0, json.load(open(p))['lift'] - LIFT_GRASP)

def aligned(key, reach, lift, extra=0.0):
    p = os.path.expanduser(f'~/align_{key}.json')
    if os.path.exists(p) and time.time() - os.path.getmtime(p) < 300:
        d = json.load(open(p)); print(f'  camera alignment: reach {d["reach"]:.3f}, lift {d["lift"]+extra:.3f}')
        return d['reach'], d['lift'] + extra
    return reach, lift

class Arm(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.traj = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        self.pos = self.create_client(Trigger, '/switch_to_position_mode')
        self.nav = self.create_client(Trigger, '/switch_to_navigation_mode')
    def mode(self, cli):
        cli.wait_for_service(timeout_sec=5.0)
        f = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
    def go(self, label, **joints):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(joints)
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in joints.values()]
        p.time_from_start.sec = 4
        g.trajectory.points = [p]
        self.traj.wait_for_server()
        f = self.traj.send_goal_async(g); rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted: raise RuntimeError(f'{label}: rejected')
        r = h.get_result_async(); rclpy.spin_until_future_complete(self, r)
        print(f'  {label}: {r.result().result.error_string}')

def pick(a):
    print('PICK'); r, l = aligned('t1', REACH_PICK, LIFT_GRASP); a.mode(a.pos)
    a.go('open gripper', joint_gripper_finger_left=GRIP_OPEN)
    a.go('retract', wrist_extension=RETRACT)
    a.go('wrist', joint_wrist_yaw=YAW)
    a.go('lower to grasp', joint_lift=l)
    a.go('extend', wrist_extension=r)
    a.go('close', joint_gripper_finger_left=GRIP_CLOSE)
    a.go('lift', joint_lift=LIFT_CARRY)
    a.go('retract', wrist_extension=RETRACT)
    a.mode(a.nav)

def place(a):
    print('PLACE'); r, l = aligned('t2', REACH_PLACE, LIFT_PLACE, 0.003); a.mode(a.pos)
    up = grasp_offset(); l += up
    if up: print(f'  object was grasped {up*100:.1f} cm higher -> placing {up*100:.1f} cm higher (lift {l:.3f})')
    a.go('wrist', joint_wrist_yaw=YAW)
    a.go('raise', joint_lift=max(LIFT_CARRY, l + 0.06))
    a.go('extend', wrist_extension=r)
    a.go('lower to place', joint_lift=l)
    a.go('release', joint_gripper_finger_left=GRIP_OPEN)
    a.go('lift clear', joint_lift=max(LIFT_CLEAR, l + 0.06))
    a.go('retract', wrist_extension=RETRACT)
    a.mode(a.nav)

def run(cmd):
    print('>>', cmd)
    if subprocess.call(['bash', '-c', cmd]) != 0: raise SystemExit(cmd + ' failed')

if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    rclpy.init(); a = Arm()
    if cmd == 'pick': pick(a)
    elif cmd == 'place': place(a)
    elif cmd == 'show':
        a.mode(a.pos); a.go('open gripper', joint_gripper_finger_left=GRIP_OPEN); a.go('wrist', joint_wrist_yaw=YAW)
        a.go('lift', joint_lift=LIFT_GRASP); a.go('extend', wrist_extension=REACH_PICK)
    elif cmd == 'stow':
        a.mode(a.pos); a.go('retract', wrist_extension=RETRACT); a.go('lift', joint_lift=LIFT_CARRY); a.mode(a.nav)
    elif cmd == 'full':
        print('FULL RUN - hand on the run-stop'); time.sleep(2)
        if True:
            a.mode(a.nav)
            run('bash ~/t1.sh'); run('/usr/bin/python3 ~/aruco_align.py align t1'); pick(a)
            run('bash ~/t2.sh'); run('/usr/bin/python3 ~/aruco_align.py align t2'); place(a)
    else: print('usage: python3 ~/pick_place.py pick | place | show | stow | full')
    rclpy.shutdown()
