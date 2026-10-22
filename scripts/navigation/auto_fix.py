#!/usr/bin/env python3
# usage: python3 ~/auto_fix.py t1|t2
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

GOALS = {'t1': (-1.146, 0.345, math.radians(-4.2)),
         't2': (-0.660, 1.682, math.radians(-178.9))}
MAX_TURN = math.radians(25)   # bigger sideways fixes are left for the arm

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Fix(Node):
    def __init__(self):
        super().__init__('auto_fix')
        self.pose = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb, qos)
        self.nomo = self.create_client(Empty, '/request_nomotion_update')
        self.pos_mode = self.create_client(Trigger, '/switch_to_position_mode')
        self.nav_mode = self.create_client(Trigger, '/switch_to_navigation_mode')
        self.traj = ActionClient(self, FollowJointTrajectory,
                                 '/stretch_controller/follow_joint_trajectory')
    def cb(self, m): self.pose = m.pose.pose
    def call(self, cli, req):
        cli.wait_for_service(timeout_sec=5.0)
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
    def move(self, joint, val, secs):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = [joint]
        p = JointTrajectoryPoint(); p.positions = [float(val)]
        p.time_from_start.sec = int(secs)
        g.trajectory.points = [p]
        self.traj.wait_for_server()
        f = self.traj.send_goal_async(g); rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted: raise RuntimeError('goal rejected')
        r = h.get_result_async(); rclpy.spin_until_future_complete(self, r)
        print(f'  done {joint} {val:+.3f}: {r.result().result.error_string}')

def main():
    key = sys.argv[1] if len(sys.argv) > 1 else ''
    if key not in GOALS: print('usage: python3 ~/auto_fix.py t1|t2'); return
    gx, gy, gyaw = GOALS[key]
    rclpy.init(); n = Fix()
    print('refreshing AMCL (4x)...')
    for _ in range(4):
        n.call(n.nomo, Empty.Request()); time.sleep(0.3)
    t0 = time.time()
    while time.time() - t0 < 1.0: rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None: print('no /amcl_pose received'); return
    p = n.pose
    yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    dx, dy = p.position.x - gx, p.position.y - gy
    ahead = dx * math.cos(gyaw) + dy * math.sin(gyaw)
    left = -dx * math.sin(gyaw) + dy * math.cos(gyaw)
    tilt = wrap(yaw - gyaw)
    print(f'AMCL: {ahead*100:+.1f} cm ahead, {left*100:+.1f} cm left, tilt {math.degrees(tilt):+.1f} deg')
    if ahead >= 0: h = math.atan2(left, ahead); d = -math.hypot(ahead, left)
    else: h = math.atan2(-left, -ahead); d = math.hypot(ahead, left)
    if abs(left) >= 0.02 and abs(h) > MAX_TURN and ahead >= 0 and abs(left) <= 0.35:
        extra = abs(left) / math.tan(MAX_TURN) - ahead + 0.02
        a2 = ahead + extra; h = math.atan2(left, a2); d = -math.hypot(a2, left)
        plan = [('rotate_mobile_base', -tilt), ('translate_mobile_base', extra), ('rotate_mobile_base', h),
                ('translate_mobile_base', d), ('rotate_mobile_base', -h)]; skip = 0.0
    elif abs(h) > MAX_TURN or abs(left) < 0.02:
        plan = [('rotate_mobile_base', -tilt), ('translate_mobile_base', -ahead)]; skip = left
    else:
        plan = [('rotate_mobile_base', wrap(h - tilt)), ('translate_mobile_base', d),
                ('rotate_mobile_base', -h)]; skip = 0.0
    plan = [(j, v) for j, v in plan
            if abs(v) > (math.radians(1) if 'rotate' in j else 0.01)]
    print('PLAN:')
    for j, v in plan:
        print(f'  turn {math.degrees(v):+.1f} deg (+ = left)' if 'rotate' in j
              else f'  drive {v*100:+.1f} cm (- = backward)')
    if abs(skip) >= 0.01: print(f'  sideways {skip*100:+.1f} cm NOT fixed (arm covers it)')
    if not plan: print('already within tolerance'); return
    safe = all(abs(v) <= 0.90 for j, v in plan if 'translate' in j) and ('translate' in plan[-1][0] or abs(plan[-1][1]) <= math.radians(25))
    if '--yes' in sys.argv and safe: print('plan within limits -> running')
    elif '--yes' in sys.argv: print('plan outside safe limits -> stopping'); sys.exit(1)
    elif input('Run-stop SOLID, hand ready. Run? [y/N] ').strip().lower() != 'y':
        print('cancelled'); return
    n.call(n.pos_mode, Trigger.Request())
    try:
        for j, v in plan:
            secs = int(abs(v) / (0.2 if 'rotate' in j else 0.10)) + 1
            n.move(j, v, secs)
    finally:
        n.call(n.nav_mode, Trigger.Request())
    rclpy.shutdown()

if __name__ == '__main__': main()
