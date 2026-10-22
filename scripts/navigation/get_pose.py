import math, time, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n = rclpy.create_node('get_pose'); last = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: last.append(m.pose.pose), qos)
c = n.create_client(Empty, '/request_nomotion_update'); c.wait_for_service(timeout_sec=5.0)
for _ in range(10):
    rclpy.spin_until_future_complete(n, c.call_async(Empty.Request()), timeout_sec=2.0); time.sleep(0.5)
t0 = time.time()
while time.time() - t0 < 2.0: rclpy.spin_once(n, timeout_sec=0.1)
if not last: print('no pose received'); raise SystemExit
p = last[-1]; yaw = math.degrees(2*math.atan2(p.orientation.z, p.orientation.w))
print(f'x {p.position.x:.3f}  y {p.position.y:.3f}  heading {yaw:.1f} deg  (z {p.orientation.z:.4f}, w {p.orientation.w:.4f})  [{len(last)} updates]')
