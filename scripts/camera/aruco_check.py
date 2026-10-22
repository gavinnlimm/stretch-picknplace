import cv2, numpy as np, rclpy
from sensor_msgs.msg import Image
D = {n: getattr(cv2.aruco, n) for n in ['DICT_4X4_50','DICT_5X5_100','DICT_6X6_250','DICT_7X7_1000','DICT_ARUCO_ORIGINAL','DICT_APRILTAG_36h11']}
rclpy.init(); node = rclpy.create_node('aruco_check'); imgs = []
node.create_subscription(Image, '/camera/color/image_raw', lambda m: imgs.append(m), 1)
while not imgs: rclpy.spin_once(node, timeout_sec=1.0)
m = imgs[-1]
img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if img.shape[2] == 3 else img[:,:,0]
cv2.imwrite('/home/hello-robot/cam.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
print(f'image {m.width}x{m.height} {m.encoding}')
found = False
for name, d in D.items():
    dic = cv2.aruco.getPredefinedDictionary(d)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dic).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic)
    if ids is not None:
        found = True
        for c, i in zip(corners, ids.ravel()):
            side = np.mean([np.linalg.norm(c[0][k]-c[0][(k+1)%4]) for k in range(4)])
            print(f'{name}: id {i}, ~{side:.0f} px wide')
if not found: print('no markers found - point the head at the tag (camera view saved to ~/cam.png)')
