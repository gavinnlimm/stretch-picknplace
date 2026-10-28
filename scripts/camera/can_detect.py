#!/usr/bin/env python3
# usage: python3 ~/can_detect.py t1|t2 [frames]    (prints only, never moves anything)
# Run right after:  python3 ~/aruco_align.py align t1   (head is then aimed at the tag)
import sys, os, json, time
import numpy as np, cv2, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo

CAL = os.path.expanduser('~/aruco_cal.json')
R_CAN = 0.033          # can radius (m) - only used with ADAPT=0
ADAPT = os.environ.get('ADAPT') != '0'   # size-based grasp (ADAPT=0 -> old can-only values)
H_MIN, H_MAX = 0.02, 0.30   # keep points this far above the table (m)
TOP_OK, W_MAX = (0.08, 0.25), 0.11   # can-like: top height range, max visible width

def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def find_can(P, tz, tag_xy, cam_xy, xw=0.30):
    """P: Nx3 points in base_link. Returns dict or None."""
    roi = (np.abs(P[:, 0] - tag_xy[0]) < xw) & (P[:, 1] > tag_xy[1] - 0.30) & (P[:, 1] < -0.30)
    S = P[roi & (np.abs(P[:, 2] - tz) < 0.10)]
    plane = None
    if len(S) > 200:      # fit the table plane from the depth itself (handles tag-z error and tilt)
        hist, ed = np.histogram(S[:, 2], bins=np.arange(tz - 0.10, tz + 0.1001, 0.005))
        z0 = ed[np.argmax(hist)] + 0.0025
        I = S[np.abs(S[:, 2] - z0) < 0.015]
        for _ in range(3):
            A = np.c_[I[:, 0], I[:, 1], np.ones(len(I))]
            co = np.linalg.lstsq(A, I[:, 2], rcond=None)[0]
            r = np.abs(np.c_[S[:, 0], S[:, 1], np.ones(len(S))] @ co - S[:, 2])
            I = S[r < 0.01]
            if len(I) < 100: break
        plane = co
    if plane is None: plane = np.array([0.0, 0.0, tz])
    h = P[:, 2] - (P[:, 0] * plane[0] + P[:, 1] * plane[1] + plane[2])
    ok = roi & (h > H_MIN) & (h < H_MAX)
    Q = P[ok]; hq = h[ok]
    if len(Q) < 30: return None
    x0, y0 = Q[:, 0].min() - 0.02, Q[:, 1].min() - 0.02
    ix = ((Q[:, 0] - x0) * 100).astype(int); iy = ((Q[:, 1] - y0) * 100).astype(int)
    g = np.zeros((iy.max() + 3, ix.max() + 3), np.uint8); g[iy + 1, ix + 1] = 255
    g = cv2.dilate(g, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(g)
    L = lab[iy + 1, ix + 1]
    best = None
    for k in range(1, n):
        C = Q[L == k]; hc = hq[L == k]
        if len(C) < 30: continue
        c = C[:, :2].mean(0)
        ev, evec = np.linalg.eigh(np.cov((C[:, :2] - c).T))
        ext = np.ptp((C[:, :2] - c) @ evec[:, 1]); top = np.percentile(hc, 98)
        if os.environ.get('CAN_DEBUG'): print(f'  component {k}: points {len(C)}, top {top*100:.1f} cm, width {ext*100:.1f} cm')
        if TOP_OK[0] < top < TOP_OK[1] and ext < W_MAX and (best is None or len(C) > best[0]):
            best = (len(C), C, c, ext, top)
    if best is None: return None
    _, C, c, ext, top = best
    u = c - np.array(cam_xy); u /= (np.linalg.norm(u) + 1e-9)
    r = float(np.clip(ext / 2, 0.015, 0.05)) if ADAPT else R_CAN   # object radius from its visible width
    center = c + 0.64 * r * u              # visible arc centroid sits ~0.64r in front of the axis
    return {'x': center[0], 'y': center[1], 'top': top, 'n': len(C), 'ext': ext, 'r': r,
            'tz': plane[2] + plane[0] * c[0] + plane[1] * c[1]}

class N(Node):
    def __init__(s):
        super().__init__('can_detect')
        s.d = None; s.info = None
        s.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', lambda m: setattr(s, 'd', m), 1)
        s.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: setattr(s, 'info', m), 1)
        s.tfb = tf2_ros.Buffer(); s.tfl = tf2_ros.TransformListener(s.tfb, s)
    def spin(s, t):
        t0 = time.time()
        while time.time() - t0 < t: rclpy.spin_once(s, timeout_sec=0.05)
    def frame(s, tz, tag_xy, xw=0.30):
        s.d = None
        t0 = time.time()
        while s.d is None and time.time() - t0 < 5: rclpy.spin_once(s, timeout_sec=0.1)
        if s.d is None or s.info is None:
            print('no depth image. camera topics I can see:')
            for nme, _ in s.get_topic_names_and_types():
                if 'camera' in nme and ('depth' in nme or 'info' in nme): print('  ', nme)
            raise SystemExit('is the camera running (Tab F)?')
        m = s.d
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)[::2, ::2].astype(np.float32) * 0.001
        v, u = np.mgrid[0:m.height:2, 0:m.width:2]
        K = np.array(s.info.k).reshape(3, 3); sc = m.width / s.info.width
        fx, fy, cx, cy = K[0, 0] * sc, K[1, 1] * sc, K[0, 2] * sc, K[1, 2] * sc
        k = (d > 0.2) & (d < 1.5)
        z = d[k]; P = np.stack([(u[k] - cx) * z / fx, (v[k] - cy) * z / fy, z], 1)
        for _ in range(20):
            try: t = s.tfb.lookup_transform('base_link', m.header.frame_id, Time()); break
            except Exception: s.spin(0.3)
        else: raise SystemExit('no TF to base_link')
        T = t.transform.translation
        P = P @ quat_R(t.transform.rotation).T + np.array([T.x, T.y, T.z])
        try:   # ignore the robot's own gripper
            gc = s.tfb.lookup_transform('base_link', 'link_grasp_center', Time()).transform.translation
            P = P[np.hypot(P[:, 0] - gc.x, P[:, 1] - gc.y) > 0.10]
        except Exception: pass
        return find_can(P, tz, tag_xy, (T.x, T.y), xw)

def main():
    key = sys.argv[1]; frames = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    cal = json.load(open(CAL))[key]; tag = cal['tag']
    rclpy.init(); n = N(); n.spin(1.5)
    r = []
    while len(r) < frames:
        f = n.frame(tag[2], tag[:2])
        if f is None: print('no can-like object in the search area'); r = []; break
        r.append(f)
    if r:
        x = np.array([f['x'] for f in r]); y = np.array([f['y'] for f in r])
        top = np.median([f['top'] for f in r]); ext = np.median([f['ext'] for f in r]); npt = int(np.median([f['n'] for f in r]))
        cx, cy = float(np.median(x)), float(np.median(y))
        print(f'can center in base_link: x {cx:.3f}  y {cy:.3f}   (spread {x.std()*1000:.1f} / {y.std()*1000:.1f} mm over {frames} frames)')
        print(f'relative to tag {tag_id(key)}: {100*(cx-tag[0]):+.1f} cm along, {100*(cy-tag[1]):+.1f} cm sideways')
        print(f'height above table {top*100:.1f} cm (can ~12), visible width {ext*100:.1f} cm (~5-7), points {npt}')
        print(f'table height from depth {np.median([f["tz"] for f in r]):.3f} m vs tag {tag[2]:.3f} m')
        if not (0.09 < top < 0.15): print('WARNING: height is off - this may not be the can')
        if ext > 0.12: print('WARNING: object is too wide - maybe two objects in view')
        json.dump({'x': cx, 'y': cy, 'top': top, 'time': time.time()}, open(os.path.expanduser('~/can_last.json'), 'w'))
    rclpy.shutdown()

def tag_id(key): return {'t1': 8, 't2': 7}[key]

if __name__ == '__main__': main()
