#!/usr/bin/env python3
# usage: /usr/bin/python3 ~/demo_logger.py <episode_dir> [--wrist-topic <ros image topic>]
#        (full_auto_obj.sh and record_demo.sh start and stop it)
# Records one demo for VLA (pi0.5) fine-tuning: the head camera color image (resized) + all joint states
# + base odometry + the current pipeline stage, 10 times a second, until stopped (Ctrl+C / SIGINT / SIGTERM).
# Output: <episode_dir>/frames/00000.jpg ... (head), <episode_dir>/wrist/00000.jpg ... (only with --wrist-topic),
#         and <episode_dir>/steps.jsonl (one line per frame).
import sys, os, json, time, signal
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry

HZ = 10
SIZE = (320, 240)                       # saved image size (w, h); pi0/pi0.5 resize to ~224 anyway
STAGE = os.path.expanduser('~/demo_stage')   # full_auto_obj.sh writes the current stage name here

def to_bgr(m):
    """ROS Image (rgb8/bgr8) -> BGR array, or None if the encoding isn't supported"""
    if m.encoding not in ('rgb8', 'bgr8'): return None
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a

class Logger(Node):
    def __init__(s, out, wrist_topic=None):
        super().__init__('demo_logger')
        s.out = out; s.img = s.js = s.odom = s.wimg = None; s.n = 0; s.wrist = wrist_topic
        os.makedirs(os.path.join(out, 'frames'), exist_ok=True)
        if s.wrist:
            os.makedirs(os.path.join(out, 'wrist'), exist_ok=True)
            s.create_subscription(Image, s.wrist, lambda m: setattr(s, 'wimg', m), 1)
        s.f = open(os.path.join(out, 'steps.jsonl'), 'a', buffering=1)   # line-buffered: every frame is on disk at once
        s.create_subscription(Image, '/camera/color/image_raw', lambda m: setattr(s, 'img', m), 1)
        s.create_subscription(JointState, '/stretch/joint_states', lambda m: setattr(s, 'js', m), 1)
        s.create_subscription(Odometry, '/odom', lambda m: setattr(s, 'odom', m), 1)
        s.create_timer(1.0 / HZ, s.tick)
    def tick(s):
        if s.img is None or s.js is None: return
        if s.wrist and s.wimg is None: return            # wait until the wrist camera is streaming too
        a = to_bgr(s.img)
        if a is None:
            if s.n == 0: print('unsupported head image encoding', s.img.encoding); s.n = -1
            return
        name = f'frames/{s.n:05d}.jpg'
        cv2.imwrite(os.path.join(s.out, name), cv2.resize(a, SIZE, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 90])
        wname = None
        if s.wrist:
            w = to_bgr(s.wimg)
            if w is not None:
                wname = f'wrist/{s.n:05d}.jpg'
                cv2.imwrite(os.path.join(s.out, wname), cv2.resize(w, SIZE, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 90])
        o = s.odom.pose.pose if s.odom else None
        try: stage = open(STAGE).read().strip()
        except OSError: stage = ''
        s.f.write(json.dumps({'t': round(time.time(), 3), 'frame': name, 'wrist': wname, 'stage': stage,
                              'joints': {k: round(v, 5) for k, v in zip(s.js.name, s.js.position)},
                              'odom': None if o is None else [round(o.position.x, 4), round(o.position.y, 4),
                                                              round(o.orientation.z, 5), round(o.orientation.w, 5)]}) + '\n')
        s.n += 1

def main():
    out = os.path.expanduser(sys.argv[1])
    wrist = sys.argv[sys.argv.index('--wrist-topic') + 1] if '--wrist-topic' in sys.argv else None
    rclpy.init(); n = Logger(out, wrist)
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try: rclpy.spin(n)
    except BaseException: pass
    finally:
        n.f.close(); print(f'demo_logger: {max(n.n, 0)} frames -> {out}')

if __name__ == '__main__': main()
