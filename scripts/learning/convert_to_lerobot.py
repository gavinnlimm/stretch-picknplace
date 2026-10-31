#!/usr/bin/env python3
"""
Convert recorded Stretch demos (~/demos/<episode>/, made by demo_logger.py) into a LeRobot dataset
for training Diffusion Policy or VQ-BeT with LeRobot.

Run on the GPU computer (or anywhere LeRobot 0.6.x is installed), after copying ~/demos from the robot:
    python convert_to_lerobot.py --demos ~/demos --out ~/datasets/stretch_grasp --stages grasp,teleop_grasp

What it builds, per frame (10 Hz, same as the recorder):
    observation.images.head   head camera image (240x320 RGB)
    observation.images.wrist  wrist camera image  (only with --wrist)
    observation.state         [base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt]
    action                    [base_x, lift, arm, wrist_yaw, gripper] of the NEXT frame (absolute targets)
    task                      e.g. "pick up the soup can"
base_x = how far the base has moved forward since the start of the episode (m), from odometry.
Head pan/tilt are observed but not commanded (keep the head still during demos).
"""
import argparse, json, math, os, sys
from pathlib import Path
import numpy as np
from PIL import Image

STATE_NAMES = ['base_x', 'lift', 'arm', 'wrist_yaw', 'gripper', 'head_pan', 'head_tilt']
ACTION_NAMES = ['base_x', 'lift', 'arm', 'wrist_yaw', 'gripper']
JOINTS = {'lift': 'joint_lift', 'arm': 'wrist_extension', 'wrist_yaw': 'joint_wrist_yaw',
          'gripper': 'joint_gripper_finger_left', 'head_pan': 'joint_head_pan', 'head_tilt': 'joint_head_tilt'}
H, W = 240, 320


def load_episode(ep: Path, stages, wrist):
    """Return (rows, meta) for one episode folder, keeping only frames in the chosen stages."""
    steps = ep / 'steps.jsonl'
    if not steps.exists(): return [], {}
    meta = json.loads((ep / 'meta.json').read_text()) if (ep / 'meta.json').exists() else {}
    rows = []
    for line in steps.read_text().splitlines():
        try: r = json.loads(line)
        except json.JSONDecodeError: continue          # a half-written last line if the logger was killed
        if stages and r.get('stage') not in stages: continue
        if wrist and not r.get('wrist'): continue
        rows.append(r)
    return rows, meta


def base_forward(rows):
    """Forward distance (m) of the base along its starting heading, per row, from odometry."""
    o0 = next((r['odom'] for r in rows if r.get('odom')), None)
    if o0 is None: return [0.0] * len(rows)
    yaw0 = 2 * math.atan2(o0[2], o0[3])
    out, last = [], 0.0
    for r in rows:
        if r.get('odom'):
            x, y = r['odom'][0] - o0[0], r['odom'][1] - o0[1]
            last = x * math.cos(yaw0) + y * math.sin(yaw0)
        out.append(last)
    return out


def state_vector(r, bx):
    j = r['joints']
    return [bx] + [float(j.get(JOINTS[k], 0.0)) for k in STATE_NAMES[1:]]


def load_image(path: Path):
    im = Image.open(path).convert('RGB')
    if im.size != (W, H): im = im.resize((W, H))
    return np.asarray(im, dtype=np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--demos', default='~/demos', help='folder with one subfolder per recorded demo')
    ap.add_argument('--out', required=True, help='output dataset folder (must not exist yet)')
    ap.add_argument('--repo-id', default='local/stretch_demos', help='dataset name (only used locally)')
    ap.add_argument('--stages', default='grasp,teleop_grasp',
                    help='comma-separated stages to keep ("" = whole episode)')
    ap.add_argument('--task', default='pick up the {object}', help='task text; {object} is filled in')
    ap.add_argument('--include-failed', action='store_true', help='also keep demos whose result is not OK')
    ap.add_argument('--source', choices=['all', 'teleop', 'scripted'], default='all')
    ap.add_argument('--wrist', action='store_true', help='include the wrist camera (frames without it are skipped)')
    ap.add_argument('--videos', action='store_true', help='store images as video (smaller; needs ffmpeg)')
    ap.add_argument('--min-frames', type=int, default=20)
    a = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    stages = set(s for s in a.stages.split(',') if s)
    eps = sorted(p for p in Path(os.path.expanduser(a.demos)).iterdir() if p.is_dir())
    img_type = 'video' if a.videos else 'image'
    features = {
        'observation.images.head': {'dtype': img_type, 'shape': (H, W, 3), 'names': ['height', 'width', 'channels']},
        'observation.state': {'dtype': 'float32', 'shape': (len(STATE_NAMES),), 'names': STATE_NAMES},
        'action': {'dtype': 'float32', 'shape': (len(ACTION_NAMES),), 'names': ACTION_NAMES},
    }
    if a.wrist:
        features['observation.images.wrist'] = {'dtype': img_type, 'shape': (H, W, 3), 'names': ['height', 'width', 'channels']}
    ds = LeRobotDataset.create(repo_id=a.repo_id, fps=10, features=features, root=os.path.expanduser(a.out),
                               robot_type='stretch_re1', use_videos=a.videos)

    kept, skipped = 0, []
    for ep in eps:
        rows, meta = load_episode(ep, stages, a.wrist)
        src = meta.get('source', 'teleop' if '_teleop_' in ep.name else 'scripted')
        why = None
        if not meta: why = 'no meta.json (run unfinished?)'
        elif meta.get('result') != 'OK' and not a.include_failed: why = f"result {meta.get('result')}"
        elif a.source != 'all' and src != a.source: why = f'source {src}'
        elif len(rows) < a.min_frames: why = f'{len(rows)} frames in stages {sorted(stages) or "all"}'
        if why: skipped.append((ep.name, why)); continue

        bx = base_forward(rows)
        states = [state_vector(r, b) for r, b in zip(rows, bx)]
        task = a.task.format(object=meta.get('object', 'object').replace('_', ' '))
        for i, r in enumerate(rows):
            nxt = states[min(i + 1, len(states) - 1)]          # action = where the robot is one step later
            frame = {'observation.images.head': load_image(ep / r['frame']),
                     'observation.state': np.asarray(states[i], np.float32),
                     'action': np.asarray(nxt[:len(ACTION_NAMES)], np.float32),
                     'task': task}
            if a.wrist: frame['observation.images.wrist'] = load_image(ep / r['wrist'])
            ds.add_frame(frame)
        ds.save_episode()
        kept += 1
        print(f'  + {ep.name}: {len(rows)} frames ({len(rows) / 10:.1f} s), {src}, "{task}"')

    ds.finalize()
    for name, why in skipped: print(f'  - skipped {name}: {why}')
    print(f'\n{kept} episodes, {ds.num_frames} frames -> {a.out}')
    if kept == 0: sys.exit('nothing converted - check --stages / --source / results')


if __name__ == '__main__':
    main()
