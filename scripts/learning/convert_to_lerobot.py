#!/usr/bin/env python3
"""
Convert recorded Stretch demos (~/demos/<episode>/, made by demo_logger.py) into a LeRobot dataset
for training ACT (or Diffusion Policy / VQ-BeT) with LeRobot.

Run on the GPU computer (or anywhere LeRobot 0.6.x is installed), after copying ~/demos from the robot.
Pick and drop are trained as separate modules, so each gets its own dataset (--stages chooses the frames):
    python convert_to_lerobot.py --demos ~/demos --out ~/datasets/stretch_pick --stages grasp,teleop_grasp
    python convert_to_lerobot.py --demos ~/demos --out ~/datasets/stretch_drop --stages teleop_drop --source teleop
At the end it prints how many demos were kept and skipped (with the reason for each), and how many frames
were left out of kept demos because their stage was not listed.

What it builds, per frame (10 Hz, same as the recorder):
    observation.images.head   head camera image (240x320 RGB)
    observation.images.wrist  wrist camera image  (only with --wrist)
    observation.state         [base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt]
    action                    [base_x, lift, arm, wrist_yaw, gripper] of the NEXT frame (absolute targets)
    task                      e.g. "pick up the soup can"
base_x (m) = how far the base has driven forward since the FIRST KEPT FRAME of the demo. It is the odometry
displacement from that first frame (x, y in the odom frame), projected onto the heading the base had at that
frame; sideways drift and turning are not part of it. So base_x is 0.0 at the start of every demo and says
nothing about where the robot is on the floor: the policy only learns "drive this far from where I began".
That is why demos must start from different base positions (e.g. the alignment test), otherwise the
policy cannot learn to correct for the start position. The action's base_x is the next frame's base_x.
Head pan/tilt are observed but not commanded (keep the head still during demos); the converter warns if
the head moved more than HEAD_TOL during a demo.
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
HEAD_TOL = 0.03   # rad (~1.7 deg): head pan/tilt change larger than this during one demo triggers a warning


def load_episode(ep: Path, stages, wrist):
    """Return (rows, meta, other) for one episode folder.
    rows = frames in the chosen stages; other = {stage: n} for frames left out because of the stage filter."""
    steps = ep / 'steps.jsonl'
    if not steps.exists(): return [], {}, {}
    meta = json.loads((ep / 'meta.json').read_text()) if (ep / 'meta.json').exists() else {}
    rows, other = [], {}
    for line in steps.read_text().splitlines():
        try: r = json.loads(line)
        except json.JSONDecodeError: continue          # a half-written last line if the logger was killed
        if stages and r.get('stage') not in stages:
            other[r.get('stage')] = other.get(r.get('stage'), 0) + 1
            continue
        if wrist and not r.get('wrist'): continue
        rows.append(r)
    return rows, meta, other


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


def head_moved(rows):
    """{'head_pan': range, 'head_tilt': range} for each head joint that moved more than HEAD_TOL (rad)."""
    out = {}
    for k in ('head_pan', 'head_tilt'):
        v = [float(r['joints'].get(JOINTS[k], 0.0)) for r in rows]
        if v and max(v) - min(v) > HEAD_TOL: out[k] = round(max(v) - min(v), 3)
    return out


def state_vector(r, bx):
    j = r['joints']
    return [bx] + [float(j.get(JOINTS[k], 0.0)) for k in STATE_NAMES[1:]]


def states_actions(rows, base_mode='absolute'):
    """Observation states and actions for one demo.
    absolute: state [base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt]; action = next [base_x, lift, arm, yaw, gripper].
    relative: base_x is NOT in the state (the policy must judge distance from the camera); the action's first value is
              how far the base moves to the next frame (m per 0.1 s), the rest are the next joint targets."""
    bx = base_forward(rows)
    full = [state_vector(r, b) for r, b in zip(rows, bx)]
    nxt = lambda i: full[min(i + 1, len(full) - 1)]
    if base_mode == 'absolute':
        return full, [nxt(i)[:len(ACTION_NAMES)] for i in range(len(full))]
    return [f[1:] for f in full], [[nxt(i)[0] - full[i][0]] + nxt(i)[1:len(ACTION_NAMES)] for i in range(len(full))]


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
    ap.add_argument('--task', default=None,
                    help='task text for every demo; {object} is filled in. Default: the demo\'s own instruction from '
                         'meta.json (e.g. "drop the soup can in the yellow container"), else "pick up the {object}"')
    ap.add_argument('--include-failed', action='store_true', help='also keep demos whose result is not OK')
    ap.add_argument('--source', choices=['all', 'teleop', 'scripted'], default='all')
    ap.add_argument('--wrist', action='store_true', help='include the wrist camera (frames without it are skipped)')
    ap.add_argument('--videos', action='store_true', help='store images as video (smaller; needs ffmpeg)')
    ap.add_argument('--min-frames', type=int, default=20)
    ap.add_argument('--base-mode', choices=['absolute', 'relative'], default='absolute',
                    help='relative: leave base_x out of the state and make the base action a per-step move (see states_actions)')
    a = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    stages = set(s for s in a.stages.split(',') if s)
    eps = sorted(p for p in Path(os.path.expanduser(a.demos)).iterdir() if p.is_dir())
    img_type = 'video' if a.videos else 'image'
    state_names = STATE_NAMES if a.base_mode == 'absolute' else STATE_NAMES[1:]
    action_names = ACTION_NAMES if a.base_mode == 'absolute' else ['base_dx'] + ACTION_NAMES[1:]
    features = {
        'observation.images.head': {'dtype': img_type, 'shape': (H, W, 3), 'names': ['height', 'width', 'channels']},
        'observation.state': {'dtype': 'float32', 'shape': (len(state_names),), 'names': state_names},
        'action': {'dtype': 'float32', 'shape': (len(action_names),), 'names': action_names},
    }
    if a.wrist:
        features['observation.images.wrist'] = {'dtype': img_type, 'shape': (H, W, 3), 'names': ['height', 'width', 'channels']}
    ds = LeRobotDataset.create(repo_id=a.repo_id, fps=10, features=features, root=os.path.expanduser(a.out),
                               robot_type='stretch_re1', use_videos=a.videos)

    kept, skipped, left_out, head_warn = 0, [], {}, []
    for ep in eps:
        rows, meta, other = load_episode(ep, stages, a.wrist)
        src = meta.get('source', 'teleop' if '_teleop_' in ep.name else 'scripted')
        why = None                                           # (short reason for the summary, detail for the line)
        if not meta: why = ('no meta.json (run unfinished?)', 'no meta.json')
        elif meta.get('result') != 'OK' and not a.include_failed:
            why = ('result not OK', f"result {meta.get('result')}")
        elif a.source != 'all' and src != a.source: why = (f'source is not {a.source}', f'source {src}')
        elif len(rows) < a.min_frames:
            have = f'; the demo only has stages {sorted(k or "(none)" for k in other)}' if other else ''
            why = ('too few frames in the listed stages', f'{len(rows)} frames in stages {sorted(stages) or "all"}{have}')
        if why: skipped.append((ep.name, *why)); continue

        for st, n in other.items(): left_out[st] = left_out.get(st, 0) + n
        moved = head_moved(rows)
        if moved: head_warn.append(ep.name); print(f'  ! {ep.name}: head moved during the demo {moved} rad - keep the head still')
        states, actions = states_actions(rows, a.base_mode)
        task = (a.task or meta.get('instruction') or 'pick up the {object}').format(
            object=meta.get('object', 'object').replace('_', ' '))
        for i, r in enumerate(rows):
            frame = {'observation.images.head': load_image(ep / r['frame']),
                     'observation.state': np.asarray(states[i], np.float32),
                     'action': np.asarray(actions[i], np.float32),
                     'task': task}
            if a.wrist: frame['observation.images.wrist'] = load_image(ep / r['wrist'])
            ds.add_frame(frame)
        ds.save_episode()
        kept += 1
        print(f'  + {ep.name}: {len(rows)} frames ({len(rows) / 10:.1f} s), {src}, "{task}"')

    ds.finalize()
    for name, _, detail in skipped: print(f'  - skipped {name}: {detail}')
    print(f'\nKept {kept} of {len(eps)} demos, skipped {len(skipped)}.')
    reasons = {}
    for _, short, _ in skipped: reasons[short] = reasons.get(short, 0) + 1
    for short, n in reasons.items(): print(f'    {n} x {short}')
    if left_out:
        print(f'Frames left out of the kept demos because their stage is not in --stages: '
              + ', '.join(f'{st or "(none)"}: {n}' for st, n in sorted(left_out.items(), key=lambda kv: str(kv[0]))))
    if head_warn: print(f'Head moved in {len(head_warn)} kept demo(s): {", ".join(head_warn)}')
    print(f'{kept} episodes, {ds.num_frames} frames -> {a.out}')
    if kept == 0: sys.exit('nothing converted - check --stages / --source / results')


if __name__ == '__main__':
    main()
