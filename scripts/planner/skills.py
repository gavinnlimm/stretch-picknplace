#!/usr/bin/env python3
"""
Skills the LLM planner may use, what each one needs and does, and the robot commands that run it.
Pure Python (no ROS, no network), so it can be imported and tested anywhere.

World state the planner tracks:  {'at': 'home' | 'table1' | 'table2' | 'unknown', 'holding': None | '<object>' | 'unknown'}

Each skill has:
  args     the arguments it takes and their allowed values (None = free text)
  needs    a check on the world state before it runs (returns an error string or None)
  effect   the world state after it succeeds
  steps    the shell commands that run it on the robot, each with what to type into it and how to tell it worked
"""
import re

TABLES = ('table1', 'table2', 'home')
LEARNED_OBJECTS = ('soup_can',)                                     # what the ACT pick / drop were trained on
SCRIPTED_OBJECTS = ('soup_can', 'soda_bottle', 'carton', 'paper_cup', 'coke_can')   # what grasp_auto.py handles
CONTAINERS = ('yellow_container',)

# ---------- success checks on a finished command (return code + everything it printed) ----------
NAV_FAIL = re.compile(r'rejected|aborted|canceled|no /amcl_pose', re.I)   # same test as full_auto_obj.sh


def ok_rc(rc, out):
    return rc == 0


def ok_nav(rc, out):
    return rc == 0 and not NAV_FAIL.search(out)


def ok_policy(rc, out):
    """policy_client.py exits 0 on a timeout too, so only its --stop-when-done message counts as finished."""
    return rc == 0 and 'stop-when-done:' in out


# ---------- world-state checks ----------
def _need(at=None, holding=None, empty=False):
    def check(world, args):
        if at and world['at'] != at:
            return f"the robot must be at {at} (it is at {world['at']})"
        if empty and world['holding'] is not None:
            return f"the gripper must be empty (it holds {world['holding']})"
        if holding and world['holding'] is None:
            return 'the robot must be holding an object (the gripper is empty)'
        return None
    return check


def _no_need(world, args):
    return None


SKILLS = {
    'navigate': {
        'about': 'Drive with Nav2 to a spot on the map, then correct the pose. table1 = where objects are picked, '
                 'table2 = where the yellow container and the place tag (tag 7) are, home = the start spot.',
        'args': {'target': TABLES},
        'needs': _no_need,
        'effect': lambda w, a: {**w, 'at': a['target']},
    },
    'pick': {
        'about': 'LEARNED pick (ACT policy) at table1: reaches the object, grasps it, lifts and pulls the arm in. '
                 'Trained on the soup can only.',
        'args': {'object': LEARNED_OBJECTS},
        'needs': _need(at='table1', empty=True),
        'effect': lambda w, a: {**w, 'holding': a['object']},
    },
    'drop': {
        'about': 'LEARNED drop (ACT policy) at table2: drives along the table to the yellow container and drops the held '
                 'object into it, then pulls the arm in.',
        'args': {'container': CONTAINERS},
        'needs': _need(at='table2', holding=True),
        'effect': lambda w, a: {**w, 'holding': None},
    },
    'grasp_scripted': {
        'about': 'SCRIPTED grasp at table1 (depth camera + hand-written grasp, the baseline). Works for 5 objects; use it '
                 'for anything the learned pick was not trained on.',
        'args': {'object': SCRIPTED_OBJECTS},
        'needs': _need(at='table1', empty=True),
        'effect': lambda w, a: {**w, 'holding': a['object']},
    },
    'place_scripted': {
        'about': 'SCRIPTED place at table2: lines up on tag 7 and sets the held object down ON the table (not in the container).',
        'args': {},
        'needs': _need(at='table2', holding=True),
        'effect': lambda w, a: {**w, 'holding': None},
    },
    'ask_for_help': {
        'about': 'Stop and ask the person to do something (fix the scene, hand over an object, confirm). '
                 'Use it when no skill fits or a step keeps failing.',
        'args': {'message': None},
        'needs': _no_need,
        'effect': lambda w, a: dict(w),
    },
}


def describe_skills():
    """Text for the LLM prompt: one block per skill with its arguments, needs and effect."""
    lines = []
    needs_txt = {'navigate': 'none', 'pick': 'at table1, gripper empty', 'drop': 'at table2, holding an object',
                 'grasp_scripted': 'at table1, gripper empty', 'place_scripted': 'at table2, holding an object',
                 'ask_for_help': 'none'}
    for name, s in SKILLS.items():
        args = ', '.join(f'{k}: one of {list(v)}' if v else f'{k}: free text' for k, v in s['args'].items()) or 'none'
        lines.append(f'- {name}({args})\n    {s["about"]}\n    needs: {needs_txt[name]}')
    return '\n'.join(lines)


# ---------- plan checking ----------
def check_step(step, world):
    """Error string if this step is malformed or cannot run from this world state, else None."""
    if not isinstance(step, dict) or not isinstance(step.get('skill'), str):
        return 'each step must be an object with a "skill" name'
    name = step['skill']
    if name not in SKILLS:
        return f'unknown skill "{name}"'
    args = step.get('args') or {}
    if not isinstance(args, dict):
        return f'{name}: "args" must be an object'
    spec = SKILLS[name]['args']
    if set(args) != set(spec):
        return f'{name} takes arguments {sorted(spec)} but got {sorted(args)}'
    for k, allowed in spec.items():
        v = args[k]
        if allowed is None:
            if not isinstance(v, str) or not v.strip():
                return f'{name}: {k} must be non-empty text'
        elif v not in allowed:
            return f'{name}: {k} must be one of {list(allowed)}, got {v!r}'
    return SKILLS[name]['needs'](world, args)


def check_plan(steps, world, max_steps=12):
    """List of errors (empty = the plan can run). Walks the plan through the world state step by step."""
    if not isinstance(steps, list):
        return ['"steps" must be a list']
    if len(steps) > max_steps:
        return [f'too many steps ({len(steps)} > {max_steps})']
    errors, w = [], dict(world)
    for i, st in enumerate(steps, 1):
        err = check_step(st, w)
        if err:
            errors.append(f'step {i}: {err}')
            break                                   # later steps depend on this one
        w = SKILLS[st['skill']]['effect'](w, st.get('args') or {})
    return errors


def apply(step, world):
    return SKILLS[step['skill']]['effect'](world, step.get('args') or {})


def after_failure(step, world):
    """Best guess of the world after a failed step (a camera check will replace this guess)."""
    name, w = step['skill'], dict(world)
    if name == 'navigate':
        w['at'] = 'unknown'
    elif name in ('pick', 'grasp_scripted'):
        w['holding'] = None          # assume the grasp missed; the arm is back in
    elif name in ('drop', 'place_scripted'):
        w['holding'] = 'unknown'     # the object may still be in the gripper, or may have fallen
    return w


# ---------- robot commands ----------
DEFAULTS = {
    'pick_server': 'http://localhost:8000',      # policy_server.py with act_pick_rel2
    'drop_server': 'http://localhost:8001',      # a second policy_server.py with act_drop_rel2_new
    'pick_seconds': 50, 'drop_seconds': 90,
    'pick_flags': '',                            # e.g. '--keep-open 0.10 --hold-grip -0.15' (scripted helps: report them)
    'drop_flags': '',
}
ROS = 'source ~/ros_env.sh && '
NAV_MODE = 'ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger > /dev/null && '
PY = '/usr/bin/python3 -u'


def commands(step, cfg=None):
    """[(label, shell command, text typed into it, success check)] for one step, run in order on the robot."""
    c = {**DEFAULTS, **(cfg or {})}
    name, a = step['skill'], step.get('args') or {}
    if name == 'navigate':
        script = {'table1': '~/t1.sh', 'table2': '~/t2.sh', 'home': '~/go_home.sh'}[a['target']]
        return [(f'Nav2 to {a["target"]}', ROS + NAV_MODE + f'bash {script}', None, ok_nav)]
    if name == 'pick':
        return [('pick reset (arm in, gripper open, head to the pick pose)', ROS + f'{PY} ~/pick_reset.py --grip 0.13', '\n\n\n', ok_rc),
                ('learned pick (ACT)', ROS + f'{PY} ~/policy_client.py --server {c["pick_server"]} --module pick --go '
                 f'--seconds {c["pick_seconds"]} --base-mode relative --stop-when-done {c["pick_flags"]}'.rstrip(), None, ok_policy)]
    if name == 'drop':
        # drop_reset answers: s = keep the can (skip opening the gripper), Enter = lift/arm pose, Enter = head pose
        return [('drop reset (keep the can, carry pose, head to the drop pose)', ROS + f'{PY} ~/drop_reset.py --back 0', 's\n\n\n', ok_rc),
                ('learned drop (ACT)', ROS + f'{PY} ~/policy_client.py --server {c["drop_server"]} --module drop --go '
                 f'--seconds {c["drop_seconds"]} --base-mode relative --stop-when-done {c["drop_flags"]}'.rstrip(), None, ok_policy)]
    if name == 'grasp_scripted':
        return [('scripted grasp', ROS + f'{PY} ~/grasp_auto.py t1 go', None, ok_rc)]
    if name == 'place_scripted':
        return [('align on tag 7', ROS + f'{PY} ~/aruco_align.py align t2', None, ok_rc),
                ('scripted place', ROS + f'{PY} ~/pick_place.py place', None, ok_rc)]
    if name == 'ask_for_help':
        return []                                    # handled by the executor: it prints the message and waits
    raise ValueError(f'unknown skill {name}')
