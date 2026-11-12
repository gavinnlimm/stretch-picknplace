#!/usr/bin/env python3
"""
LLM task planner for the Stretch - runs ON THE ROBOT (or anywhere for a dry run).
A typed command becomes a plan of skills (skills.py) chosen by an LLM (llm.py: Qwen on the lab GPU by default; Claude, GPT-4o or a mock). The plan is checked before anything
moves, then run one skill at a time with the existing scripts; each skill reports success or failure. On a failure
the LLM is asked for a new plan from where the robot is now (shown to you before it runs).

    # anywhere, no robot, no API key: show the plan and the commands it would run
    python3 planner.py "put the soup can in the yellow container" --llm mock

    # robot tab (driver in position mode, camera + Nav2 running, both policy servers + tunnels up, hand on the run-stop)
    # default LLM: Qwen on the GPU PC (Ollama, port 11434, tunnelled like the policy servers); no key needed
    /usr/bin/python3 ~/planner/planner.py "put the soup can in the yellow container" --go
    # or --llm claude / --llm openai with ANTHROPIC_API_KEY / OPENAI_API_KEY exported in the shell (never in the repo)

Dry run by default: --go is needed to move the robot, and it asks once before running the plan. Ctrl+C stops it.
Every run is appended to ~/planner_runs.jsonl (command, plan, each step's result and time) for success-rate counts.
Policy servers: pick on port 8000 (act_pick_rel2), drop on port 8001 (act_drop_rel2_new); change with --pick-server / --drop-server.
STATUS: written Nov 11, tested offline (test_planner.py); not yet run on the robot.
"""
import argparse, json, os, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skills, llm   # noqa: E402

LOG = os.path.expanduser('~/planner_runs.jsonl')


def show_plan(p, cfg):
    print(f'\nPlan: {p.get("reply", "")}')
    if p.get('goal'): print(f'  goal: {p["goal"]}')
    for i, st in enumerate(p['steps'], 1):
        args = ', '.join(f'{k}={v}' for k, v in (st.get('args') or {}).items())
        print(f'  {i}. {st["skill"]}({args})')
        for label, cmd, typed, _ in skills.commands(st, cfg):
            print(f'       - {label}: {cmd}' + (f'   [types: {typed!r}]' if typed else ''))


def run_shell(cmd, typed=None):
    """Run one command, print its output live, return (return code, all output)."""
    pr = subprocess.Popen(['bash', '-c', cmd], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    if typed: pr.stdin.write(typed)
    pr.stdin.close()
    out = []
    for line in pr.stdout:
        print('      | ' + line, end=''); out.append(line)
    return pr.wait(), ''.join(out)


def run_step(st, cfg, runner=run_shell, ask=input):
    """True if every command of the skill reports success."""
    if st['skill'] == 'ask_for_help':
        print(f'\n  HELP NEEDED: {st["args"]["message"]}')
        return ask('  Press Enter when done (q = it could not be done): ').strip().lower() != 'q'
    for label, cmd, typed, ok in skills.commands(st, cfg):
        print(f'    - {label}')
        rc, out = runner(cmd, typed)
        if not ok(rc, out):
            print(f'    FAILED: {label} (exit code {rc})')
            return False
    return True


def execute(command, model, world, cfg, go, max_replans=1, runner=run_shell, ask=input):
    """Plan, check, (run), replan on failure. Returns the run record that is also written to the log."""
    rec = {'time': time.strftime('%F %T'), 'command': command, 'start': dict(world), 'plans': [], 'steps': [], 'result': None}
    history, replans = [], 0
    while True:
        p, errors = llm.plan(model, command, world, history or None)
        rec['plans'].append(p)
        if errors:
            print('\nThe LLM did not produce a plan that can run:\n  ' + '\n  '.join(errors))
            rec['result'] = 'no valid plan'; return rec
        if not p['steps']:
            print(f'\nNothing to do: {p.get("reply", "")}')
            rec['result'] = 'nothing to do' if not history else 'stopped'; return rec
        show_plan(p, cfg)
        if not go:
            print('\nDry run: nothing moved. Add --go to run it on the robot.')
            rec['result'] = 'dry run'; return rec
        if ask('\nRun this plan on the robot? [y/N] ').strip().lower() != 'y':
            rec['result'] = 'declined'; return rec
        failed = None
        for i, st in enumerate(p['steps'], 1):
            print(f'\n[{i}/{len(p["steps"])}] {st["skill"]} {st.get("args") or ""}')
            t0 = time.time(); ok = run_step(st, cfg, runner, ask)
            rec['steps'].append({'skill': st['skill'], 'args': st.get('args') or {}, 'ok': ok, 'seconds': round(time.time() - t0, 1)})
            desc = f'{st["skill"]}({", ".join(f"{k}={v}" for k, v in (st.get("args") or {}).items())})'
            if ok:
                world = skills.apply(st, world); history.append(f'{desc}: done')
            else:
                world = skills.after_failure(st, world); history.append(f'{desc}: FAILED'); failed = st
                break
        if failed is None:
            print('\nDone: every step reported success.')
            rec['result'] = 'success'; rec['end'] = world; return rec
        if replans >= max_replans:
            print('\nStopped: a step failed and no replans are left.')
            rec['result'] = 'failed'; rec['end'] = world; return rec
        replans += 1
        print(f'\nA step failed; asking the LLM for a new plan from here (robot at {world["at"]}, holding {world["holding"] or "nothing"}).')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', help='what the robot should do, in plain words')
    ap.add_argument('--llm', choices=['local', 'claude', 'openai', 'mock'], default='local',
                    help='which LLM plans: local = Qwen on the GPU PC via Ollama (default); mock = keyword rules, no network')
    ap.add_argument('--model', default=None, help='model name (default: qwen3:8b for local, claude-sonnet-5-5, gpt-4o)')
    ap.add_argument('--think', action='store_true', help='local only: let Qwen3 "think" before answering (slower, ~15 s; compare plan quality)')
    ap.add_argument('--llm-url', default=None, help=f'local only: the Ollama/vLLM chat URL (default {llm.LOCAL_URL})')
    ap.add_argument('--go', action='store_true', help='actually run the plan on the robot (default: dry run)')
    ap.add_argument('--at', choices=['home', 'table1', 'table2'], default='home', help='where the robot is now')
    ap.add_argument('--holding', default=None, help='object already in the gripper, if any (e.g. soup_can)')
    ap.add_argument('--max-replans', type=int, default=1)
    ap.add_argument('--pick-server', default=skills.DEFAULTS['pick_server'])
    ap.add_argument('--drop-server', default=skills.DEFAULTS['drop_server'])
    ap.add_argument('--pick-flags', default='', help='extra policy_client flags for the pick, e.g. "--keep-open 0.10 --hold-grip -0.15" (scripted helps: report them)')
    ap.add_argument('--drop-flags', default='', help='extra policy_client flags for the drop')
    a = ap.parse_args()
    cfg = {'pick_server': a.pick_server, 'drop_server': a.drop_server, 'pick_flags': a.pick_flags, 'drop_flags': a.drop_flags}
    world = {'at': a.at, 'holding': a.holding}
    rec = execute(a.command, llm.make(a.llm, a.model, a.llm_url, a.think), world, cfg, a.go, a.max_replans)
    rec.update({'llm': a.llm, 'model': a.model or llm.DEFAULT_MODEL[a.llm], 'think': a.think, 'cfg': cfg, 'go': a.go})
    if a.go:
        with open(LOG, 'a') as f: f.write(json.dumps(rec) + '\n')
        print(f'logged to {LOG}')
    sys.exit(0 if rec['result'] in ('success', 'dry run', 'nothing to do') else 1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit('\nstopped (Ctrl+C)')
