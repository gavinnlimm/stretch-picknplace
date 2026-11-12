#!/usr/bin/env python3
"""Offline tests for the planner (no robot, no network):  python3 -m unittest test_planner -v"""
import io, json, os, sys, unittest
from contextlib import redirect_stdout
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skills, llm, planner   # noqa: E402

HOME = {'at': 'home', 'holding': None}
NAV1 = {'skill': 'navigate', 'args': {'target': 'table1'}}
NAV2 = {'skill': 'navigate', 'args': {'target': 'table2'}}
PICK = {'skill': 'pick', 'args': {'object': 'soup_can'}}
DROP = {'skill': 'drop', 'args': {'container': 'yellow_container'}}


class FixedLLM:
    """Returns the given answers in order (to test the fix-up and replan loops)."""
    def __init__(self, *answers): self.answers = list(answers); self.seen = []
    def chat(self, messages):
        self.seen.append(messages); return self.answers.pop(0)


def quiet(f, *a, **k):
    with redirect_stdout(io.StringIO()): return f(*a, **k)


class WithScripted(unittest.TestCase):
    """Tests that use the scripted baseline turn it on, then back to the default (off)."""
    def setUp(self): skills.use_scripted(True)
    def tearDown(self): skills.use_scripted(False)


class LearnedOnly(unittest.TestCase):
    def test_scripted_skills_are_off_by_default(self):
        e = skills.check_plan([NAV1, {'skill': 'grasp_scripted', 'args': {'object': 'soup_can'}}], HOME)
        self.assertIn('scripted skills are turned off', e[0])
        self.assertNotIn('grasp_scripted', llm.system_prompt()); self.assertNotIn('place_scripted', llm.system_prompt())

    def test_mock_refuses_other_objects_and_destinations(self):
        for cmd in ('put the 7UP bottle in the container', 'put the soup can on table 2'):
            p, e = llm.plan(llm.MockLLM(), cmd, HOME)
            self.assertEqual(p['steps'], []); self.assertIn('scripted skills are off', p['reply'])

    def test_learned_plan_still_works(self):
        p, e = llm.plan(llm.MockLLM(), 'put the soup can in the yellow container', HOME)
        self.assertEqual(e, []); self.assertEqual(p['steps'], [NAV1, PICK, NAV2, DROP])


class CheckPlan(WithScripted):
    def test_standard_plan_is_valid(self):
        self.assertEqual(skills.check_plan([NAV1, PICK, NAV2, DROP], HOME), [])

    def test_unknown_skill(self):
        self.assertIn('unknown skill', skills.check_plan([{'skill': 'fly', 'args': {}}], HOME)[0])

    def test_bad_argument_value(self):
        e = skills.check_plan([NAV1, {'skill': 'pick', 'args': {'object': 'banana'}}], HOME)
        self.assertIn('must be one of', e[0])

    def test_missing_argument(self):
        self.assertIn('takes arguments', skills.check_plan([{'skill': 'navigate', 'args': {}}], HOME)[0])

    def test_pick_needs_table1(self):
        self.assertIn('must be at table1', skills.check_plan([PICK], HOME)[0])

    def test_drop_needs_an_object(self):
        self.assertIn('holding an object', skills.check_plan([NAV2, DROP], HOME)[0])

    def test_cannot_pick_twice(self):
        self.assertIn('gripper must be empty', skills.check_plan([NAV1, PICK, PICK], HOME)[0])

    def test_learned_pick_only_soup_can_scripted_others(self):
        self.assertEqual(skills.check_plan([NAV1, {'skill': 'grasp_scripted', 'args': {'object': 'soda_bottle'}},
                                            NAV2, {'skill': 'place_scripted', 'args': {}}], HOME), [])

    def test_too_many_steps(self):
        self.assertIn('too many steps', skills.check_plan([NAV1] * 13, HOME)[0])


class GoalCheck(WithScripted):
    G = {'object': 'soup_can', 'destination': 'yellow_container'}

    def test_full_plan_reaches_goal(self):
        self.assertEqual(skills.check_goal(self.G, [NAV1, PICK, NAV2, DROP], HOME), [])

    def test_plan_that_stops_early_is_rejected(self):          # what Qwen did on Nov 11
        self.assertIn('never picks up', skills.check_goal(self.G, [NAV1], HOME)[0])

    def test_plan_without_delivery_is_rejected(self):
        self.assertIn('never gets the soup_can to yellow_container', skills.check_goal(self.G, [NAV1, PICK, NAV2], HOME)[0])

    def test_missing_goal_is_rejected(self):
        self.assertIn('add a "goal"', skills.check_goal(None, [NAV1], HOME)[0])

    def test_refusal_needs_no_goal(self):
        self.assertEqual(skills.check_goal(None, [], HOME), [])

    def test_go_home(self):
        self.assertEqual(skills.check_goal({'object': None, 'destination': 'home'}, [{'skill': 'navigate', 'args': {'target': 'home'}}], {'at': 'table2', 'holding': None}), [])
        self.assertTrue(skills.check_goal({'object': None, 'destination': 'home'}, [NAV1], HOME))

    def test_ask_for_help_is_exempt(self):
        self.assertEqual(skills.check_goal(self.G, [{'skill': 'ask_for_help', 'args': {'message': 'help'}}], HOME), [])

    def test_already_holding_counts(self):
        self.assertEqual(skills.check_goal(self.G, [NAV2, DROP], {'at': 'table1', 'holding': 'soup_can'}), [])

    def test_incomplete_plan_is_sent_back_and_fixed(self):
        short = json.dumps({'reply': '', 'goal': self.G, 'steps': [NAV1]})
        full = json.dumps({'reply': 'ok', 'goal': self.G, 'steps': [NAV1, PICK, NAV2, DROP]})
        f = FixedLLM(short, full)
        p, e = llm.plan(f, 'grab the soup and toss it in the bin', HOME)
        self.assertEqual(e, []); self.assertIn('plan the whole task', f.seen[1][-1]['content'])


class Commands(WithScripted):
    def test_pick_uses_pick_server_and_stop_when_done(self):
        cmds = skills.commands(PICK)
        self.assertIn('pick_reset.py --grip 0.13', cmds[0][1]); self.assertEqual(cmds[0][2], '\n\n\n')
        self.assertIn('--server http://localhost:8000 --module pick', cmds[1][1]); self.assertIn('--stop-when-done', cmds[1][1])

    def test_drop_keeps_the_can_and_uses_drop_server(self):
        cmds = skills.commands(DROP)
        self.assertIn('drop_reset.py --back 0 --regrip --hold -0.195', cmds[0][1]); self.assertEqual(cmds[0][2], '\n\n\n')
        self.assertIn('--server http://localhost:8001 --module drop', cmds[1][1])

    def test_navigation_switches_mode_first(self):
        self.assertIn('switch_to_navigation_mode', skills.commands(NAV2)[0][1])

    def test_scripted_helps_are_passed_through(self):
        self.assertIn('--hold-grip -0.15', skills.commands(PICK, {'pick_flags': '--hold-grip -0.15'})[1][1])

    def test_success_checks(self):
        self.assertTrue(skills.ok_policy(0, '...stop-when-done: the task looks finished'))
        self.assertFalse(skills.ok_policy(0, 'done after 120 steps'))          # timed out
        self.assertFalse(skills.ok_nav(0, 'Goal aborted'))
        self.assertTrue(skills.ok_nav(0, 'Goal finished with status: SUCCEEDED'))


class LLMLayer(WithScripted):
    def test_mock_soup_can_into_container(self):
        p, e = llm.plan(llm.MockLLM(), 'put the soup can in the yellow container', HOME)
        self.assertEqual(e, []); self.assertEqual(p['steps'], [NAV1, PICK, NAV2, DROP])

    def test_mock_bottle_on_table2_uses_scripted_skills(self):
        p, e = llm.plan(llm.MockLLM(), 'put the 7UP bottle on table 2', HOME)
        self.assertEqual(e, []); self.assertEqual([s['skill'] for s in p['steps']], ['navigate', 'grasp_scripted', 'navigate', 'place_scripted'])

    def test_fenced_json_is_accepted(self):
        self.assertEqual(llm.parse_json('```json\n{"reply": "ok", "steps": []}\n```')['reply'], 'ok')

    def test_bad_plan_is_sent_back_once_and_fixed(self):
        bad = json.dumps({'reply': '', 'steps': [PICK]})                       # pick from home: invalid
        good = json.dumps({'reply': 'ok', 'goal': {'object': 'soup_can', 'destination': None}, 'steps': [NAV1, PICK]})
        f = FixedLLM(bad, good)
        p, e = llm.plan(f, 'pick up the soup can', HOME)
        self.assertEqual(e, []); self.assertIn('must be at table1', f.seen[1][-1]['content'])

    def test_still_bad_after_fix_reports_errors(self):
        bad = json.dumps({'reply': '', 'steps': [PICK]})
        p, e = llm.plan(FixedLLM(bad, bad), 'pick up the soup can', HOME)
        self.assertTrue(e)

    def test_sentence_around_json_is_accepted(self):
        self.assertEqual(llm.parse_json('Here is the plan: {"reply": "ok", "steps": []} Hope it helps.')['reply'], 'ok')

    def test_claude_request_shape(self):
        captured = {}
        class R:
            def __enter__(s): return s
            def __exit__(s, *a): pass
            def read(s): return json.dumps({'content': [{'type': 'text', 'text': '{"reply": "x", "steps": []}'}]}).encode()
        def fake_urlopen(req, timeout):
            captured['body'] = json.loads(req.data); captured['headers'] = {k.lower(): v for k, v in req.headers.items()}; return R()
        with mock.patch.dict(os.environ, {'ANTHROPIC_API_KEY': 'test-key'}), mock.patch('urllib.request.urlopen', fake_urlopen):
            out = llm.make('claude').chat([{'role': 'system', 'content': 'rules'}, {'role': 'user', 'content': 'hi'}])
        b = captured['body']
        self.assertEqual(b['model'], 'claude-sonnet-5-5'); self.assertEqual(b['system'], 'rules')
        self.assertEqual(b['messages'], [{'role': 'user', 'content': 'hi'}])                # system moved out of messages
        self.assertEqual(captured['headers']['x-api-key'], 'test-key'); self.assertIn('anthropic-version', captured['headers'])
        self.assertIn('steps', out)

    def test_local_qwen_needs_no_key_and_hits_ollama(self):
        captured = {}
        class R:
            def __enter__(s): return s
            def __exit__(s, *a): pass
            def read(s): return json.dumps({'choices': [{'message': {'content': '<think>table1 first {maybe}</think>{"reply": "x", "steps": []}'}}]}).encode()
        def fake_urlopen(req, timeout):
            captured['url'] = req.full_url; captured['body'] = json.loads(req.data)
            captured['headers'] = {k.lower() for k in req.headers}; return R()
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch('urllib.request.urlopen', fake_urlopen):
            out = llm.make('local').chat([{'role': 'user', 'content': 'hi'}])
        self.assertEqual(captured['url'], 'http://localhost:11434/v1/chat/completions')
        self.assertEqual(captured['body']['model'], 'qwen3:8b'); self.assertNotIn('authorization', captured['headers'])
        self.assertEqual(captured['body']['reasoning_effort'], 'none')          # Qwen3 thinking switched off
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertNotIn('reasoning_effort', llm.make('local', think=True).extra)   # --think leaves it on
        self.assertEqual(llm.parse_json(out)['reply'], 'x')                    # the <think> part is ignored

    def test_missing_key_stops_with_a_message(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(SystemExit): llm.make('claude')

    def test_prompt_lists_every_skill(self):
        for name in skills.SKILLS: self.assertIn(name, llm.system_prompt())

    def test_openai_request_shape(self):
        captured = {}
        class R:
            def __enter__(s): return s
            def __exit__(s, *a): pass
            def read(s): return json.dumps({'choices': [{'message': {'content': '{"reply": "x", "steps": []}'}}]}).encode()
        def fake_urlopen(req, timeout):
            captured['body'] = json.loads(req.data); captured['auth'] = req.headers['Authorization']; return R()
        with mock.patch.dict(os.environ, {'OPENAI_API_KEY': 'test-key'}), mock.patch('urllib.request.urlopen', fake_urlopen):
            out = llm.OpenAILLM().chat([{'role': 'user', 'content': 'hi'}])
        self.assertEqual(captured['body']['model'], 'gpt-4o'); self.assertEqual(captured['body']['response_format'], {'type': 'json_object'})
        self.assertEqual(captured['auth'], 'Bearer test-key'); self.assertIn('steps', out)


class Executor(WithScripted):
    def fake_runner(self, fail_on=None):
        ran = []
        def run(cmd, typed):
            ran.append(cmd)
            if fail_on and fail_on in cmd: return 0, 'done after 120 steps'           # timed out: no stop-when-done
            if 'policy_client' in cmd: return 0, 'stop-when-done: the task looks finished'
            return 0, 'Goal finished with status: SUCCEEDED'
        return run, ran

    def test_dry_run_moves_nothing(self):
        run, ran = self.fake_runner()
        rec = quiet(planner.execute, 'put the soup can in the yellow container', llm.MockLLM(), dict(HOME), {}, go=False, runner=run)
        self.assertEqual(rec['result'], 'dry run'); self.assertEqual(ran, [])

    def test_declined_plan_moves_nothing(self):
        run, ran = self.fake_runner()
        rec = quiet(planner.execute, 'put the soup can in the yellow container', llm.MockLLM(), dict(HOME), {}, go=True, runner=run, ask=lambda q: 'n')
        self.assertEqual(rec['result'], 'declined'); self.assertEqual(ran, [])

    def test_full_run_in_order(self):
        run, ran = self.fake_runner()
        rec = quiet(planner.execute, 'put the soup can in the yellow container', llm.MockLLM(), dict(HOME), {}, go=True, runner=run, ask=lambda q: 'y')
        self.assertEqual(rec['result'], 'success'); self.assertEqual(len(ran), 6)   # nav, reset+pick, nav, reset+drop
        self.assertIn('t1.sh', ran[0]); self.assertIn('--module pick', ran[2]); self.assertIn('t2.sh', ran[3]); self.assertIn('--module drop', ran[5])
        self.assertEqual(rec['end'], {'at': 'table2', 'holding': None})

    def test_failure_stops_and_replans_with_history(self):
        run, ran = self.fake_runner(fail_on='--module pick')
        m = llm.MockLLM(); seen = []
        orig = m.chat
        m.chat = lambda msgs: (seen.append(msgs[-1]['content']), orig(msgs))[1]
        answers = iter(['y', 'y', ''])                                             # run plan, run recovery plan, help done
        rec = quiet(planner.execute, 'put the soup can in the yellow container', m, dict(HOME), {}, go=True, runner=run, ask=lambda q: next(answers))
        self.assertFalse(any('t2.sh' in c for c in ran))                           # never drove on after the failed pick
        self.assertIn('pick(object=soup_can): FAILED', seen[-1])
        self.assertEqual(rec['steps'][-1]['skill'], 'ask_for_help'); self.assertEqual(rec['result'], 'success')

    def test_no_replans_left(self):
        run, ran = self.fake_runner(fail_on='--module pick')
        rec = quiet(planner.execute, 'put the soup can in the yellow container', llm.MockLLM(), dict(HOME), {}, go=True,
                    max_replans=0, runner=run, ask=lambda q: 'y')
        self.assertEqual(rec['result'], 'failed'); self.assertEqual(rec['end'], {'at': 'table1', 'holding': None})


if __name__ == '__main__':
    unittest.main()
