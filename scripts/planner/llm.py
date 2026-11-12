#!/usr/bin/env python3
"""
The language part of the planner: turns a typed command into a plan (JSON) using only the skills in skills.py.

Four back ends:
  local   an open-source model (Qwen) served on the lab GPU PC by Ollama, through the same OpenAI-style API. No key, no cost.
          Default URL http://localhost:11434/v1/chat/completions (Ollama's port, tunnelled from the GPU PC like the policy servers).
  claude  calls the Claude Messages API (default model claude-sonnet-5-5). Needs ANTHROPIC_API_KEY in the environment.
  openai  calls the OpenAI chat API (default model gpt-4o). Needs OPENAI_API_KEY in the environment.
  mock    simple keyword rules, no network. For tests and for trying the executor without an API key.
API keys go in the shell only (export ...), never in a file in the repo.

Both return {"reply": "<one sentence to the person>", "steps": [{"skill": ..., "args": {...}}, ...]}
(steps may be empty when the command can't be done; then "reply" says why).
Standard library only, so it runs on the robot's /usr/bin/python3 without installing anything.
"""
import json, os, re, urllib.request

import skills

SYSTEM = """You plan tasks for a Hello Robot Stretch RE1 in a lab. You do not move the robot yourself: you choose a
sequence of skills, and a program runs them one by one and checks each one.

The lab: the robot starts at "home". Objects to pick up are on table1. Table2 has a yellow container (things are
DROPPED into it) and a tag (tag 7) where things are PLACED on the table. The robot can hold one object at a time.

Skills (use only these, with exactly these argument names and values):
{skills}

Rules:
{skill_rule}
- Every step's "needs" must hold, given the steps before it. Navigate before you pick or drop.
- Objects to pick up are ALWAYS on table1, even when the robot is somewhere else. To move an object, go to table1
  and pick it up first. Do not ask the person to bring objects.
- Plan the WHOLE task: the last step must finish what the person asked (object delivered, or robot where asked).
- If the command is unclear or impossible with these skills, return no steps and say why in "reply".
- Keep plans short. Do not add steps the person did not ask for (for example, do not go home unless asked).
- Also state the goal: "object" = the object to move (or null), "destination" = "yellow_container" (dropped in it),
  "table2" (set down on table2), "home", or null. A program checks that your steps reach this goal.

Examples:
{examples}Command: water the plants. Robot now: at home, holding nothing.
{{"reply": "I can't water plants with my skills.", "goal": null, "steps": []}}

Answer with JSON only, in this form:
{{"reply": "<one short sentence to the person>", "goal": {{"object": <name or null>, "destination": <name or null>}},
 "steps": [{{"skill": "<name>", "args": {{...}}}}]}}"""


EXAMPLES_SCRIPTED = """Command: put the coke can in the container. Robot now: at home, holding nothing.
{{"reply": "Taking the coke can to the container.", "goal": {{"object": "coke_can", "destination": "yellow_container"}},
 "steps": [{{"skill": "navigate", "args": {{"target": "table1"}}}}, {{"skill": "grasp_scripted", "args": {{"object": "coke_can"}}}},
           {{"skill": "navigate", "args": {{"target": "table2"}}}}, {{"skill": "drop", "args": {{"container": "yellow_container"}}}}]}}
Command: set the paper cup down on table 2. Robot now: at table2, holding nothing.
{{"reply": "Fetching the paper cup from table 1.", "goal": {{"object": "paper_cup", "destination": "table2"}},
 "steps": [{{"skill": "navigate", "args": {{"target": "table1"}}}}, {{"skill": "grasp_scripted", "args": {{"object": "paper_cup"}}}},
           {{"skill": "navigate", "args": {{"target": "table2"}}}}, {{"skill": "place_scripted", "args": {{}}}}]}}
"""

EXAMPLES_LEARNED = """Command: please take the soup can over to the yellow container. Robot now: at table2, holding nothing.
{{"reply": "Fetching the soup can from table 1.", "goal": {{"object": "soup_can", "destination": "yellow_container"}},
 "steps": [{{"skill": "navigate", "args": {{"target": "table1"}}}}, {{"skill": "pick", "args": {{"object": "soup_can"}}}},
           {{"skill": "navigate", "args": {{"target": "table2"}}}}, {{"skill": "drop", "args": {{"container": "yellow_container"}}}}]}}
Command: put the coke can in the container. Robot now: at home, holding nothing.
{{"reply": "I can only pick up the soup can.", "goal": null, "steps": []}}
"""

RULE_SCRIPTED = """- Prefer the LEARNED skills (pick, drop) when the object and target allow it; use grasp_scripted for objects the
  learned pick was not trained on, and place_scripted when the person wants the object set down on table2 rather
  than in the container."""
RULE_LEARNED = """- Only the learned skills are available: the robot can only pick up the soup can, and only drop it into the yellow
  container. For other objects or destinations, return no steps and say why in "reply"."""


def system_prompt():
    scripted = 'grasp_scripted' in skills.ENABLED
    return SYSTEM.format(skills=skills.describe_skills(), skill_rule=RULE_SCRIPTED if scripted else RULE_LEARNED,
                         examples=EXAMPLES_SCRIPTED if scripted else EXAMPLES_LEARNED)


def user_message(command, world, history=None):
    """The command plus the current world state, and (when replanning) what happened so far."""
    msg = f'Command: {command}\nRobot now: at {world["at"]}, holding {world["holding"] or "nothing"}.'
    if history:
        msg += '\nWhat happened so far:\n' + '\n'.join(f'- {h}' for h in history)
        msg += '\nPlan only what is left to do from the robot\'s current state.'
    return msg


def parse_json(text):
    """The model's answer -> dict. Accepts a ```json fenced block, or a sentence before/after the JSON object."""
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.S)        # Qwen3 may "think" before answering
    m = re.search(r'```(?:json)?\s*(\{.*\})\s*```', text, re.S)
    if m:
        text = m.group(1)
    elif not text.strip().startswith('{') and '{' in text:
        text = text[text.index('{'):text.rindex('}') + 1]
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError('the answer is not a JSON object')
    data.setdefault('steps', []); data.setdefault('reply', '')
    return data


class OpenAILLM:
    """OpenAI's API, or any server that speaks the same API (Ollama, vLLM) when need_key=False."""
    def __init__(self, model='gpt-4o', url='https://api.openai.com/v1/chat/completions', timeout=60, need_key=True, extra=None):
        self.extra = extra or {}                     # extra request fields, e.g. {'reasoning_effort': 'none'} for Qwen3 on Ollama
        self.key = os.environ.get('OPENAI_API_KEY')
        if need_key and not self.key:
            raise SystemExit('set OPENAI_API_KEY in the environment first (e.g. export OPENAI_API_KEY=...; never put it in the repo)')
        self.model, self.url, self.timeout = model, url, timeout

    def chat(self, messages):
        body = json.dumps({'model': self.model, 'messages': messages, 'temperature': 0,
                           'response_format': {'type': 'json_object'}, **self.extra}).encode()
        headers = {'Content-Type': 'application/json'}
        if self.key: headers['Authorization'] = f'Bearer {self.key}'
        req = urllib.request.Request(self.url, data=body, headers=headers)
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())['choices'][0]['message']['content']


class ClaudeLLM:
    def __init__(self, model='claude-sonnet-5-5', url='https://api.anthropic.com/v1/messages', timeout=60):
        self.key = os.environ.get('ANTHROPIC_API_KEY')
        if not self.key:
            raise SystemExit('set ANTHROPIC_API_KEY in the environment first (e.g. export ANTHROPIC_API_KEY=...; never put it in the repo)')
        self.model, self.url, self.timeout = model, url, timeout

    def chat(self, messages):
        system = '\n\n'.join(m['content'] for m in messages if m['role'] == 'system')
        convo = [m for m in messages if m['role'] != 'system']
        body = json.dumps({'model': self.model, 'max_tokens': 1024, 'temperature': 0,
                           'system': system, 'messages': convo}).encode()
        req = urllib.request.Request(self.url, data=body, headers={'content-type': 'application/json', 'x-api-key': self.key,
                                                                   'anthropic-version': '2023-06-01'})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return ''.join(b.get('text', '') for b in json.loads(r.read())['content'] if b.get('type') == 'text')


class MockLLM:
    """Keyword rules standing in for the LLM. Knows the two standard tasks; asks for help on a replan."""
    NAMES = {'soup': 'soup_can', '7up': 'soda_bottle', 'bottle': 'soda_bottle', 'soda': 'soda_bottle',
             'carton': 'carton', 'cup': 'paper_cup', 'coke': 'coke_can'}

    def chat(self, messages):
        text = messages[-1]['content']
        if 'What happened so far' in text:
            return json.dumps({'reply': 'A step failed; asking the person for help.',
                               'steps': [{'skill': 'ask_for_help', 'args': {'message': 'A step failed. Please check the robot and the object.'}}]})
        cmd = text.split('\n')[0].lower()
        obj = next((v for k, v in self.NAMES.items() if k in cmd), None)
        at = re.search(r'Robot now: at (\w+)', text).group(1)
        if obj is None:
            return json.dumps({'reply': 'I did not recognise an object I can pick up.', 'steps': []})
        into_container = any(w in cmd for w in ('container', 'bin', 'drop', 'basket', ' in '))
        steps = [] if at == 'table1' else [{'skill': 'navigate', 'args': {'target': 'table1'}}]
        scripted = 'grasp_scripted' in skills.ENABLED
        if not scripted and (obj not in skills.LEARNED_OBJECTS or not into_container):
            return json.dumps({'reply': 'I can only pick up the soup can and drop it in the yellow container (scripted skills are off).',
                               'goal': None, 'steps': []})
        if obj in skills.LEARNED_OBJECTS:
            steps.append({'skill': 'pick', 'args': {'object': obj}})
        else:
            steps.append({'skill': 'grasp_scripted', 'args': {'object': obj}})
        steps.append({'skill': 'navigate', 'args': {'target': 'table2'}})
        steps.append({'skill': 'drop', 'args': {'container': 'yellow_container'}} if into_container
                     else {'skill': 'place_scripted', 'args': {}})
        goal = {'object': obj, 'destination': 'yellow_container' if into_container else 'table2'}
        return json.dumps({'reply': f'Moving the {obj.replace("_", " ")} to table 2.', 'goal': goal, 'steps': steps})


DEFAULT_MODEL = {'local': 'qwen3:8b', 'claude': 'claude-sonnet-5-5', 'openai': 'gpt-4o', 'mock': 'mock'}
LOCAL_URL = 'http://localhost:11434/v1/chat/completions'


def make(kind, model=None, url=None, think=False):
    model = model or DEFAULT_MODEL[kind]
    if kind == 'local':   # thinking off by default: Qwen3 otherwise "thinks" for ~400 tokens (14 s vs 3.7 s for a short answer, Nov 11)
        return OpenAILLM(model, url or LOCAL_URL, timeout=180, need_key=False, extra={} if think else {'reasoning_effort': 'none'})
    if kind == 'claude': return ClaudeLLM(model)
    if kind == 'openai': return OpenAILLM(model)
    return MockLLM()


def plan(llm, command, world, history=None, max_fixes=1):
    """Ask for a plan and check it against the skills and the world state. If it doesn't check out, send the
    errors back once so the model can fix them. Returns (plan dict, errors); errors empty = runnable."""
    messages = [{'role': 'system', 'content': system_prompt()},
                {'role': 'user', 'content': user_message(command, world, history)}]
    for attempt in range(max_fixes + 1):
        raw = llm.chat(messages)
        try:
            p = parse_json(raw)
            errors = skills.check_plan(p['steps'], world) or skills.check_goal(p.get('goal'), p['steps'], world)
        except (ValueError, json.JSONDecodeError) as e:
            p, errors = {'reply': '', 'steps': []}, [f'the answer was not valid JSON: {e}']
        if not errors:
            return p, []
        messages += [{'role': 'assistant', 'content': raw},
                     {'role': 'user', 'content': 'That plan cannot run: ' + '; '.join(errors) + '. Send a corrected plan, JSON only.'}]
    return p, errors
