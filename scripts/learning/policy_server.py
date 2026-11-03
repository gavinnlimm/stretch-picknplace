#!/usr/bin/env python3
"""
ACT policy server - runs on the GPU PC. The robot client (policy_client.py) sends the head image and the robot
state; this returns the next chunk of actions.

    source ~/gavin/lerobot-env/bin/activate
    python policy_server.py --checkpoint ~/gavin/outputs/act_drop_chunk20/checkpoints/last/pretrained_model --port 8000
    python policy_server.py --fake                      # no model: answers "stay where you are" (tests the link only)
    python policy_server.py --checkpoint <dir> --selftest ~/gavin/demos/<demo folder>   # check the model on a recorded demo

POST /act   {"image": <base64 JPEG, 320x240, same as demo_logger.py saves>, "state": [7 floats]}
            -> {"actions": [[base_x, lift, arm, wrist_yaw, gripper], ...], "ms": <ms>}
GET  /health -> {"ok": true, ...}
The state is [base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt], the same order the converter used.
One server serves one module (pick or drop); start one per checkpoint, on different ports.
STATUS: written but not yet run against a real checkpoint - use --selftest first.
"""
import argparse, base64, io, json, os, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from PIL import Image

N_STATE, N_ACTION = 7, 5


def decode_image(b64):
    """base64 JPEG -> HxWx3 uint8 RGB (PIL, exactly like the converter's load_image)."""
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert('RGB'), dtype=np.uint8)


def load_real(ckpt, device):
    import torch
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference
    ckpt = os.path.expanduser(ckpt)
    policy = ACTPolicy.from_pretrained(ckpt); policy.to(device); policy.eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)
    n = policy.config.n_action_steps
    print(f'loaded {ckpt}: chunk_size {policy.config.chunk_size}, n_action_steps {n}, device {device}')

    def act(rgb, state):
        obs = {'observation.images.head': rgb, 'observation.state': np.asarray(state, np.float32)}
        with torch.inference_mode():
            batch = prepare_observation_for_inference(obs, torch.device(device), None, 'stretch_re1')
            batch = pre(batch)
            chunk = policy.predict_action_chunk(batch)[:, :n]       # (1, n, 5), normalized
            chunk = post(chunk)                                      # back to metres / radians
        return chunk.squeeze(0).float().cpu().numpy()
    return act


def load_fake():
    def act(rgb, state):
        return np.tile(np.asarray(state[:N_ACTION], np.float32), (20, 1))   # hold still
    return act


def serve(act, port):
    lock = threading.Lock()

    class H(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code); self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            self._send(200, {'ok': True, 'state_dim': N_STATE, 'action_dim': N_ACTION})
        def do_POST(self):
            try:
                req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                state = [float(v) for v in req['state']]
                if len(state) != N_STATE: return self._send(400, {'error': f'state needs {N_STATE} values'})
                t0 = time.time()
                with lock: actions = act(decode_image(req['image']), state)
                self._send(200, {'actions': np.asarray(actions, float).round(5).tolist(), 'ms': round((time.time() - t0) * 1000, 1)})
            except Exception as e:
                self._send(500, {'error': repr(e)})
        def log_message(self, *a): pass
    print(f'serving on 0.0.0.0:{port}  (Ctrl+C to stop)')
    ThreadingHTTPServer(('0.0.0.0', port), H).serve_forever()


def selftest(act, demo):
    """Run the model on a recorded demo and compare with what the demo actually did next (needs the repo's converter)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from pathlib import Path
    import convert_to_lerobot as c
    demo = Path(os.path.expanduser(demo))
    rows, meta, _ = c.load_episode(demo, set(), False)
    states = [c.state_vector(r, b) for r, b in zip(rows, c.base_forward(rows))]
    print(f'{demo.name}: {len(rows)} frames')
    errs = []
    for i in range(0, len(rows) - 20, max(1, (len(rows) - 20) // 6)):
        chunk = act(c.load_image(demo / rows[i]['frame']), states[i])
        want = np.asarray([s[:N_ACTION] for s in states[i + 1:i + 1 + len(chunk)]])
        n = min(len(chunk), len(want)); e = np.abs(chunk[:n] - want[:n]).mean(axis=0); errs.append(e)
        print(f'  frame {i:4d}: mean |predicted - recorded| per [base_x lift arm yaw gripper] =', np.round(e, 3))
    print('average:', np.round(np.mean(errs, axis=0), 3), '(metres / radians; small = the model reproduces the demo)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--checkpoint', help='.../checkpoints/last/pretrained_model of a trained ACT run')
    ap.add_argument('--fake', action='store_true', help='no model; hold-still answers')
    ap.add_argument('--device', default='cuda'); ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--selftest', metavar='DEMO_DIR', help='compare the model with a recorded demo, then exit')
    a = ap.parse_args()
    if not a.fake and not a.checkpoint: sys.exit('give --checkpoint <dir> (or --fake)')
    act = load_fake() if a.fake else load_real(a.checkpoint, a.device)
    if a.selftest: return selftest(act, a.selftest)
    serve(act, a.port)


if __name__ == '__main__':
    main()
