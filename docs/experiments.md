# Experiments: learned pick and drop (ACT)

**Nothing here has results yet.** Fill the tables in only with numbers from real runs.

## Task

Pick up an object and drop it into a yellow container on the table. Six steps:

1. Drive to Table 1
2. Align to the object at Table 1
3. Grasp
4. Drive to Table 2
5. Align to the container at Table 2
6. Drop

Two modules, trained separately: **Module 1** (steps 2–3, pick up at Table 1) and **Module 2** (steps 5–6, drop a held object into the yellow container at Table 2). Navigation (steps 1 and 4) stays on Nav2. A learned navigation policy (for example NoMaD) is a later idea and probably won't work on this setup; to be checked.

## Setup (keep identical across demos and trials)

- Policy: ACT (LeRobot 0.6.1), trained on the lab GPU (RTX 5090).
- Cameras: head camera only. The RE1 has no wrist camera.
- Head pose fixed for every demo and trial; yellow container and table positions fixed.
- Baseline: the scripted pipeline (`grasp_auto.py`, `pick_place.py`), with results in the README.

## Test plan

1. **Grasp only (step 3).** Run steps 1–2 with the scripted pipeline so the robot starts aligned, and let ACT do only the grasp. This isolates the grasp from alignment. Compare to the scripted grasp on held-out trials (objects and positions not used in training).
2. **Alignment plus grasp (steps 2–3).** Only if (1) works. Record demos from varied start offsets (about ±5, ±10 and ±15 cm along the table, since Nav2 often leaves the robot 10–17 cm off) and include the corrections. Report success **by offset**.
3. **Drop only (step 6)**, then **alignment plus drop (steps 5–6)**, the same way.
4. Optional comparison: train Diffusion Policy on the same dataset and compare on identical trials.

## Demo recording protocol

1. Camera tab, driver tab (gamepad mode), recorder tab. Camera at about 15 Hz, controller moving the robot.
2. Fix the scene: object and yellow container in place, head pointed so the camera sees both. Do not move the head afterward.
3. Practice 5–10 times without recording until the motion is smooth and repeatable.
4. Record one test demo and inspect `~/demos/<folder>`: frames in order, object and container visible, about 10 frames per second of demo, plus `steps.jsonl` and `meta.json`.
5. Record 10–20 demos (several objects and table positions, a few with deliberate corrections) to test the pipeline. The full set (50 or more) comes after the pipeline is proven.
6. Label each demo success or failure. For the drop, only an object inside the container counts as success.

## Trial log (real runs only)

| Date | Module | Policy | Object | Start offset (cm) | Success? | Failure cause | Run time (s) |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

Failure causes to use: object hidden from the camera, misaligned, gripper missed, slipped, missed the container, other (describe).

## Open questions

- Can ACT absorb the alignment, or is a scripted alignment step needed? Hypothesis: it can learn small corrections if the demos include varied starts and the corrections. To be tested.
- Does a held object block the head camera's view of the container at Table 2? Check the head pose.
- Is the head camera enough, or do the failures point to needing a wrist camera?

## Robot client bring-up (not yet run)

`scripts/learning/policy_server.py` (GPU PC) loads a trained ACT checkpoint and answers requests; `scripts/learning/policy_client.py` (robot) sends the head image and state, and moves toward the returned targets. Offline checks so far: the client builds exactly the same state vector as the converter on a real demo, its images match the training images (mean difference 0.14/255), the request/response path works against a fake server, and the safety clipping rejects bad chunks. Nothing has moved the robot yet. Bring it up in this order, one module at a time (drop first):

1. **Model check (GPU PC, no robot):** `python policy_server.py --checkpoint ~/gavin/outputs/act_drop_chunk20/checkpoints/last/pretrained_model --selftest ~/gavin/demos/<a drop demo>`. Errors should be small on demos it trained on; if not, the loading or preprocessing is wrong.
2. **Network:** start the server (same command without `--selftest`), then on the robot `curl http://<gpu ip>:8000/health`. If the robot cannot reach the GPU PC, sort out a route before going further.
3. **Copy the client:** from the Mac, `scp scripts/learning/policy_client.py stretch:~/`.
4. **Dry run:** driver in gamepad mode, head at the module's demo pose, `/usr/bin/python3 ~/policy_client.py --server http://<gpu ip>:8000 --module drop`. It only prints. Drive by hand and check the printed plans point the right way.
5. **First moves:** relaunch the driver without `mode:=gamepad`, area clear, hand on the run-stop, start from a demo-like position, add `--go`. It asks for Enter before every move and clips targets to the demo joint ranges and to 10 cm (or 0.05 rad) per step.
6. **Trials:** only after 5 looks sane, run full attempts and record success by start offset, as in the test plan above.
