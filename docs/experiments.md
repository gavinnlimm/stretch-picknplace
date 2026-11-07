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

1. **Model check (GPU PC, no robot):** `python policy_server.py --checkpoint ~/gavin/outputs/act_drop_chunk20/checkpoints/last/pretrained_model --selftest ~/gavin/demos/<a drop demo>`. Errors should be small on demos it trained on; if not, the loading or preprocessing is wrong. *Done Nov 3 (drop checkpoint, demo 15-34-25): average error base 1.7 cm, lift 0.5 cm, arm 1.7 cm, gripper 0.007; largest at the start of the demo (base 4.6 cm). This was a training demo, so it confirms the loading and preprocessing, not that the policy generalizes.*
2. **Network:** start the server (same command without `--selftest`), then on the robot `curl http://<gpu ip>:8000/health`. If the robot cannot reach the GPU PC, sort out a route before going further.
3. **Copy the client:** from the Mac, `scp scripts/learning/policy_client.py stretch:~/`.
4. **Dry run:** driver in gamepad mode, head at the module's demo pose, `/usr/bin/python3 ~/policy_client.py --server http://<gpu ip>:8000 --module drop`. It only prints. Drive by hand and check the printed plans point the right way.
5. **First moves:** relaunch the driver without `mode:=gamepad`, area clear, hand on the run-stop, start from a demo-like position, add `--go`. It asks for Enter before every move and clips targets to the demo joint ranges and to 10 cm (or 0.05 rad) per step.
6. **Trials:** only after 5 looks sane, run full attempts and record success by start offset, as in the test plan above.

## Drop: absolute-base vs relative-base ACT, first robot trials (Nov 5)

**Why:** the first ACT drop model (`act_drop_chunk20`) takes the base position since the start of the run as an input. Every run starts with that value at 0, so the model can learn "drive about 38 cm" instead of "drive until the container is close". The retrained model (`act_drop_rel`: same 13 drop demos, 1348 frames, same ACT settings, chunk 20, 5000 steps) leaves the base position out of the state and predicts base movement per step instead. Training: loss 9.90 -> 0.37, L1 0.51 -> 0.165. On a training demo its average error is base 0.2 cm, lift 0.8 cm, arm 1.4 cm, gripper 0.005 (this checks loading, not generalisation).

**Setup:** the can is held in the gripper, `drop_reset.py --back X` drives the robot X m back from the drop position (by odometry; the true gripper-to-container gap was not measured), every move is confirmed by hand, and the run is stopped by hand at the end.

| Model | Reset `--back` | Measured gap | Base travel before the arm extended / run ended |
|---|---|---|---|
| absolute | 0.25 m | | 0.381 m |
| absolute | 0.35 m | | 0.386 m |
| absolute | 0.50 m | | 0.386 m |
| absolute | 0.50 m | | 0.394 m |
| relative | 0.40 m | | 0.338 m |
| relative | 0.40 m | | 0.403 m |
| relative | 0.20 m | about 15 cm (along the table) | 0.200 m |
| relative | 0.35 m | about 28 cm (along the table) | 0.342 m |
| relative, `--auto-release` | 0.35 m | about 27.5 cm (along the table) | 0.330 m |
| relative, `--auto-release` | 0.50 m | not measured | 0.695 m |
| relative, `--auto-release --release-open 0.03` | 0.35 m | about 32 cm (along the table) | 0.360 m |
| relative, `--auto-release --release-open 0.03` | 0.15 m | not measured | 0.167 m |
| relative, softer start grip (`--hold -0.195`), release +0.05 | 0.15 m | not measured | 0.135 m |
| relative, softer start grip, scripted release | 0.35 m | about 33 cm (along the table) | not recorded |
| relative, softer start grip, release +0.10 | 0.50 m (see note) | about 27.5 cm (along the table) | 0.327 m |

**Result so far:** the absolute model drove 38–39 cm in all four runs although the start moved by 25 cm, so it is acting as a fixed-distance script, as suspected. For the relative model the two runs at 0.40 m differ by 6 cm, which hints at some dependence on the image, but 0.40 m is close to the distance the absolute model always drives, so these two runs cannot separate the models.

**Relative model with a measured 15 cm gap:** the base drove only 0.20 m (18 steps), then lifted and extended the arm (arm 0.44 m, lift 0.93 m, inside the demo ranges) with the base planning 0.000 m for the rest of the run. This is the first run where the travel clearly follows the start distance (0.20 m here vs 0.38 m for the absolute model from any start), so the relative-base change works for this case. With a 28 cm gap the base drove 0.342 m, so across the two measured gaps the travel is the gap plus about 5–6 cm. Two runs, not a statistical result.

**Failure in that run: no release.** The gripper only opened to -0.08 and the policy then stalled for about 8 steps, commanding almost no change (gripper target -0.07 to -0.08), so the can stayed in the gripper. The absolute-base model did release in earlier runs, so this is not a gripper or data limit in general. Hypothesis (untested): without the base position in its state the relative model has fewer cues about which phase it is in, so at the end pose (can held over the container) it settles on "stay still". The 28 cm run stalled the same way (gripper stuck at -0.076, arm 0.43–0.44, base planning 0), so it is 2 out of 2 so far. `policy_client.py --auto-release` (off by default) opens the gripper to -0.02 after 3 stalled steps; use it only to separate "did the approach put the can over the container" from "does the policy release", and report results with and without it.

**With `--auto-release` (27.5 cm gap):** the base drove 0.330 m, the policy stalled with the gripper at about -0.06 and the arm at 0.45 m, the script opened the gripper to -0.02 after 3 stalled steps, and the can dropped into the container. First complete drop with the relative model; the release itself was scripted. The demos only opened the gripper slightly (-0.02 to -0.12), so the learned opening is small and leaves little margin.

**Long start (reset 0.50 m, gap not measured):** the base drove 0.695 m, far more than the 0.38 m of the absolute model and more than the 0.20 and 0.34 m of the shorter starts, so the travel is not a fixed distance. The arm also came out early (0.18 m at the start), was pulled back in while the base kept driving, and extended again at the end. After the scripted release (gripper target -0.02) the can was still held in the gripper; the cause is not yet known.

**With `--release-open 0.03` (32 cm gap):** the scripted release opened the gripper to +0.03 and the can fell into the container. With the release at -0.02 the can fell once (27.5 cm gap) and stuck once (long start), so -0.02 has too little margin and +0.03 worked in the one run so far.

**Short start (reset 0.15 m, gap not recorded):** the base drove 0.167 m and the arm extended. The scripted release at +0.03 did not free the can this time (it stayed gripped), although it did at 32 cm. `drop_reset.py` opens the gripper to +0.10 and that freed it, so the scripted release was raised to allow up to +0.10 (the policy's own limit stays 0.05).

**Short start again, softer grip at the start and release at +0.05:** the base drove 0.135 m, the arm extended, the scripted release fired at step 12 and the can was released. Two settings changed at once (the start hold in `drop_reset.py` from -0.21 to -0.195, and the release opening from +0.03 to +0.05), so this run does not show which of the two fixed the sticking. Gripping the can less tightly is the working hypothesis, because the reset closes to -0.21 while the can stops the fingers at about -0.197, so the gripper keeps squeezing.

**Repeat at a 33 cm gap (softer start grip, scripted release):** reported as working; the base travel was not recorded.

**Release +0.10, run without Enter prompts (27.5 cm gap):** the base drove 0.327 m, the policy stalled at step 13 (17.4 s into the run, about half the time of the earlier per-step runs), the scripted release opened the gripper to +0.10 and the can fell into the container. The reset was `--back 0.50`, but the measured gap was only 27.5 cm, so the base did not drive back the full distance; measure the gap rather than trusting `--back`.

**Base travel against measured gap (relative model, six runs):** 15 cm gap -> 0.200 m, 27.5 cm -> 0.330 m and 0.327 m, 28 cm -> 0.342 m, 32 cm -> 0.360 m. The travel is the gap plus 4–6 cm each time, while the absolute model drove 0.38–0.39 m from every start.

**Not yet recorded:** whether the can landed in the container in the earlier runs (fill in the trial log).

**Next:** run the relative model at clearly different starts (about 0.20 and 0.50 m, ideally with the real gap measured). If its travel follows the start, the fix works; if it stays near 0.38 m, it is still memorising a distance. Then repeat with Diffusion Policy on the same dataset.

## Pick: relative-base ACT, first robot trials (Nov 6)

**Model:** `act_pick_rel`: the 15 pick demos converted with the relative base, ACT chunk 20, 5000 steps. On a training demo the average error is lift 0.8 cm, arm 2.1 cm, gripper 0.007.

**Start gripper opening:** the pick demos started with the gripper at -0.12 to +0.05, but even +0.04 was not wider than the can on the robot. A wider start (+0.10) was used with `--keep-open 0.10`, which holds that opening until the policy starts to close. At +0.10 the policy (which has not seen that opening) plans a gripper target of +0.02 to -0.06, i.e. it never plans a full close.

**Attempts:**
1. Arm went out to 0.435 m and the gripper passed the can without closing (stopped by hand).
2. Same: the arm reached 0.42 m, the policy stalled with the gripper target near -0.05 and did not close.
3. With `--auto-grasp` (if the arm is out and the policy has stalled for 4 steps, the script closes the gripper to -0.20): the arm reached 0.434 m with the base at +0.013 m, the script closed the gripper, and the policy then lifted the can (0.77 -> 0.89 m), pulled the arm back to 0.006 m and set the lift back to 0.78 m. The can was grasped and carried. The distance to the can was not recorded.

4. Same, with `--grasp-close -0.195 --grip-floor -0.195` (no command below -0.195): grasped, lifted (0.76 -> 0.89 m) and retracted the arm to 0.02 m; the gripper reading while carrying was -0.18 (it was -0.19 with the harder grip). The base did not move in this run (the can was within arm reach from the start), so it does not test the base following the gap for the pick.

5. Can moved about 15 cm further along the table (rough estimate, not measured): the base drove only 0.063 m (0.051 m on the first step, then about 0) and the arm went out to 0.43 m. The gripper never lined up with the can; the arm contact warning fired at the first step and the can ended up pushed against the arm, and the scripted close then caught it. Counted as a miss, not a success: the grasp worked by luck.

**What this shows:** the approach, lift and retract are learned; the decision to close is not (the policy stalls at the can, the same stall as at the drop release). The grasp was very tight: the can stops the fingers at about -0.19 while the commands were -0.20 to -0.21, so the gripper kept squeezing. `--grip-floor` stops the client commanding below a chosen value.



**Pick base: result so far.** The base moved in the right direction when the can was moved (0.00 m at the first position, 0.063 m after moving the can about 15 cm), but covered well under half of the distance. With 15 demos the pick model does not yet follow the can's position along the table reliably, unlike the drop (travel = gap + 4 to 6 cm in six runs). The next step is more pick demos with the can at varied positions along the table and at varied distances from the table edge, with the gripper starting wide, then retraining with the relative base.
