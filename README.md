# Stretch RE1: pick and place, and learning from demonstrations (HRC² Lab)

This repo has two parts:

1. **A scripted pipeline** that moves an object from Table 1 to Table 2 on the lab's Stretch RE1, using Nav2 to drive and the head camera to grasp and place. It works on 5 different objects and is the baseline.
2. **A learning pipeline** (record teleoperated demos, convert them to LeRobot format, train ACT on the lab GPU) that aims to replace the scripted grasp and drop. Demos and first training runs exist. **Nothing learned has been run on the robot yet.**

## Run the scripted pipeline

```bash
bash ~/full_auto_obj.sh <object_name>      # soup_can, carton, soda_bottle, paper_cup, coke_can
```

The scripts live in `~/` on the robot (`hello-robot@stretch-re1-1050`); this repo groups them in folders. The robot's IP changes when it restarts or changes networks (`hostname -I` on the robot). Put it in one place with a `Host stretch` entry in `~/.ssh/config`, then `ssh stretch`.

### What one run does

1. **nav1:** Nav2 to the Table 1 staging spot, then `auto_fix.py` (AMCL refresh, turn-drive-turn onto the table pose). Files: `t1.sh`, `go_t1_stage.sh`.
2. **grasp:** depth only. Scan with the head, find the object and measure its height and width, drive along the table, side-step if more than 46 cm away, lower and extend to a size-based height, close to the measured width, lift and retract (`grasp_auto.py`, `can_detect.py`).
3. **nav2:** Nav2 to the Table 2 staging spot, then `auto_fix.py`. Files: `t2.sh`, `go_t2_stage.sh`.
4. **align2:** find tag 7 with the head camera while the object is held low (`aruco_align.py`).
5. **place:** reach by the measured distance, lower by the same offset used at the grasp (`~/grasp_last.json`), release, lift clear, retract (`pick_place.py place`).

Each run logs stage times to `~/object_trials.csv` and is saved as a demo in `~/demos/` (head image, joint states, odometry and stage at 10 Hz). Options: `RECORD=0` skips the demo, `ADAPT=0` uses the old fixed grasp, `ALIGN_LIFT=<m>` sets the carry height during the tag search, `NO_OVERLAP=1` runs arm moves one after the other.

### Key values

- Map `lab_map_v3`, home = (0, 0). Table 1 pose (-1.146, 0.345, -4.2°), Table 2 pose (-0.660, 1.682, -178.9°).
- Tag 7 (`DICT_6X6_250`, 8 cm) is the Table 2 place target. Tag 8 is only stored in `aruco_cal.json` (search region and table height).
- Adaptive grasp: grasp lift = 0.7447 + half the object's height above 11 cm (max +8 cm); carry lift 0.86; the gripper closes to the measured width minus 1.5 cm using `gripper_cal.json`. Redo that calibration with `python3 ~/gripper_cal.py` (run-stop released).
- Detection (`can_detect.py`): points 2–30 cm above the table, object top 8–25 cm, width under 11 cm.

## Starting up

1. Robot on the home tape, area clear, run-stop solid, charger unplugged.
2. In every robot tab: `ssh stretch` from your Mac, then `source ~/ros_env.sh`. Without it conda's Python runs and `rclpy` fails to import. Never type `ssh stretch` inside a robot tab.
3. Home: `stretch_free_robot_process.py; stretch_system_check.py; stretch_robot_home.py`. A `FileNotFoundError ... stretch_body_robot_pid.txt.lock` here is harmless.
4. Nav tab: `ros2 launch stretch_nav2 navigation.launch.py use_rviz:=false map:=${HELLO_FLEET_PATH}/maps/lab_map_v3.yaml`
5. Camera tab: `ros2 launch stretch_core d435i_high_resolution.launch.py`. Check `ros2 topic hz /camera/color/image_raw` (about 15 Hz). If frames stop, unplug and replug the camera USB.
6. Set the start pose: publish `/initialpose` (x 0, y 0, w 1), then call `/request_nomotion_update` about 10 times. `bash ~/start_bg.sh` does steps 4–6 in the background.
7. Put the object on the Table 1 spot, keep tag 7 clear, run the command at the top.

**Copying scripts to the robot.** The robot only runs what is in its home directory. After changing a script, copy it from your Mac (not from a robot tab):

```bash
cd ~/stretch-picknplace
scp scripts/learning/demo_logger.py scripts/learning/record_demo.sh scripts/full_auto_obj.sh stretch:~/
```

Press Ctrl+C **once** to stop the driver and wait for it to exit.

## Results (Oct 2026)

Soup can 6/6, avg ~91 s. Adaptive grasp, one run each, all successful:

| Object | Height, width | Grasp height | Total |
|---|---|---|---|
| 7UP bottle | 21.4 cm, 6.7 cm | +5.3 cm | 88 s |
| Soup can | 10.9 cm, 8.0 cm | +0.0 cm | 101 s |
| Coke can | 12.3 cm, 6.2 cm | +0.7 cm | 103 s |
| Paper cup | 11.3 cm, 8.8 cm | +0.1 cm | 105 s |
| Carton | 14.2 cm, 5.9 cm | +1.6 cm | 109 s (incl. side-step) |

Earlier carton runs: 3/3, avg 115 s. Per-run times are in `results/object_trials.csv`.

## Known limits

- Detection only accepts objects 8–25 cm tall and under 11 cm wide, in a strip around the stored tag position. The 7 cm reach offset is fixed.
- Objects narrower than a 5 cm gap close only to the narrowest calibrated point.
- Nav2 often stops 10–17 cm too far from Table 1, which costs a side-step (about 9 s).
- Table 2 needs tag 7 visible; a tall carried object can block it unless held low.
- `aruco_align.py` does not restore navigation mode if it exits early.

## Learning from demonstrations (ACT)

**Goal:** replace the scripted grasp and place with learned policies for a new **pick and drop** task: pick an object at Table 1, then drop it into a yellow container at Table 2. It is split into two modules trained separately: *pick* and *drop*. The head camera is the only camera (the RE1 has no wrist camera). Test plan: [docs/experiments.md](docs/experiments.md).

**Status (Nov 3, 2026)**

- 36 teleoperated demos recorded with the Xbox 360 gamepad. Usable: **13 drop** and **15 pick** demos.
- LeRobot 0.6.1 datasets built on the lab GPU PC: drop 13 episodes (1348 frames), pick 15 episodes (2033 frames).
- ACT on the drop set: a 200-step test run, then 5000 steps (chunk size 20, about 6.5 min). Loss fell from about 6.2 to about 0.5. This is **training loss only**, on 13 demos, with no held-out check.
- **Not done:** no policy has been run on the robot, and there is no learned success rate. The robot client is written and checked offline but has not moved the robot.

**Recording rules**

- Keep the head fixed within each set: pick pan -1.29, tilt -0.75; drop pan -1.38, tilt -0.60. Images are saved as recorded (rotated 90°); the future robot client must use the same orientation.
- No base turning in demos. The data only records forward distance (`base_x`), and the arm reaches sideways, so forward is along the table. Start inside arm reach of the table.
- Vary the start position along the table between demos. `base_x` is measured from each demo's start, so identical starts would teach the policy nothing about correcting a miss.
- Mark a correction only for a real recovery from a mistake.

**Workflow**

1. **Record (robot):** drive with the gamepad ([docs/gamepad_setup.md](docs/gamepad_setup.md)) and run `bash ~/record_demo.sh soup_can grasp` for the pick, or `... soup_can drop` for the drop. Press Enter to stop, then answer success and correction. An optional wrist camera can be added with `WRIST_TOPIC=<image topic> bash ~/record_demo.sh ...` and `--wrist` in the converter; keyboard teleop is the fallback if the gamepad fails. Each demo is saved to `~/demos/<date>_teleop_<task>_<object>/` with `meta.json` (stage, result, correction).
2. **Copy** `~/demos` to the GPU PC, for example robot to Mac to GPU with `rsync -a`.
3. **Convert (GPU PC, `~/gavin/lerobot-env`):** one dataset per module.

   ```bash
   python scripts/learning/convert_to_lerobot.py --demos ~/gavin/demos --out ~/gavin/datasets/stretch_pick --stages grasp,teleop_grasp
   python scripts/learning/convert_to_lerobot.py --demos ~/gavin/demos --out ~/gavin/datasets/stretch_drop --stages teleop_drop --source teleop
   ```

   State is `[base_x, lift, arm, wrist_yaw, gripper, head_pan, head_tilt]`; the action is the next step's `[base_x, lift, arm, wrist_yaw, gripper]` (absolute), at 10 Hz. `base_x` is the forward distance since the first kept frame, projected on the starting heading. The converter keeps only successful demos, prints how many were kept and skipped and why, and warns if the head moved. To exclude a demo, set its `result` in `meta.json` to something other than `OK`.
4. **Train:**

   ```bash
   nohup lerobot-train --policy.type=act --dataset.repo_id=local/stretch_drop --dataset.root=$HOME/gavin/datasets/stretch_drop --policy.push_to_hub=false --policy.device=cuda --policy.chunk_size=20 --policy.n_action_steps=20 --num_workers=8 --batch_size=8 --steps=5000 --output_dir=$HOME/gavin/outputs/act_drop_chunk20 > act_drop.log 2>&1 &
   ```

   `--num_workers=8` matters: with the default, data loading took 0.65 s per step against 0.07 s on the GPU. Diffusion Policy and VQ-BeT also run on the same datasets as comparisons. See the [LeRobot ACT docs](https://huggingface.co/docs/lerobot/act).
5. **Deploy (written, not yet run on the robot):** `scripts/learning/policy_server.py` serves a checkpoint from the GPU PC; `scripts/learning/policy_client.py` runs on the robot, sends the head image and state, and moves toward the returned targets. It is a dry run unless `--go` is given, asks for Enter before every move, and clips targets to the demo joint ranges. Bring-up steps: [docs/experiments.md](docs/experiments.md#robot-client-bring-up-not-yet-run).

## Back up from the robot

`bash backup.sh` (robot on; uses the `stretch` ssh alias), or `ROBOT=hello-robot@<ip> bash backup.sh`.
