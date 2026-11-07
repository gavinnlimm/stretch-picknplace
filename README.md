# Stretch RE1: pick and place, and learning from demonstrations (HRC² Lab)

Two parts:

1. **Scripted pipeline (the baseline).** Moves an object from Table 1 to Table 2 on the lab's Stretch RE1. Nav2 drives, the head camera finds the object and the place tag. Works on 5 objects.
2. **Learning pipeline.** Record teleoperated demos, train ACT (LeRobot) on the lab GPU, run the policy on the robot. Goal: replace the scripted grasp and drop with learned **pick** and **drop** modules. First robot trials work; see [Learning](#learning-from-demonstrations-act).

## Scripted pipeline

```bash
bash ~/full_auto_obj.sh <object_name>      # soup_can, carton, soda_bottle, paper_cup, coke_can
```

The scripts live in `~/` on the robot (`hello-robot@stretch-re1-1050`); this repo groups them in folders. The robot's IP changes when it restarts (`hostname -I`). Put it in `~/.ssh/config` as `Host stretch`, then `ssh stretch`.

**One run:**

1. **nav1:** Nav2 to the Table 1 spot, then `auto_fix.py` corrects the pose.
2. **grasp:** depth only. Scan with the head, measure the object, drive along the table, lower and extend to a size-based height, close to the measured width, lift, retract (`grasp_auto.py`, `can_detect.py`).
3. **nav2:** Nav2 to the Table 2 spot, then `auto_fix.py`.
4. **align2:** find tag 7 with the head camera while the object is held low (`aruco_align.py`).
5. **place:** reach by the measured distance, lower, release, lift clear, retract (`pick_place.py place`).

Each run logs times to `~/object_trials.csv` and saves a demo in `~/demos/`. Options: `RECORD=0` no demo, `ADAPT=0` old fixed grasp, `ALIGN_LIFT=<m>` carry height during the tag search, `NO_OVERLAP=1` arm moves one at a time.

**Key values:** map `lab_map_v3`, home = (0, 0). Table 1 (-1.146, 0.345, -4.2°), Table 2 (-0.660, 1.682, -178.9°). Tag 7 (`DICT_6X6_250`, 8 cm) is the place target. The gripper closes to the measured width minus 1.5 cm (`gripper_cal.json`; redo with `python3 ~/gripper_cal.py`, run-stop released). Detection: object top 8–25 cm, width under 11 cm.

### Starting up

1. Robot on the home tape, area clear, run-stop solid, charger unplugged.
2. In every robot tab: `ssh stretch` from your Mac, then `source ~/ros_env.sh` (otherwise `rclpy` fails to import). Never type `ssh stretch` inside a robot tab.
3. Home with an **empty gripper**: `stretch_free_robot_process.py; stretch_system_check.py; stretch_robot_home.py`. A `...stretch_body_robot_pid.txt.lock` error here is harmless.
4. Nav tab: `ros2 launch stretch_nav2 navigation.launch.py use_rviz:=false map:=${HELLO_FLEET_PATH}/maps/lab_map_v3.yaml`
5. Camera tab: `ros2 launch stretch_core d435i_high_resolution.launch.py`. Check `ros2 topic hz /camera/color/image_raw` (about 15 Hz); if frames stop, replug the camera USB.
6. Set the start pose: publish `/initialpose` (x 0, y 0, w 1), then call `/request_nomotion_update` about 10 times. `bash ~/start_bg.sh` does steps 4–6.
7. Put the object on the Table 1 spot, keep tag 7 clear, run the command above.

**Copying scripts to the robot.** The robot only runs what is in its home directory. After editing, copy from your Mac (not a robot tab):

```bash
cd ~/stretch-picknplace
scp scripts/learning/policy_client.py scripts/learning/pick_reset.py scripts/learning/drop_reset.py stretch:~/
```

### Results (Oct 2026)

Soup can 6/6, average about 91 s. Adaptive grasp, one run each, all successful:

| Object | Height, width | Total |
|---|---|---|
| 7UP bottle | 21.4 cm, 6.7 cm | 88 s |
| Soup can | 10.9 cm, 8.0 cm | 101 s |
| Coke can | 12.3 cm, 6.2 cm | 103 s |
| Paper cup | 11.3 cm, 8.8 cm | 105 s |
| Carton | 14.2 cm, 5.9 cm | 109 s (with side-step) |

Per-run times: `results/object_trials.csv`.

**Known limits:** objects must be 8–25 cm tall and under 11 cm wide; Nav2 often stops 10–17 cm too far from Table 1 (costs a side-step, about 9 s); Table 2 needs tag 7 visible; `aruco_align.py` does not restore navigation mode if it exits early.

## Learning from demonstrations (ACT)

**Task:** pick the soup can at Table 1, drop it into a yellow container at Table 2. Two modules, trained separately: *pick* and *drop*. The head camera is the only camera. Full results and trial logs: [docs/experiments.md](docs/experiments.md).

**Where it stands (Nov 6, 2026).** Everything below is a handful of runs, not a success rate.

- **Drop:** the first model followed the gap but stalled at the release: its demos ended with the arm still out and only a shallow release. A new model trained on 10 demos with a wide release and the arm pulled in (`act_drop_rel2_new`) released and retracted by itself in 5 of 7 attempts (gaps 28 to 35 cm); both failures were freezes at the start at one spot. Small sample, one can, one setup.
- **Pick:** the model trained on 30 demos picked the can in 5 of 5 runs with the can 8, 18, 28, 36 and 42 cm along the table, with no scripted help (base travel within 3.7 cm of the gap). One setup, one can, five runs: not a general success rate. The robot moves in a stop-and-go way (one step about every 1.8 s).
- **Key finding:** the state must not contain the absolute base position. It starts at 0 every run, so the first model learned a fixed ~38 cm drive. Training with a relative base (`--base-mode relative`) fixed that.
- **Not done:** no success rates, no comparison with Diffusion Policy yet.

**Recording rules**

- Keep the head fixed within each set: pick pan -1.29, tilt -0.75; drop pan -1.38, tilt -0.60.
- No base turning. The base only drives forward along the table; the arm reaches sideways. Vary the start position and arm distance between demos.
- Start the pick demos with the gripper wide open, wider than the can.
- End each drop demo with the gripper wide open and the arm pulled all the way in, and only then stop the recording. Demos that stop at the release, with the arm out, teach a policy that stalls there.
- Mark a correction only for a real recovery from a mistake.

**Workflow**

1. **Record (robot):** drive with the gamepad ([docs/gamepad_setup.md](docs/gamepad_setup.md)) and run `bash ~/record_demo.sh soup_can grasp` (pick) or `... soup_can drop`. Press Enter to stop, then answer success and correction. Demos go to `~/demos/<date>_teleop_<task>_<object>/`.
2. **Copy** `~/demos` to the GPU PC (robot to Mac to GPU with `rsync -a`).
3. **Convert (GPU PC, `~/gavin/lerobot-env`):**

   ```bash
   python scripts/learning/convert_to_lerobot.py --demos ~/gavin/demos --out ~/gavin/datasets/stretch_pick_rel --stages grasp,teleop_grasp --base-mode relative
   python scripts/learning/convert_to_lerobot.py --demos ~/gavin/demos --out ~/gavin/datasets/stretch_drop_rel --stages teleop_drop --source teleop --base-mode relative
   ```

   Data are at 10 Hz. With `relative`, the state is `[lift, arm, wrist_yaw, gripper, head_pan, head_tilt]` and the action includes `base_dx` per step. Only demos with `result` OK are kept; the converter prints what it skipped and why.
4. **Train:**

   ```bash
   nohup lerobot-train --policy.type=act --dataset.repo_id=local/stretch_pick_rel --dataset.root=$HOME/gavin/datasets/stretch_pick_rel --policy.push_to_hub=false --policy.device=cuda --policy.chunk_size=20 --policy.n_action_steps=20 --num_workers=8 --batch_size=8 --steps=8000 --output_dir=$HOME/gavin/outputs/act_pick_rel > act_pick.log 2>&1 &
   ```

   Keep `--num_workers=8`; the default makes data loading 10× slower than the GPU. `--policy.type=diffusion` trains Diffusion Policy on the same data for comparison.
5. **Serve (GPU PC):** `python ~/gavin/policy_server.py --checkpoint <run>/checkpoints/last/pretrained_model --port 8000`. Check `curl http://localhost:8000/health` says `"base_mode": "relative"`.
6. **Tunnel (robot, own tab):** `ssh -N -L 8000:localhost:8000 cornell@<gpu-ip>`. The robot reaches the GPU only this way.
7. **Run (robot):** reset first, then the client. It moves right away; Ctrl+C stops it. Add `--confirm` to be asked before each move.

   ```bash
   yes '' | /usr/bin/python3 ~/pick_reset.py --grip 0.13 && /usr/bin/python3 ~/policy_client.py --server http://localhost:8000 --module pick --go --seconds 90 --base-mode relative
   ```

   For the drop, use `drop_reset.py --back <gap in m>` and `--module drop`. `--stop-when-done` ends a run on its own once the task is finished. Scripted helps (`--hold-grip`, `--auto-grasp`, `--keep-open`, `--auto-release`) are fallbacks: always report results with and without them. Push the robot back to its start before each trial.

## Back up from the robot

`bash backup.sh` (robot on; uses the `stretch` ssh alias), or `ROBOT=hello-robot@<ip> bash backup.sh`.
