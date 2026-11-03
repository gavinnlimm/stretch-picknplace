# Gamepad teleop on the Stretch RE1

Used for recording teleoperated demos (Nov 3, 2026). The Xbox 360 wireless controller with its USB receiver works. The Xbox One controller did not work.

## What works

- Xbox 360 wireless controller plus the Xbox 360 wireless receiver (USB `045e:028e`).
- Plug the receiver into the robot, then turn the controller on. It connected without any pairing steps.
- Launch the driver in gamepad mode:

  ```
  source ~/ros_env.sh
  ros2 launch stretch_core stretch_driver.launch.py mode:=gamepad
  ```

- Clear the area and keep a hand on the run-stop before moving anything.

## What did not work

- Xbox One controller (tried over USB and Bluetooth).
- Logitech F710 gamepad.

## Notes

- The Logitech Unifying receiver is for the lab's wireless keyboard and mouse. It is not a gamepad receiver.
- In `mode:=gamepad` the driver refuses trajectory goals (`Cannot execute goals while in mode=gamepad`), so you cannot set the head pose with `follow_joint_trajectory` while it runs. Switch the driver out of gamepad mode first, or move the head with the gamepad.
- The demo recorder (`record_demo.sh`) only needs the driver's joint states, so it works with the gamepad, keyboard teleop or any other way of driving.
- Fallback: relaunch the driver without `mode:=gamepad`, then `export TERM=xterm-256color` and `ros2 run stretch_core keyboard_teleop` (keys 8/2 lift, w/x arm, 5/0 gripper, 4/6 base, s small steps, q quit).
- Press Ctrl+C once on the old driver and wait for it to exit before relaunching.
