# Gamepad (Xbox One controller) teleop on the Stretch RE1

Notes from Nov 2, 2026. The controller worked with the driver in gamepad mode at the end of that session, but the exact step that fixed it was not isolated.

> **TODO: confirm.** The sequence that most likely made it work: gamepad receivers unplugged, the controller paired over Bluetooth with `bluetoothctl`, then the driver launched in `mode:=gamepad` while the controller was awake. Replace this note with the confirmed sequence.

## How the Stretch gamepad mode works

- Gamepad mode runs inside `stretch_driver` (there is no separate gamepad node).
- `stretch_body/gamepad_controller.py` reads events with the `inputs` library (`ABS_X`, `BTN_SOUTH`, and so on) and checks for devices whose name starts with `xpad`. Its comments say the Xbox One controller was tested **over a USB cable**.
- Topics: `/is_gamepad_dongle` (does the driver see a gamepad) and `/stretch_gamepad_state` (what it reads).
- With no gamepad found the driver logs `Gamepad Dongle DISCONNECTED........`.
- Read Hello Robot's gamepad documentation for the control layout before moving the arm. The mapping is not documented here yet.

## Pairing the Xbox One controller (Bluetooth)

The controller connects to one device at a time. Forget it on any phone or console first.

1. Unplug other gamepad receivers from the robot (see below).
2. In a robot tab: `bluetoothctl`, then

   ```
   power on
   agent on
   default-agent
   remove <MAC>
   scan on
   ```

3. Hold the small pair button on the controller's top edge for about 3 seconds, until the light flashes fast.
4. When `[NEW] Device <MAC> Xbox Wireless Controller` appears, run `pair <MAC>`, `trust <MAC>`, `connect <MAC>` right away. The pairing window is short. This controller's MAC was `EC:83:50:E0:E0:86`.
5. Do not type `scan off` or `exit` until the device exists. Wiggle a stick, because the controller sleeps after a few idle minutes and drops its input device.
6. Check it registered:

   ```
   ls /dev/input/js*
   grep -E "Name=|Handlers=" /proc/bus/input/devices | grep -B1 js
   ```

   You want an `Xbox Wireless Controller` entry with a `jsN`. Commands like `ls` do not run inside `bluetoothctl`.
7. Test live input (use your `js` number):

   ```bash
   python3 - <<'EOF'
   import struct
   f = open('/dev/input/js0','rb')
   print('READY - move the sticks now')
   while True:
       t,v,ty,n = struct.unpack('IhBB', f.read(8))
       if ty & 0x80: continue
       print('button' if ty==1 else 'axis', n, v)
   EOF
   ```

8. Launch the driver while the controller is awake (Ctrl+C **once** on the old driver, wait, then):

   ```
   ros2 launch stretch_core stretch_driver.launch.py mode:=gamepad
   ```

   Check `ros2 topic echo /is_gamepad_dongle --once` (should say `true`). Clear the area and keep a hand on the run-stop before pressing anything.

## Troubleshooting

- `connect` fails with `br-connection-create-socket`: wake the controller first, or `remove` it and pair again. `echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm` followed by `sudo systemctl restart bluetooth` helped some of the time (it resets on reboot).
- Linked (`Paired: yes`, `Connected: yes`) but no `js` device: `sudo modprobe joydev` and `sudo modprobe uhid`, wake the controller, check again.
- A small random-looking Bluetooth address that was already in the list before pairing mode is somebody's phone or earbuds, not the controller. The controller shows up with a name.
- If nothing works, fall back to keyboard teleop: relaunch the driver without `mode:=gamepad`, then `export TERM=xterm-256color` and `ros2 run stretch_core keyboard_teleop` (keys 8/2 lift, w/x arm, 5/0 gripper, 4/6 base, s small steps, q quit). The demo recorder only needs the driver's joint states, so it works either way.

## Receivers on the robot

These all take a `js` number and can confuse the driver:

| Device | USB ID | Notes |
|---|---|---|
| Xbox 360 wireless receiver ("GAME FOR WINDOWS") | `045e:028e` | Red flashing light. Cannot talk to an Xbox One controller. Registered as `js0`. |
| Logitech F710 receiver | `046d:c21f` | Only for the F710 gamepad. |
| Logitech Unifying Receiver | `046d:c534` | The lab's wireless keyboard and mouse. Not related to gamepads. |

## Raw axis map seen on `js1` (not verified against the Stretch layout)

- axes 0 and 1: left stick (x, y)
- axes 2 and 3: right stick
- axes 4 and 5: triggers (rest at -32767)
- axes 6 and 7: d-pad
- buttons 0, 1, 3, 4: face buttons; buttons 6 and 7 look like the shoulder buttons
