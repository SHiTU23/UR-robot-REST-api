# UR REST API

A REST API for a **Universal Robots arm (UR10e) with an OnRobot RG6 gripper** (or a Robotiq 2F-85, see
[Grippers](#grippers)), on **ROS 2 Jazzy + MoveIt 2** (Universal Robots' `ur_robot_driver`), in Docker.

It speaks the same HTTP contract as the Franka API,
[XiaoxiaoZ/Franka-Emika-Panda-robot-REST-api](https://github.com/XiaoxiaoZ/Franka-Emika-Panda-robot-REST-api).
The endpoint names, parameters, JSON fields, outcome names and the scene-hash definition are all the same. A
client written for the Franka, such as pipeFit's `grasp_try.Robot`, can therefore drive this arm by changing
the URL. Its joint vectors have 6 values instead of 7, and its frames and gripper are the UR ones (see
[Differences](#differences-from-the-franka-api)).

```
alias api=curl
api http://localhost:5000/help                                   # every endpoint, generated from the code
api http://localhost:5000/state                                  # TCP pose, gripper, joints
api "http://localhost:5000/control/goto_joints?joints=0,-100,100,-90,-90,0&unit=deg&dry_run=1"
api "http://localhost:5000/control/plan_cartesian_path?x=0.6&y=0.2&z=0.3&roll=180&pitch=0&yaw=0"   # MOVES THE ARM
api "http://localhost:5000/control/gripper_grasp?width=0.013&force=40&eps_out=0.02"
curl http://localhost:5000/control/stop
open http://localhost:5000/ui                                    # browser panel: state, jog, gripper, force, camera, STOP
```

## What runs

```
 ┌─ ur-ros (docker) ──────────────────────────────┐      ┌─ ur-api (docker) ─────────────────────┐
 │ ur_robot_driver (headless) ── UR controller    │      │ Flask server (ur_api/server.py)       │
 │ move_group (OMPL + Pilz), cell URDF/SRDF       │◀────▶│  motion.py   plan / execute / stop    │
 │ finger_state_publisher (RG6 joint for MoveIt)  │ ROS 2│  gripper.py  RG6 via pendant programs │
 └────────────────────────────────────────────────┘      │  scene_*.py  collision scene (v1, v2) │
         ▲ RTDE / Dashboard (29999) / reverse ports       │  camera.py   Orbbec Gemini 2 / D4xx   │
         └──────────────── the robot ─────────────────────┴───────────────────────────────────────┘
```

| path | content |
|---|---|
| `config/cell.yaml` | **everything cell-specific**, one value per line: robot IP, home pose, tool changer, which gripper, RG6 programs and registers, camera, fixed collision boxes. `MEASURE` marks estimates |
| `config/grippers/` | one file per gripper model: links, driving joint, limits, and the width table computed from the model |
| `ros/urdf/` | `ur_cell.urdf.xacro`: the driver's UR description plus the tool on the flange (`cell_tool.xacro`: tool changer, the gripper from `grippers/`, camera at its hand-eye pose, `tcp` link) |
| `ros/meshes/` | the RG6 meshes (MIT, from BrettRD/onrobot_grippers). The 2F-85 comes from `ros-jazzy-robotiq-description` |
| `ros/srdf/` | planning group `ur_manipulator`, `base_link` → `tcp`; tool/wrist collision exclusions (from the gripper's link list) |
| `tools/gripper_table.py` | computes a gripper's width table from its model |
| `ros/launch/cell.launch.py` | driver + move_group + finger states |
| `ur_api/` | the server (see the table below) |
| `tests/` | unit tests, the end-to-end `api_smoke.py`, URSim set-up |

| module | does |
|---|---|
| `server.py` | builds everything, `/`, `/help`, `/ui`, the motion gate (dirty scene / stale `expect_scene_version` → 409) |
| `routes_robot.py` | `/state`, `/joints`, `/control/*`, gripper, `/force/*`, `/health`, `/recover`, `/simulation/*`, `/robot/*` |
| `routes_plan.py` | `/plan/check_state`, `dry_run`, `ik`, `check_trajectory`, `dry_run_sequence` (never move) |
| `routes_scene.py`, `scene_core.py`, `legacy_scene.py`, `scene_io.py` | `/scene/*`: v2 batches with confirmed diffs, rollback and dirty namespaces; models/objects; ACM; padding |
| `motion.py` | MoveIt 2 through its services and actions; guards; stop; protective-stop handling; recovery |
| `kinematics.py` | URDF FK + all IK configurations (closed-form UR solution refined on the URDF) |
| `gripper.py`, `rtde_regs.py`, `dashboard.py` | the RG6 sequence (registers → pendant program → control back to the driver) |
| `force.py`, `payload.py` | flange F/T sensor: tare, compensation, RViz arrows, payload identification |
| `camera.py`, `routes_io.py` | `/camera/*` (Orbbec Gemini 2, RealSense or none), `/files/*` |

## Running it

```bash
./run.sh mock                 # no robot: mock hardware + simulated gripper   -> http://localhost:5000
./run.sh ursim                # URSim (UR10e) in docker + the real driver      -> http://localhost:5000
./run.sh up                   # the real arm, at robot.ip in config/cell.yaml (or: ./run.sh up <ip>)
./run.sh test [url] [ursim ip]   # end-to-end checks (MOVES the robot it talks to)
./run.sh view rg6             # mock robot with that gripper + RViz (also: view robotiq_2f85)
./run.sh rviz                 # RViz on whatever stack runs
./run.sh logs | down
./run.sh token                # the API token
./run.sh save-start           # the real arm's current pose becomes the mock robot's start pose (then restart)
```

## Grippers

Three ways pick the gripper, the first that is set wins:
1. `UR_GRIPPER=<model>` in front of any `run.sh` command (the web page then cannot switch it).
2. **The web page** (`/ui`, Gripper card, *model* + *Switch*), or `POST /gripper/model {"model": "robotiq_2f85"}`:
   saved in `state/gripper_model`, then the API and the ROS side restart with it (about 30-60 s, the arm does
   not move; refused with 409 while anything runs). `GET /gripper/models` lists the models and the one in use.
   The ROS side follows through `ros/scripts/cell.sh`, which exits when the choice changes so Docker restarts it.
3. `gripper.model` in `config/cell.yaml`.

Switch only when that gripper is really on the flange. The robot model (URDF), MoveIt's collision exclusions (SRDF), the finger display, the TCP and
the API's limits all follow it. The tool changer, camera and bracket stay the same.

| model | from | on the real arm | width | force | TCP (flange → closed fingertips) |
|---|---|---|---|---|---|
| `rg6` | MIT meshes from [BrettRD/onrobot_grippers](https://github.com/BrettRD/onrobot_grippers) | pendant programs (below) | 0–160 mm | 25–120 N | 0.322 m, **measured** |
| `robotiq_2f85` | official `ros-jazzy-robotiq-description`, on Robotiq's UR adapter | pendant program `aikido.urp` (below; was `uw_wara.urp`), **not yet tried on the arm** | 0–85 mm | 20–235 N | 0.235 m, **derived** (same flange + changer, adapter + model) |

- Both grippers close their fingers on a linkage, so the finger joint is an angle, not half the width.
  `config/grippers/<model>.yaml` holds the table between joint angle, width and fingertip depth. It is
  computed from the model's collision meshes by `tools/gripper_table.py`. The model's widths match the
  datasheets: 85.0 mm for the 2F-85, and 159 mm vs 160 mm for the RG6.
- **The RG6 fingertips move 55 mm along the approach axis:** 228 mm below the gripper base fully open, 283 mm
  closed. The 2F-85's move about 14 mm. The TCP is the closed position, so with open fingers the tips sit
  higher than the TCP. Grasp planning has to allow for that.
- **Flange + tool changer are 61 mm** (10 + 51) for every gripper (`tool.changer_length_m`).
- **The RG6 is turned -29.1 deg on the flange** (`mount_yaw_rad`): its fingers open parallel to the camera's long side.
  The TCP frame is not turned, so it matches the pendant's TCP: to line the fingers up with a direction, turn the TCP
  29.1 deg from it.
- **The RG6 TCP is off-axis:** `tcp_xy_m` in `config/grippers/rg6.yaml` holds x -4.07 mm, y -5.19 mm from the TCP
  calibration (pendant TCP `hand_eye_tcp`). Recalibrating means updating it there.
- **The RG6 model is too long:** 283.2 mm from base to closed fingertips, the real gripper about 261 mm. Its
  `mount_offset_m: -0.0222` keeps the fingertips right (TCP 322 mm), so the model's body sits 22.2 mm too close to
  the flange. Which part of the model is too long is unknown.
- The RG6 model's joint range was widened from ±0.628 rad, where it opened only 151 mm, to ±0.90 rad.
- A new gripper needs four things: a xacro macro `gripper_<model>` in `ros/urdf/grippers/`, a
  `config/grippers/<model>.yaml`, an include line in `cell_tool.xacro`, and its table from
  `docker compose exec ros python3 /app/tools/gripper_table.py <model>`.

Before the first run on the **real arm**:
1. **Remote Control:** in PolyScope, Settings > System > Remote Control, then switch to Remote (top right).
   The driver runs headless (it sends its own control script, so no External Control program is needed), and
   the Dashboard calls for the gripper programs need Remote mode too.
2. **Gripper programs:** `gripper_close_adaptive.urp` (`gripper.open_program` and `close_program`: one script for both) must exist on the pendant (they did on
   2026-09-16). Each has one RG6 node that takes its width (mm) from `read_input_float_register(18)` and its
   force (N) from `read_input_float_register(19)` in a Script node, as the sota UI requires. For the API to
   know the real finger width, add `write_output_float_register(N, <the URCap's width variable>)` at the end of
   the program and set `gripper.width_feedback_register: N`. Without that, the API reports the commanded
   width with `width_measured: false`, and a grasp counts as successful once the program finishes.
   **Robotiq 2F-85 instead:** start with `UR_GRIPPER=robotiq_2f85` (or `gripper.model: robotiq_2f85`). Its block
   `gripper.robotiq_2f85` in `config/cell.yaml` replaces only the program names; the RG6 settings stay untouched.
   On the pendant (Robotiq URCap installed, gripper set up in the Installation tab) make `aikido.urp` with
   one Script node (check the exact `rq_*` names in the Script node's function list for your URCap version):
   ```
   write_output_float_register(19, -1)                                   # object detection: unknown until the move ends
   rq_activate_and_wait()
   rq_set_force_norm((read_input_float_register(19) - 20) / 215 * 100)   # N -> % (2F-85: 20-235 N)
   rq_set_speed_norm(50)
   rq_move_and_wait_mm(read_input_float_register(18))                   # mm; needs the URCap's mm calibration
   write_output_float_register(18, rq_current_pos_mm())
   if rq_is_object_detected():                                         # the fingers stopped on something (force), not at the target
     write_output_float_register(19, 1)
   else:
     write_output_float_register(19, 0)
   end
   ```
   With the two register-19 parts in the program, uncomment `object_feedback_register: 19` in the Robotiq block: every gripper
   reply then has `object_detected` true / false (null without it, and always for the RG6).
   The first activation sweeps the fingers fully. The TCP (0.235 m) is derived from the model, not measured:
   measure it and set it (pendant + `/robot/tcp`) before grasping; set the payload on the pendant too.
3. **Network:** the robot must reach this PC, because the driver uses reverse connections on ports
   50001–50004. Set `robot.ip` in `config/cell.yaml`, or pass `UR_ROBOT_IP`.
4. **Measure the `MEASURE` values in `config/cell.yaml`:** the tool changer, camera bracket, and **the table
   height**. These are the collision model; values that are too small let plans pass that touch the real cell.
   The tool length (0.322 m) and the camera pose (hand-eye calibration, 1.3 mm rms) were measured. The RG6
   itself is its 3D model.
5. **Set the payload on the pendant** (RG6 + changer + camera). The protective-stop sensitivity and the F/T
   readings depend on it.

## Access

**The API is currently OPEN: no token, and it listens on every network.** Anyone who can reach `http://<pc>:5000` can
move the arm, including from a browser on the same Wi-Fi. Set up that way on purpose for now (`docker-compose.yml`);
`/` and `/help` say so too. To connect, use `http://<pc>:5000`. The firewall script below would block this.

To lock it again, set `UR_API_HOST=127.0.0.1` and `UR_API_AUTH=on` in `docker-compose.yml` and restart. The layers are
then, from the outside in:

1. **Firewall:** `sudo tools/lockdown_firewall.sh` lets only SSH in over Wi-Fi and stops routing from Wi-Fi to the
   robot network. Needed because ROS 2's UDP ports listen on all interfaces. `sudo ufw disable` undoes it.
2. **Listen address:** the API listens on `127.0.0.1`. From another machine, use a tunnel, which also encrypts the
   traffic: `ssh -L 5000:localhost:5000 ur@<pc>`, then open `http://localhost:5000/ui`. `UR_API_HOST=0.0.0.0` opens
   it to every network, with the token and commands unencrypted.
3. **Token:** everything except `/`, `/ui`, `/login` and `/control/stop` needs
   `Authorization: Bearer $(./run.sh token)`. The browser panel asks for it once and keeps a strict same-site cookie.
   To replace the token, delete `state/api_token` and restart.
4. **ROS 2 traffic:** `config/cyclonedds.xml` keeps it on this PC, so other machines cannot reach the controllers.

Keep the robot on its own cable, never on the Wi-Fi, and keep the PolyScope passwords set. None of this replaces the
emergency stop.

## Grasp check

`gripper_grasp` answers 409 `object_missed` with `object_held: false` when the fingers closed on nothing. Add
`require_object=1` to the move towards the placement position: it then refuses with 409 `object_not_held` unless the
last grasp was confirmed. `/state` and the panel show `object_held`. It needs the real width (pendant step 2 above);
without that, `object_held` stays `null` and `require_object=1` refuses. The check runs when the grasp ends, so an
object dropped later in the move is not noticed.

## Camera resolution

`GET /camera/config` lists the modes. Colour and depth are set separately, and both use the same `fps`:
```
/camera/config?width=1280&height=720&fps=30                        # colour
/camera/config?depth_width=640&depth_height=400                    # depth sensor (depth=auto: the largest)
/camera/config?align=0                                             # depth images at their own resolution
```
With `align=1` (default) depth is resampled onto the colour image, so it comes out at the colour size. A mode the camera
rejects leaves the previous settings in place. Changes last until the server restarts; to keep them, set `camera.width`,
`height`, `fps`, `depth_width`, `depth_height` and `align_depth` in `config/cell.yaml`.

## Camera exposure

```
/camera/exposure                      # read: auto, exposure, gain and their ranges
/camera/exposure?exposure=200&gain=20 # by hand (turns auto off)
/camera/exposure?auto=1               # the camera decides
/camera/exposure?reset=1              # undo the changes: cell.yaml's values, or auto when it sets none
```
Colour stream only, in the camera's own units. The panel has the same controls, and "Reset to default" for reset=1. It lasts until the server restarts; to
keep it, set `camera.auto_exposure`, `exposure` and `gain` in `config/cell.yaml`.

## RViz view

`robot.room_yaw_deg` in `config/cell.yaml` turns the robot against RViz's grid to match how it stands in the room (+ =
counter-clockwise from above). Then `./run.sh rviz`.

The same angle gives the room's axes: `frame=room` on `plan_cartesian_path`, `plan_joint_path` and `rotate_tcp`, and the
panel's jog and rotate buttons (default). `/state` reports the TCP in them under `room`. Coordinates without `frame=room`
are along the robot's own axes, and saved poses are unaffected. Angle set wrong, `+y` goes the wrong way.

## Arm calibration

Every UR arm has its own factory calibration. `config/calibration/ur_calibration.yaml` holds this arm's, and the driver and
model use it (`robot.kinematics_params_file` in `config/cell.yaml`).
```
curl http://<pc>:5000/robot/calibration                 # values, hash, whether it is in use and matches the arm
curl http://<pc>:5000/robot/calibration?format=yaml     # the file itself
./run.sh save-calibration                               # read it from the arm again, then restart the stack
```
`tools/save_calibration.py` does the job of `ur_calibration`'s `calibration_correction`, because the released package (3.9.0)
never connects. Read it again if the arm or controller is replaced.

## Home pose

Put the arm where you want it, then press **Save current pose as home** in the panel (or `POST /robot/home`). It is
kept in `state/home.json` and refused if the arm is moving, near a joint limit or in collision. **Reset home**
(`DELETE /robot/home`) goes back to the `cell.yaml` pose.

## Differences from the Franka API

| | Franka API | this API |
|---|---|---|
| joints | 7, `panda_joint1..7` | 6, `shoulder_pan_joint` … `wrist_3_joint`; `joints=`, `j1..j6=` |
| frame | `panda_link0` | `base_link` (MoveIt's `world` is identical). The **pendant / RTDE `base` frame is `base_link` turned 180° about z** (x and y negated); motion endpoints take `frame=base`, `/state` also returns `ur_base` |
| TCP | `panda_hand_tcp` | `tcp`, the fingertip centre with the gripper closed: 0.322 m along `tool0` z with the RG6, 0.235 m with the 2F-85 |
| big-swing guard exempt joint | joint 7 | `wrist_3_joint` |
| gripper | Franka hand, width 0–0.08 m | RG6: 0–0.160 m, force **25–120 N** (values outside are clamped and the response says so), speed ignored. `gripper_homing` just opens it. 2F-85: model only so far (501 on the real arm) |
| finger joints | two, each half the width | the gripper's one driving joint (`rg6_finger_joint` / `robotiq_85_left_knuckle_joint`), an angle from the width table |
| gripper width | always measured | measured only with `width_feedback_register` (see above) |
| force | F_ext estimated from joint torques | the e-series **flange F/T sensor**; extra `/force/zero_sensor` zeroes the sensor itself |
| `stopped_external_force` | reflex (push / contact) | UR **protective stop** |
| `stopped_user_button` | user-stop button | UR **emergency or safeguard stop** |
| `/recover` | Franka error recovery | unlocks a protective stop (refused by the controller in its first 5 s), closes popups, resends the driver's program. It does **not** power on or release the brakes: a powered-off arm is reported, not restarted |
| `/control/home` | fixed Panda pose | `robot.home_joints_rad` (the recorded eye-in-hand home) |
| extras | | `velocity=` and `acceleration=` on every motion (see *Speed* below); `/joints` has `gripper_width_measured`; `/health` adds robot mode, safety mode, driver program, dashboard |
| camera | RealSense D435 | `camera.backend`: Orbbec Gemini 2 (this arm's camera), RealSense or none. Same endpoints, depth always uint16 mm aligned to colour |

## Speed: velocity and acceleration

Every motion endpoint takes `velocity=` and `acceleration=` (query, or the JSON body of `POST /control/goto_joints`): MoveIt's
scaling factors, 0.01–0.5 of the joint speed and acceleration limits (400 outside). Short moves are limited by the acceleration,
so raising `velocity` alone barely shortens them; raise both. Each reply echoes the two values used.

| endpoint | `velocity` default | `acceleration` default |
|---|---|---|
| `/control/goto_joints`, `/control/goto_pose` | 0.2 | 0.1 |
| `/control/plan_cartesian_path`, `/control/rotate_tcp` | 0.05 | the velocity's value |
| `/control/plan_joint_path` | 0.3 | 0.05 |
| `/control/home` | 0.3 | 0.1 |

`/control/resume` repeats the last motion with its values. `/control/nudge_joint` (recovery) stays at 0.15 / 0.05.

## Verified so far (2026-10-26)

- **Mock hardware:** `tests/api_smoke.py`, 68 of 68 checks: motion, guards, stop, resume,
  gripper (simulated), both scene APIs, all `/plan/*`, files, robot model, force.
- **URSim 5.17 (UR10e)** with the real driver: 73 of 73, the same checks plus a protective stop in mid-motion.
  - Accuracy: joint targets within 0.004°, a 5 cm straight line within 0.03 mm, a target in the pendant frame
    within 0.08 mm.
  - Stop mid-motion halted the arm.
  - Protective stop: answered `stopped_external_force` within 0.6–1 s; `/recover` then `/control/resume`
    finished the move.
  - The RG6 sequence ran on the controller (stand-in programs that read the registers), with width feedback.
- **Unit tests** (`python3 -m unittest discover -s tests`):
  - the scene hash equals the client formula on 200 random scenes;
  - the scene core's batch, rollback and dirty rules hold against a fake MoveIt;
  - IK round trip: 189 random poses, worst 0.1 µm.
- **2026-10-27, gripper models:** mock hardware, 68 of 68 with each gripper model. RG6 and 2F-85 fingertips
  pushed 1 cm into the table are reported as colliding (the meshes load), and the TCP is 0.320 / 0.211 m.
- **Not yet:** the real arm, the real RG6 programs, the Orbbec camera through this API, and URSim with the
  gripper models.

## Things learned the hard way (kept in the code comments too)

- **Stopping:** cancelling MoveIt 2's `ExecuteTrajectory` goal does **not** stop the arm. `/control/stop`
  publishes `stop` on `/trajectory_execution_event`, as `MoveGroupInterface::stop()` does.
- **Protective stops:** the scaled trajectory controller does not abort on a protective stop; it scales time
  to zero and waits, and the request would hang until the timeout (34 s on URSim). A watcher polls the
  Dashboard while the arm moves and ends the execution at once.
- **Status topics:** the driver's latched `safety_mode` / `robot_program_running` topics were seen stale (a
  protective stop shown while the controller said NORMAL; the program shown running after it stopped). Safety
  mode, robot mode and program state come from the **Dashboard**; the topics are only the fallback on mock
  hardware.
- **RG6 registers:** `ur_rtde`'s `RTDEIOInterface` cannot write the RG6 registers next to the ROS driver. Its
  recipe also claims `speed_slider_mask` / `standard_digital_output_mask`, which the driver holds, so the
  controller refuses the whole recipe. `rtde_regs.py` claims only the two registers.
- **Pendant programs:** a `.urp` whose XML does not parse makes PolyScope refuse to load it. On URSim the arm
  was in POWER_OFF 0.3 s after such a failed load (seen once, cause not proven); the API then reports
  `control_restored: false` and `/recover` explains it. URScript has no `max()`. Escape `<` in a `.urp`.
- **The Franka README's hash snippet** builds `dims` eagerly (`{"box": s["size"], "sphere": [s["radius"]], ...}`),
  which raises `KeyError` for every shape. `tests/test_scene_hash.py` uses the same values evaluated lazily.

## TCP

The `tcp` link (the fingertip centre the API plans to and reports in `/state`) is built at start from the saved TCP in
`config/calibration/tcp_<gripper model>.yaml` (`robot.tcp_file`, one per gripper). The controller's TCP is not used for motion: the API sends joint
trajectories planned to the `tcp` link.

```bash
curl localhost:5000/robot/tcp                         # in use, saved, restart_required
curl "localhost:5000/robot/tcp?controller=1"          # + the controller's active TCP (read-only)
curl -X PUT -H 'Content-Type: application/json' localhost:5000/robot/tcp \
     -d '{"xyz_m":[0,0,0.322],"rotvec_rad":[0,0,0],"note":"pendant hand_eye_TCP"}'   # metres; rpy_rad also accepted
curl -X POST localhost:5000/robot/tcp/from_controller # save the controller's active TCP instead
./run.sh restart                                      # the new TCP takes effect (restart_required -> false)
```

- `robot.tcp_mode: file` (default): the saved TCP wins. At start the controller is read only to compare (a NOTE in
  `./run.sh logs` when it differs), or to create the file if there is none. `controller`: re-read at every start.
- `follow` (the Robotiq's block, `gripper.robotiq_2f85.tcp_mode`, since 2026-11-20): like `controller`, and while
  running the API reads the controller's active TCP every 5 s (read-only). When it changes (e.g. TCP_1 edited on the
  pendant) and is the same on two reads, plausible, and the arm has been idle for 15 s, the API saves it and restarts
  itself and the ROS side (`state/restart_ros` → `ros/scripts/cell.sh`): about 30–60 s, the arm does not move. At most
  one restart per 2 min. A gripper's own `tcp_mode` overrides `robot.tcp_mode`.
- **Don't use `controller` or `follow` mode with the RG6.** Its URCap switches the active TCP to its own (234 mm, turned 180°)
  whenever an RG6 node runs, so the pendant's Installation screen and the active TCP disagree.
- PUT refuses z outside 0.02–0.8 m (so millimetres are caught) and x, y more than 0.2 m off the flange axis. It warns
  when z is more than 5 mm off the gripper model's fingertips, or when the TCP is rotated.

## Wrist camera pose (hand-eye)

The `wrist_camera` link (and the bracket box) is part of the robot description: RViz shows it there, and the
planning scene checks it against the bin and the tables. Its pose on the flange comes from the client's hand-eye
calibration, not from hand-typed numbers:

```bash
curl localhost:5000/robot/camera_mount                # in use (the loaded link), saved, restart_required
curl -X PUT -H 'Content-Type: application/json' localhost:5000/robot/camera_mount \
     -d '{"T_tool0_camera": [[...4x4, tool0 -> camera optical frame...]], "apply": true}'
```

It is saved to `config/calibration/camera_mount.yaml` (`tool.camera_mount_file`). `cell.launch.py` hands it to the
URDF at every start through env `UR_CAMERA_MOUNT`. With `"apply": true`, the API and the ROS side restart at once
(202, about 30-60 s; refused while anything runs; the arm does not move). If the mount is unchanged (within 1 mm and
0.2 deg), nothing restarts. With the file present, the bracket becomes a bar from the tool changer out to the camera
body (width and thickness from `bracket_size_m`). Without the file, `tool.camera_xyz_m` / `camera_rpy_rad` /
`bracket_*` of `config/cell.yaml` are used, as before. pipeFit sends its accepted calibration automatically
(`ur_calib.send_camera_mount`).

## Tables (pick and place heights)

The pick table (bin of pipes, next to the robot) and the place table (the one the robot stands on) are collision
boxes whose height comes from the client's calibration, so either can change height without a restart. Footprints
are in `config/cell.yaml`, section `surfaces`.

1. `POST /cell/surfaces/pick/calibrate`: the pick table's box leaves the collision scene, so the calibration poses
   down at the new height are not blocked. **Until step 2 nothing stops the arm at that table.**
2. Measure the table top with the camera depth, then `PUT /cell/surfaces/pick` with
   `{"points": [[x, y, z], ...], "frame_id": "base_link"}` (or `"point": [x, y, z]`; `"frame_id": "base"` for the
   pendant's frame). The box comes back with its top at the median z, centred on the mean x, y. Heights outside
   `z_range_m` are refused; points more than 10 mm apart in height, or the arm touching the new box, give warnings.
3. `POST /cell/surfaces/pick/cancel` instead puts the previous box back.

The same for `place`; the robot's base may touch the place box (`allowed_links`). `GET /cell/surfaces` shows both.
The boxes live in the scene namespace `cell.surfaces`, persist across restarts and show in RViz. The bin is not a
surface: send it with `/scene/batch` in the client's own namespace once its position is calibrated.

## Safety

- **Access:** currently open, with no token; see [Access](#access).
- **Collision model:** the real cell has NO fixed geometry since 2026-11-11 (`world_boxes: []` in
  `config/cell.yaml`): the floor box under the base blocked picking from a table lower than the robot's own. The
  tables are the calibrated surfaces (see [Tables](#tables-pick-and-place-heights)); until both are set, the arm can
  plan into them. URSim
  (`tests/ursim/cell_ursim.yaml`) keeps its floor. Tool sizes marked `MEASURE` are estimates. The gripper's own links are excluded
  from checks against each other and against the changer, camera and wrist.
- **Motion guards:** every motion endpoint keeps the Franka rules:
  - targets and paths stay away from joint limits;
  - the big-swing guard applies;
  - collision checks run against the planning scene;
  - one motion at a time (409 when busy);
  - `/control/stop` always passes.
- **Speed:** keep the pendant speed slider low for first runs. The scaled controller follows it.
