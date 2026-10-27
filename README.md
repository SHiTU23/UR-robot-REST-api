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
curl http://<pc>:5000/help                                   # every endpoint, generated from the code
curl http://<pc>:5000/state                                  # TCP pose, gripper, joints
curl "http://<pc>:5000/control/goto_joints?joints=0,-100,100,-90,-90,0&unit=deg&dry_run=1"
curl "http://<pc>:5000/control/plan_cartesian_path?x=0.6&y=0.2&z=0.3&roll=180&pitch=0&yaw=0"   # MOVES THE ARM
curl "http://<pc>:5000/control/gripper_grasp?width=0.013&force=40&eps_out=0.02"
curl http://<pc>:5000/control/stop
open http://<pc>:5000/ui                                     # browser panel: state, jog, gripper, force, camera, STOP
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
UR_ROBOT_IP=<ip> ./run.sh up  # the real arm
./run.sh test [url] [ursim ip]   # end-to-end checks (MOVES the robot it talks to)
./run.sh view rg6             # mock robot with that gripper + RViz (also: view robotiq_2f85)
./run.sh rviz                 # RViz on whatever stack runs
./run.sh logs | down
```

## Grippers

One line picks the gripper: `gripper.model` in `config/cell.yaml`, or `UR_GRIPPER=<model>` in front of any
`run.sh` command. The robot model (URDF), MoveIt's collision exclusions (SRDF), the finger display, the TCP and
the API's limits all follow it. The tool changer, camera and bracket stay the same.

| model | from | on the real arm | width | force | TCP (flange → closed fingertips) |
|---|---|---|---|---|---|
| `rg6` | MIT meshes from [BrettRD/onrobot_grippers](https://github.com/BrettRD/onrobot_grippers) | pendant programs (below) | 0–160 mm | 25–120 N | 0.320 m, **measured** |
| `robotiq_2f85` | official `ros-jazzy-robotiq-description`, on Robotiq's UR adapter | **not yet**: gripper endpoints answer 501 (mock: simulated) | 0–85 mm | 20–235 N | 0.211 m, **derived** (same changer + adapter + model) |

- Both grippers close their fingers on a linkage, so the finger joint is an angle, not half the width.
  `config/grippers/<model>.yaml` holds the table between joint angle, width and fingertip depth. It is
  computed from the model's collision meshes by `tools/gripper_table.py`. The model's widths match the
  datasheets: 85.0 mm for the 2F-85, and 159 mm vs 160 mm for the RG6.
- **The RG6 fingertips move 55 mm along the approach axis:** 228 mm below the gripper base fully open, 283 mm
  closed. The 2F-85's move about 14 mm. The TCP is the closed position, so with open fingers the tips sit
  higher than the TCP. Grasp planning has to allow for that.
- **The RG6 mount is anchored to the measured TCP.** The model is 283 mm from base to closed tips, while the
  sota note gives 1 cm plate + 5 cm changer + 26 cm gripper. The changer cylinder therefore gets the
  remaining 36.8 mm (`tool.changer_length_m`, `MEASURE`). Measure from the flange to the RG6 housing.
- The RG6 model's joint range was widened from ±0.628 rad, where it opened only 151 mm, to ±0.90 rad.
- A new gripper needs four things: a xacro macro `gripper_<model>` in `ros/urdf/grippers/`, a
  `config/grippers/<model>.yaml`, an include line in `cell_tool.xacro`, and its table from
  `docker compose exec ros python3 /app/tools/gripper_table.py <model>`.

Before the first run on the **real arm**:
1. **Remote Control:** in PolyScope, Settings > System > Remote Control, then switch to Remote (top right).
   The driver runs headless (it sends its own control script, so no External Control program is needed), and
   the Dashboard calls for the gripper programs need Remote mode too.
2. **Gripper programs:** `gripper_open.urp` and `gripper_close.urp` must exist on the pendant (they did on
   2026-09-16). Each has one RG6 node that takes its width (mm) from `read_input_float_register(18)` and its
   force (N) from `read_input_float_register(19)` in a Script node, as the sota UI requires. For the API to
   know the real finger width, add `write_output_float_register(N, <the URCap's width variable>)` at the end of
   both programs and set `gripper.width_feedback_register: N`. Without that, the API reports the commanded
   width with `width_measured: false`, and a grasp counts as successful once the program finishes.
3. **Network:** the robot must reach this PC, because the driver uses reverse connections on ports
   50001–50004. Set `robot.ip` in `config/cell.yaml`, or pass `UR_ROBOT_IP`.
4. **Measure the `MEASURE` values in `config/cell.yaml`:** the tool changer, camera bracket, and **the table
   height**. These are the collision model; values that are too small let plans pass that touch the real cell.
   The tool length (0.320 m) and the camera pose (hand-eye calibration, 1.3 mm rms) were measured. The RG6
   itself is its 3D model.
5. **Set the payload on the pendant** (RG6 + changer + camera). The protective-stop sensitivity and the F/T
   readings depend on it.

## Differences from the Franka API

| | Franka API | this API |
|---|---|---|
| joints | 7, `panda_joint1..7` | 6, `shoulder_pan_joint` … `wrist_3_joint`; `joints=`, `j1..j6=` |
| frame | `panda_link0` | `base_link` (MoveIt's `world` is identical). The **pendant / RTDE `base` frame is `base_link` turned 180° about z** (x and y negated); motion endpoints take `frame=base`, `/state` also returns `ur_base` |
| TCP | `panda_hand_tcp` | `tcp`, the fingertip centre with the gripper closed: 0.320 m along `tool0` z with the RG6, 0.211 m with the 2F-85 |
| big-swing guard exempt joint | joint 7 | `wrist_3_joint` |
| gripper | Franka hand, width 0–0.08 m | RG6: 0–0.160 m, force **25–120 N** (values outside are clamped and the response says so), speed ignored. `gripper_homing` just opens it. 2F-85: model only so far (501 on the real arm) |
| finger joints | two, each half the width | the gripper's one driving joint (`rg6_finger_joint` / `robotiq_85_left_knuckle_joint`), an angle from the width table |
| gripper width | always measured | measured only with `width_feedback_register` (see above) |
| force | F_ext estimated from joint torques | the e-series **flange F/T sensor**; extra `/force/zero_sensor` zeroes the sensor itself |
| `stopped_external_force` | reflex (push / contact) | UR **protective stop** |
| `stopped_user_button` | user-stop button | UR **emergency or safeguard stop** |
| `/recover` | Franka error recovery | unlocks a protective stop (refused by the controller in its first 5 s), closes popups, resends the driver's program. It does **not** power on or release the brakes: a powered-off arm is reported, not restarted |
| `/control/home` | fixed Panda pose | `robot.home_joints_rad` (the recorded eye-in-hand home) |
| extras | | `velocity=` on `plan_cartesian_path`; `/joints` has `gripper_width_measured`; `/health` adds robot mode, safety mode, driver program, dashboard |
| camera | RealSense D435 | `camera.backend`: Orbbec Gemini 2 (this arm's camera), RealSense or none. Same endpoints, depth always uint16 mm aligned to colour |

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

## Safety

- **No access control:** there is no token, the same as the Franka API (lab network only). Anyone who can
  reach port 5000 can move the arm.
- **Collision model:** the fixed scene is one table box (`world_boxes` in `config/cell.yaml`); add walls and a
  ceiling there for this cell. Tool sizes marked `MEASURE` are estimates. The gripper's own links are excluded
  from checks against each other and against the changer, camera and wrist.
- **Motion guards:** every motion endpoint keeps the Franka rules:
  - targets and paths stay away from joint limits;
  - the big-swing guard applies;
  - collision checks run against the planning scene;
  - one motion at a time (409 when busy);
  - `/control/stop` always passes.
- **Speed:** keep the pendant speed slider low for first runs. The scaled controller follows it.
