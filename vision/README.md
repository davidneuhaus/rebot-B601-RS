# Vision: open-vocabulary detection (YOLO-World)

Standalone module, no dependency on the GUI, recorder or robot code.

## API (`vision/detector.py`)
```python
import sys; sys.path.insert(0, "~/humanoid/vision")
from detector import Detector
det = Detector(model="yolov8s-worldv2.pt", device=None,   # None = MPS if available, else CPU
               imgsz=640, conf=0.15, min_interval=0.0)    # min_interval>0 throttles: returns last result
det.set_classes(["blue ball", "Nivea tin", "trash bin"])  # free text; re-callable at runtime
dets = det.detect(bgr_frame)   # [(label, conf, x1, y1, x2, y2), ...] original pixel coords, sorted by conf
out = det.draw(bgr_frame, dets)  # annotated copy
```
Frames are BGR numpy arrays. `detect` resizes internally to `imgsz`. Call `set_classes` only when prompts
change. Not thread-safe: use one Detector per thread. Weights live in `vision/models/`
(override with `HUMANOID_VISION_MODELS`).

## CLI
```
~/humanoid/.venv/bin/python ~/humanoid/vision/detect_live.py --source 1 --prompts 'blue ball,Nivea tin,trash bin'
~/humanoid/.venv/bin/python ~/humanoid/vision/detect_live.py --source C920 --prompts 'trash bin'   # by name (needs ffmpeg)
~/humanoid/.venv/bin/python ~/humanoid/vision/detect_live.py --image in.jpg --prompts 'person,bus' --out out.jpg
```
Options: `--device mps|cpu`, `--imgsz 640`, `--conf 0.15`, `--max-fps 10`, `--no-window`, `--frames N`.
The env has opencv-python-headless (no imshow). Live mode then prints fps and writes `/tmp/detect_last.jpg`.
A real window needs a GUI-enabled OpenCV, which is not installed to avoid breaking LeRobot.

## Install (done on SH-0000-0052)
`pip install ultralytics` and `pip install git+https://github.com/ultralytics/CLIP.git` in the lerobot env,
then restore `numpy==2.2.6` and `opencv-python-headless==4.13.0.92` (ultralytics pulls numpy 2.5 and
opencv-python, which conflict with lerobot). `pip check` shows only ultralytics metadata warnings.

## Measured (810x1080 sample, imgsz 640, 4 prompts)
MPS ~34 ms/frame (~30 fps), CPU ~224 ms (~4.5 fps). Peak RSS ~1.2 GB (MPS), ~1.4 GB (CPU).
Next to teleop, throttle to 5-10 fps on MPS.

## Calibration (`vision/calibrate.py`)
Pixel -> table (mm) -> robot (mm) for the overview camera. Data in `vision/calibration/`
(`intrinsics.json`, `table.json`, `board.json` optional, `history/cal_*.json`, last 10 kept).
Default board: 9x6 inner corners, 25 mm squares (override in `calibration/board.json`).

1. Intrinsics, once (only redo if lens or resolution changes): `calibrate.py intrinsics --source 1`
   (hold the board at ~15 angles).
2. Quick recalibration, < 1 s compute (do this every session or after the camera/table moved):
   lay the board flat in its usual spot, run `calibrate.py table --source 1`
   or in Python `recalibrate_table(source)` -> `{"ok","message","shift_mm":{"mean","max"},"warnings","path"}`.
   It keeps intrinsics, saves a timestamped history entry, and reports the shift vs the previous calibration in mm.
3. Table -> robot (step c), once: `calibrate.py robot --pairs '[[tx,ty,rx,ry],...]'` (>= 3 points, mm; touch
   board corners with the gripper by hand). The quick path reuses it if the board is on its mark; a shift above
   20 mm raises a warning.
4. Use: `pixel_to_table(px)`, `table_to_robot(xy)`. `load_table_with_warnings(verify_source)` warns if
   cameras.json changed after the calibration, intrinsics are missing or resolution changed, the board can't be
   seen (cannot confirm), or the board is >3 px off the saved pose. `calibrate.py check --source 1` does this.
Tested on synthetic board renders (corner detection 10/10, an 8 mm shift reported as 7.9 mm). Not yet tested
with a real board or camera.

## Step (c): table -> robot (FK + capture, no CAN access)
**FK source:** `vision/robot_fk.py` parses the Seeed URDF `lerobot_robot_seeed_b601/assets/00-arm-rs_asm-v3.urdf`
(plugin 1.0.5, the same file the plugin's gravity model uses). Input: the 6 arm joints in degrees as the follower
reports them (`<joint>.pos`, raw motor angle, mapped 1:1 to URDF joint1..6, as the plugin's gravity code does).
Output: tip position in the robot base frame (mm). `HUMANOID_TIP_OFFSET_MM` / `--tip-offset-mm` shifts the tip
along the gripper_end x axis (finger direction); default 0 = gripper_end origin. Measure the real offset if the
residual is large. `python robot_fk.py` prints a sanity report.
Sanity check: at the zero pose gripper_end is at (302, 0, 218) mm, tool z axis pointing up; lift=90 elbow=90 gives
(538, 0, 454). Max reach by link lengths 959 mm, consistent with the Seeed spec (754.7 mm with gripper).
Not verified on hardware: the sign convention of pan, wrist_flex, wrist_yaw (and tip offset), so the fit tries
all 8 sign combos and keeps (+,+,+) unless another is clearly better. Seeed's wiki lists J2/J3 as 0..-220 for a
different library, while this plugin/URDF uses positive 0..170/0..200, so check the first real result.

**Why no direct CAN read:** RobStride motors have no state query (motorbridge `request_feedback` is a no-op for
them); state only arrives in reply to commands. A truly passive read is therefore impossible. Instead the capture
tool reads **teleop's UDP telemetry tap** (127.0.0.1:8765, `lerobot_teleoperate_tap.py`): no CAN access, no commands.

**Steps for David (arm update 3.7 installed, board lying where table calibration was done):**
1. Start teleop in the app (Start). Do not start Record (it also listens on port 8765).
2. In Terminal: `~/humanoid/.venv/bin/python ~/humanoid/vision/calibrate.py robot-capture`
3. For corners A, B, C, D (printed with their table mm; A = origin, B along the long side, D along the short side,
   all inner corners): guide the gripper tip with the leader onto the corner, hold still ~1 s, press Enter.
   The tool takes the median of the joint angles while the arm is still (velocity < 3 deg/s).
4. It prints the fit: RMS residual in mm, tip-height spread (the board is flat, so all tips should have the same z),
   per-point errors. Good: < 5 mm. Above 10 mm it warns and lists alternative sign conventions.
5. Done: stored in `calibration/table.json` and history. `table_to_robot(xy)` / `table_to_robot_xyz(xy)` give
   robot coordinates (z = measured table height). Quick recalibration of the table (step b) reuses it while the
   board stays on its mark.
Fallbacks: `--manual` (type the 6 joint degrees), or the old `robot --pairs` with known pairs.
Tested only with synthetic joint data (IK-generated poses, known mirrored transform): residual 0.2 mm, z spread 0.1 mm,
a flipped wrist_yaw sign was detected; the UDP reader was tested with a fake sender. Not tested on the real arm.

## Auto calibration (3.9): hand-eye with the ChArUco board
`handeye.sh teach | plan | run | replay | apply | status | run --sim` (wrapper for `vision/handeye.py`), GUI box
"🤖 Auto-Kalibrierung". Board: `calibration/board_charuco.json` (7×5 squares, 35 mm, DICT_5X5_100), helpers in
`charuco.py`. Planner + safety: `handeye_plan.py`; solver: `handeye_solve.py`; synthetic truth: `handeye_synth.py`.
Results: `calibration/handeye.json` and `table.json` → `table_to_robot.T_board2base`, `T_overview2base`.
`calibrate.table_to_robot_xyz(xy)` uses the 3D transform when present. Tests: `tests/test_handeye.py`.
Details: `~/humanoid/rebot-B601-RS/docs/APP_V3_9.md`, runbook: `~/humanoid/docs/OPERATING_PROCEDURE.md` section 8.
