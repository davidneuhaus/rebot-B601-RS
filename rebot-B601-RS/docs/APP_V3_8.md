# App 3.8: session recording with live markers, shared cameras, vision tools

Installed on the laptop (<laptop>) on 2026-11-21. Builds on 3.7 (guard v3.3). The GUI labels are German.
Files: `teleop_gui.py`, `gui_v38.py` (new), `recorder.py`, `camera_feeds.py`, `tools/export_dataset.py` (new).
Snapshots: `rebot-B601-RS/versions/before_v3_8` and `versions/v3_8` (`./rollback.sh before_v3_8`).

## 1. Session recording ("⏺ Session aufnehmen")
- Turn it on in the "Datensatz aufnehmen" box. The recorder then records **takes** back to back while teleop runs.
- End each take with a marker. The next take starts immediately:

  | Button | Key | Effect |
  |---|---|---|
  | ✓ geklappt | S | save the take, label `success` |
  | ✗ fehlgeschlagen | F | save the take, label `fail` |
  | ↻ Neuer Versuch | N | save the take, label `play` (default label) |
  | 🗑 Verwerfen | D | drop the take |

  The keys work while no text field has the focus. Click the camera area or a button first.
- "■ Session beenden" saves the running take as `play`, if it is at least 1 s long. Takes under 1 s are never saved.
- The **task text** is the "Aufgabe" field. Its value at the moment of the marker is stored with the episode.
  If the field is empty, the task is `play`.
- Saving is asynchronous. A background thread in `recorder.py` writes each finished take: open/resume dataset,
  add frames, `save_episode`, `finalize`. So the dataset is valid on disk after every take, and recording never
  waits. The status line shows `⏳ speichert N` while saves are queued.
  Measured on the M1: about 2 s per short take. The first save of a session takes about 7 s (library import).
- Labels: `<dataset>/meta/episode_labels.jsonl`, one line per episode:
  `{"episode_index", "label", "task", "frames", "seconds", "wall_s", "recorded_at", "take"}`.
  Classic "Episode starten/speichern" episodes get label `play`.
- Take memory: frames are kept as JPEG (q 92) in RAM, about 3 MB/s. After 300 s a take is split automatically
  (saved as `play`). Env `REC_TAKE_MAX_S`, `REC_JPEG_Q`.
- If teleop stops (no joint data) or a camera drops out, the take **pauses** and keeps its frames. It continues when
  data is back. ■ STOPP in the app pauses the take too, so the rest move is not recorded.
- The app does not reload itself while a session runs or saves are still queued.
- Video encoding now runs in-process (`REC_PARALLEL_ENCODING=0`). LeRobot's parallel encoding started fresh
  processes that re-imported torch, so a 2 s episode took 13–17 s to save. Now it takes about 2 s.

### Export a filtered dataset
```
~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1 --list
~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1               # -> pick_place_v1_success
~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1 --labels success,play --out pp_good
```
It uses LeRobot's `delete_episodes`, so the source is untouched. Kept episodes are renumbered, and the label file
is copied with `source_episode`. Options: `--dry-run`, `--force`, `--root`.

## 2. Cameras open only once (shared frames)
- The GUI owns the cameras. Each camera thread scales the newest frame once to 640×360 (aspect kept) and writes
  it into POSIX shared memory `humanoid_cam_<role>` (seqlock header). `recorder.py` and `play_policy.py` read it through
  `camera_feeds.ShmFeed`. The GUI sets `HUMANOID_CAM_SHM=1` for its children.
- Started without the GUI (Terminal), they open the cameras directly, as before.
- The preview also uses the small frame, which is cheaper. Full-resolution frames stay in the GUI for the enlarge window,
  calibration and detection.

## 3. Vision tools ("Kamera-Werkzeuge (Übersichtskamera)")
- **📐 Neu kalibrieren**: `vision/calibrate.recalibrate_table(frame)` with the newest full-resolution frame of the
  overview camera (role `overview`), so no second camera open. Shows the message, `shift_mm` (mean/max) and warnings.
  Lay the board flat in its usual spot first. The call writes `vision/calibration/table.json` and a history entry.
- **🔍 Erkennung** + prompt field (comma-separated, e.g. `blue ball, trash bin`): `vision/detector.Detector`
  (YOLO-World, MPS) in a worker thread, overview feed only, at most 8 fps (`DETECT_MAX_FPS`). Boxes are drawn on the
  preview and on the ⤢ enlarge window. Turning it off frees the model. Uses about 1.2 GB RAM.
- **🤝 Arm mit Tisch verbinden** (step c): only while teleop runs and no recording or session is active.
  A dialog asks for corners A–D: guide the tip with the leader, hold still, press "Weiter"/Enter.
  The joint angles are the median while the arm is still (velocity < 3°/s). They come from the teleop telemetry,
  which `recorder.py` forwards to 127.0.0.1:8766 (`TELEMETRY_FWD_PORT`), so there is no port conflict with the
  recorder (it keeps 8765). Then `calibrate.fit_robot_from_joints` runs. Result: RMS residual and tip-height spread
  (< 5 mm good, > 10 mm warning plus sign alternatives). Read-only: no motor commands.
  The Terminal command `calibrate.py robot-capture` listens on 8765 itself, so it only works while the app is closed.

## 4. Other
- The window fits the screen (height = screen − 70 px) and the whole content scrolls (mouse wheel / scrollbar).
- Recorder log (`logs/recorder.log`) shows the loop rate every 10 s while recording (`loop: 28.6 Hz, tick avg …`).
- Installed `lerobot[dataset]` (datasets 5.2.0, pyarrow 26, pandas 2.3.3, av 15.1.0, torchcodec 0.17.0) with
  `numpy==2.2.6` and `opencv-python-headless==4.13.0.92` pinned (`~/humanoid/tools/pins.txt`). Before this,
  **recording could not work on the laptop** (`datasets` was missing). TorchCodec cannot find FFmpeg and LeRobot falls
  back to PyAV for decoding (warning only). cv2 and av both ship libavdevice (objc duplicate-class warning,
  harmless so far).

## Tests (laptop, no robot, no cameras)
- `tests/test_session_v38.py`: end-to-end recorder session with synthetic shared-memory frames and fake UDP telemetry:
  async save, labels, pause/resume, discard, too-short take, classic mode, export, dataset loads. 12 s take: 343 frames
  (28.6 fps).
- `tests/test_gui_v38.py`: GUI with `TELEOP_GUI_TEST=1`: buttons, S/F/N/D shortcuts (not while typing), reload block,
  vision buttons, overlay drawing, robot-link dialog with fake telemetry on the forward port (calibration redirected
  to a temp dir).
- `tests/test_detect_v38.py`: real YOLO-World on MPS via the overlay worker: labels found, 7.8 fps (limit 8).
