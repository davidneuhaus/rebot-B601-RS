#!/usr/bin/env python3
"""Dataset recorder for the teleop GUI (v3.3) – LeRobot 0.7 dataset format, local only (no Hub push).

Runs as its own process (started by teleop_gui.py), so recording can never slow the teleop loop:
- joint data: UDP telemetry from lerobot_teleoperate_tap.py (follower observation + commanded leader action,
  sent once per teleop cycle to 127.0.0.1:$TELEMETRY_PORT, default 8765);
- images: the two cameras from cameras.json, mapped by "role" (overview / gripper; v3.7, display order free),
  read by camera_feeds.Feed threads that only keep the newest JPEG;
- a 30 Hz loop samples the newest joint data + newest frames and calls LeRobotDataset.add_frame().
After every saved episode the dataset is finalized (valid on disk even if the app is closed or crashes);
the next episode re-opens it with LeRobotDataset.resume().

Dataset (repo_id "local/<name>", root ~/humanoid/datasets/<name>):
  observation.state      float32[7]  follower joint angles, deg (motor frame) – <motor>.pos
  observation.velocity   float32[7]  follower joint velocities, deg/s
  observation.effort     float32[7]  follower motor torques, Nm
  action                 float32[7]  commanded action (leader, as passed to send_action) – <motor>.pos
  observation.images.overview / .gripper   video, REC_HEIGHT x REC_WIDTH (default 360 x 640)

Control (stdin, one command per line):  start {"dataset": "...", "task": "..."} | save | discard | pause | quit
Session mode (v3.8, continuous recording with live markers):
  session_start {"dataset": "...", "task": "..."}  -> records "takes" back to back
  mark success|fail|play|discard {"task": "..."}  -> ends the current take with that label (discard = drop it) and
                                                     starts the next take at once; the finished take is saved by a
                                                     background saver thread (recording never waits for encoding)
  session_stop {"task": "..."}                     -> ends the current take as "play" (if >= 1 s), stops the session
  Labels go to <dataset>/meta/episode_labels.jsonl (episode_index, label, task, frames, seconds, recorded_at).
  Takes are held in RAM as JPEG (q 92), max TAKE_MAX_S (300 s) per take, then auto-split as "play".
Camera frames come from the GUI's shared memory (HUMANOID_CAM_SHM=1, camera_feeds.ShmFeed) when the GUI runs.
Telemetry datagrams are forwarded to 127.0.0.1:$TELEMETRY_FWD_PORT (8766) so the robot-link capture can listen.
Status (stdout, JSON per line): {"type": "status", "state": idle|recording|paused|saving, ...}
Env: REC_WIDTH/REC_HEIGHT (640/360), REC_FPS (30), TELEMETRY_PORT (8765), REC_ROOT (~/humanoid/datasets).
"""
import io, json, math, os, queue, shutil, socket, sys, threading, time
import collections

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
from PIL import Image
import camera_feeds

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
POS = [f"{m}.pos" for m in MOTORS]
VEL = [f"{m}.vel" for m in MOTORS]
TRQ = [f"{m}.torque" for m in MOTORS]
CAM_KEYS = ["observation.images.overview", "observation.images.gripper"]
W = int(os.environ.get("REC_WIDTH", "640")); H = int(os.environ.get("REC_HEIGHT", "360"))
FPS = int(os.environ.get("REC_FPS", "30"))
PORT = int(os.environ.get("TELEMETRY_PORT", "8765"))
ROOT = os.path.expanduser(os.environ.get("REC_ROOT", "~/humanoid/datasets"))
JOINT_STALE_S = 0.25      # joint sample older than this -> no frame
CAM_STALE_S = 0.5         # camera frame older than this -> reuse last image (counted)
CAM_LOST_S = 2.0          # camera gone this long while recording -> pause
ROBOT_TYPE = "seeed_b601_rs_follower"
FWD_PORT = int(os.environ.get("TELEMETRY_FWD_PORT", "8766"))   # v3.8: 0 = no forwarding
TAKE_MAX_S = float(os.environ.get("REC_TAKE_MAX_S", "300"))
JPEG_Q = int(os.environ.get("REC_JPEG_Q", "92"))
LABELS = ("success", "fail", "play")
LABELS_FILE = "episode_labels.jsonl"
# v3.8: LeRobot's parallel encoding spawns fresh processes that re-import lerobot/torch (~5-8 s each on the M1)
# -> a 2 s episode took 13-17 s to save; in-process encoding takes ~5 s. REC_PARALLEL_ENCODING=1 restores it.
PAR_ENC = os.environ.get("REC_PARALLEL_ENCODING", "0") == "1"


def append_label(name, rec):
    """v3.8: one JSON line per saved episode in <dataset>/meta/episode_labels.jsonl."""
    path = os.path.join(ROOT, name, "meta", LABELS_FILE)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def read_labels(root_dir):
    out = {}
    try:
        with open(os.path.join(root_dir, "meta", LABELS_FILE)) as f:
            for line in f:
                try:
                    r = json.loads(line); out[int(r["episode_index"])] = r
                except Exception:
                    pass
    except FileNotFoundError:
        pass
    return out

out_lock = threading.Lock()
def emit(**kw):
    with out_lock:
        try:
            sys.stdout.write(json.dumps(kw, ensure_ascii=False) + "\n"); sys.stdout.flush()
        except (BrokenPipeError, ValueError, OSError):
            pass   # GUI gone (app reload/close) – keep going, finalize still runs


def elog(msg):
    """Event log -> stderr (= ~/humanoid/logs/recorder.log when started by the GUI)."""
    try:
        print(time.strftime("%H:%M:%S ") + msg, file=sys.stderr, flush=True)
    except Exception:
        pass


def features():
    vec = lambda names: {"dtype": "float32", "shape": (len(names),), "names": names}
    f = {"observation.state": vec(POS), "observation.velocity": vec(VEL), "observation.effort": vec(TRQ),
         "action": vec(POS)}
    for k in CAM_KEYS:
        f[k] = {"dtype": "video", "shape": (H, W, 3), "names": ["height", "width", "channels"]}
    return f


# ---------------- inputs ----------------
class Telemetry(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        for i in range(20):              # an old recorder (app reload) may still be finalizing
            try:
                self.sock.bind(("127.0.0.1", PORT)); break
            except OSError:
                if i == 19: raise
                time.sleep(0.5)
        self.sock.settimeout(0.5)
        self.latest, self.ts, self.count = None, 0.0, 0
        self.fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if FWD_PORT else None

    def run(self):
        while True:
            try:
                d, _ = self.sock.recvfrom(65536)
                if self.fwd is not None:
                    try: self.fwd.sendto(d, ("127.0.0.1", FWD_PORT))   # v3.8: for calibrate robot-capture
                    except OSError: pass
                m = json.loads(d)
                if m.get("obs") and m.get("act"):
                    self.latest, self.ts = m, time.time(); self.count += 1
            except socket.timeout:
                pass
            except Exception:
                pass

    def fresh(self):
        return self.latest if self.latest and time.time() - self.ts < JOINT_STALE_S else None


class Cam:
    """Newest frame of one feed as HxWx3 uint8 at recording size (decoded once per new JPEG)."""
    def __init__(self, feed):
        self.feed = feed; self.seq = -1; self.img = None; self.img_ts = 0.0

    def frame(self):
        f = self.feed
        with f._lock:
            seq, jpeg, bgr, ts = f._seq, f._jpeg, f._bgr, f._last_ts
        if seq != self.seq and (jpeg is not None or bgr is not None):
            try:
                if jpeg is not None:
                    im = Image.open(io.BytesIO(jpeg)); im.draft("RGB", (W, H)); im = im.convert("RGB")
                else:
                    im = Image.fromarray(bgr[:, :, ::-1].copy())
                if im.size != (W, H):        # keep aspect, pad black
                    im.thumbnail((W, H), Image.BILINEAR)
                    if im.size != (W, H):
                        bg = Image.new("RGB", (W, H)); bg.paste(im, ((W - im.size[0]) // 2, (H - im.size[1]) // 2)); im = bg
                self.img = np.asarray(im, dtype=np.uint8); self.seq = seq; self.img_ts = ts
            except Exception:
                pass
        return self.img, (time.time() - self.img_ts) if self.img_ts else 1e9


# ---------------- recorder ----------------
class Recorder:
    def __init__(self):
        self.tel = Telemetry(); self.tel.start()
        feeds, cfg = camera_feeds.make_feeds((W, H))
        self.feeds = camera_feeds.by_role(feeds)   # v3.7: [overview, gripper] by role, not display order
        for f in self.feeds: f.start()
        self.cams = [Cam(f) for f in self.feeds]
        self.state, self.ds, self.name, self.task = "idle", None, None, ""
        self.frames = 0; self.t_start = 0.0; self.reused = 0; self.skipped = 0; self.msg = ""
        self.episodes = 0; self.cmds = queue.Queue()
        # v3.8 session mode
        self.session = False; self.take = []; self.take_no = 0; self.take_t0 = 0.0; self.take_paused = False
        self.saveq = queue.Queue(); self.saving_n = 0; self.counts = collections.Counter(); self.last_label = ""
        self.save_lock = threading.Lock()
        self._loopstat = [0, 0.0, 0.0, 0, time.perf_counter()]
        threading.Thread(target=self._saver, daemon=True).start()

    # -- dataset handling --
    def _open(self, name):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        root = os.path.join(ROOT, name)
        if os.path.exists(os.path.join(root, "meta", "info.json")):
            ds = LeRobotDataset.resume(repo_id=f"local/{name}", root=root, image_writer_threads=4)
            want = features(); have = ds.meta.features
            for k, v in want.items():
                if k not in have or tuple(have[k]["shape"]) != tuple(v["shape"]):
                    raise RuntimeError(f"Datensatz '{name}' hat andere Merkmale ({k}) – neuen Namen wählen")
        else:
            if os.path.exists(root) and os.listdir(root):
                raise RuntimeError(f"Ordner {root} existiert, ist aber kein Datensatz – anderen Namen wählen")
            if os.path.exists(root):
                os.rmdir(root)
            os.makedirs(ROOT, exist_ok=True)
            ds = LeRobotDataset.create(repo_id=f"local/{name}", fps=FPS, features=features(), root=root,
                                       robot_type=ROBOT_TYPE, use_videos=True, image_writer_threads=4)
        return ds

    def _count(self, name):
        try:
            with open(os.path.join(ROOT, name, "meta", "info.json")) as f:
                return int(json.load(f).get("total_episodes", 0))
        except Exception:
            return 0

    # -- commands --
    def cmd_start(self, arg):
        if self.state in ("recording", "saving"):
            return
        if self.session or self.saving_n:
            self.msg = "Session läuft / speichert noch – erst beenden"; return
        try:
            a = json.loads(arg) if arg else {}
        except Exception:
            a = {}
        name = "".join(ch for ch in str(a.get("dataset", "pick_place_v1")) if ch.isalnum() or ch in "-_") or "pick_place_v1"
        self.task = str(a.get("task") or "Pick up the object and place it in the box").strip()
        if self.tel.fresh() is None:
            self.msg = "Keine Gelenkdaten – erst ▶ START (Teleop) drücken"; return
        for c, f in zip(self.cams, self.feeds):
            img, age = c.frame()
            if img is None or age > CAM_STALE_S:
                self.msg = f"{f.name}: kein Bild – iPhone prüfen"; return
        try:
            if self.state == "paused" and self.ds is not None and self.name == name:
                pass   # continue the paused episode
            elif self.ds is not None and self.ds.has_pending_frames():
                # a buffered episode exists (e.g. after a failed save) -> never drop it implicitly
                self.state = "paused"
                self.msg = f"⚠ Ungespeicherte Episode ({self.frames} Bilder) – erst speichern oder verwerfen"
                return
            else:
                if self.ds is None or self.name != name:
                    self._close()
                    self.ds = self._open(name); self.name = name
                self.frames = 0; self.reused = 0; self.skipped = 0; self.t_start = time.time()
            self.episodes = self._count(name)
            self.state = "recording"; self.msg = f"Aufnahme Episode {self.episodes + 1} …"
        except Exception as e:
            import traceback; elog("open failed:\n" + traceback.format_exc())
            self.msg = f"⚠ Fehler beim Öffnen: {e}"; self.state = "idle"

    # -- v3.8 session mode --
    def _args(self, arg):
        try:
            return json.loads(arg) if arg else {}
        except Exception:
            return {}

    def _check_inputs(self):
        if self.tel.fresh() is None:
            return "Keine Gelenkdaten – erst ▶ START (Teleop) drücken"
        for c, f in zip(self.cams, self.feeds):
            img, age = c.frame()
            if img is None or age > CAM_STALE_S:
                return f"{f.name}: kein Bild"
        return None

    def cmd_session_start(self, arg):
        if self.session:
            return
        if self.state in ("recording", "paused", "saving") or (self.ds is not None and self.ds.has_pending_frames()):
            self.msg = "Erst die laufende Episode speichern oder verwerfen"; return
        a = self._args(arg)
        name = "".join(ch for ch in str(a.get("dataset", "pick_place_v1")) if ch.isalnum() or ch in "-_") or "pick_place_v1"
        err = self._check_inputs()
        if err:
            self.msg = err; return
        root = os.path.join(ROOT, name)
        if os.path.exists(root) and os.listdir(root) and not os.path.exists(os.path.join(root, "meta", "info.json")):
            self.msg = f"Ordner {root} existiert, ist aber kein Datensatz – anderen Namen wählen"; return
        self._close()                      # the saver thread owns the dataset in session mode
        self.name, self.task = name, str(a.get("task") or "play").strip()
        self.session = True; self.counts = collections.Counter(); self.episodes = self._count(name)
        self._new_take()
        self.state = "session"; self.msg = f"Session läuft – Versuch {self.take_no}"
        elog(f"session start dataset {name}")

    def _new_take(self):
        self.take = []; self.take_no += 1; self.take_t0 = time.time(); self.take_paused = False
        self.reused = 0; self.skipped = 0

    def _end_take(self, label, task):
        take, n = self.take, len(self.take)
        t_wall = time.time() - self.take_t0
        if label == "discard":
            self.msg = f"Versuch {self.take_no} verworfen ({n / FPS:.1f} s)"; self.counts["discard"] += 1
        elif n < FPS:
            self.msg = f"Versuch {self.take_no} zu kurz (< 1 s) – nicht gespeichert"
        else:
            job = {"name": self.name, "task": task or self.task or "play", "label": label, "frames": take,
                   "take": self.take_no, "wall_s": round(t_wall, 2), "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S",
                   time.localtime(self.take_t0))}
            with self.save_lock:
                self.saving_n += 1
            self.saveq.put(job)
            self.msg = f"Versuch {self.take_no}: {label} – wird gespeichert ({n / FPS:.1f} s)"
        self.last_label = label
        elog(f"take {self.take_no} -> {label} ({n} frames)")

    def cmd_mark(self, arg):
        if not self.session:
            self.msg = "Keine Session aktiv"; return
        label, _, rest = arg.partition(" ")
        label = label.strip().lower()
        if label not in LABELS + ("discard",):
            self.msg = f"Unbekannte Markierung {label!r}"; return
        task = str(self._args(rest).get("task") or self.task).strip()
        self._end_take(label, task)
        self._new_take()
        self.msg += f" · Versuch {self.take_no} läuft"

    def cmd_session_stop(self, arg=""):
        if not self.session:
            return
        task = str(self._args(arg).get("task") or self.task).strip()
        self._end_take("play", task)
        self.session = False; self.take = []
        self.state = "saving" if self.saving_n else "idle"
        self.msg += " · Session beendet"
        elog("session stop")

    def _saver(self):
        """Background saver: the only place that touches the dataset in session mode (one take at a time).
        Opens/resumes the dataset, adds the take's frames, save_episode(), finalize() -> valid on disk after every take."""
        import cv2
        while True:
            job = self.saveq.get()
            t0 = time.time(); ds = None
            try:
                ds = self._open(job["name"])
                for fr in job["frames"]:
                    d = {"observation.state": fr[0], "observation.velocity": fr[1], "observation.effort": fr[2],
                         "action": fr[3], "task": job["task"]}
                    for k, jpg in zip(CAM_KEYS, fr[4]):
                        d[k] = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)[:, :, ::-1]
                    ds.add_frame(d)
                ds.save_episode(parallel_encoding=PAR_ENC)
                ds.finalize(); ds = None
                n_ep = self._count(job["name"])
                rec = {"episode_index": n_ep - 1, "label": job["label"], "task": job["task"],
                       "frames": len(job["frames"]), "seconds": round(len(job["frames"]) / FPS, 2),
                       "wall_s": job["wall_s"], "recorded_at": job["recorded_at"], "take": job["take"]}
                append_label(job["name"], rec)
                self.counts[job["label"]] += 1; self.episodes = n_ep
                emit(type="saved", dataset=job["name"], episodes=n_ep, frames=len(job["frames"]), label=job["label"],
                     task=job["task"], episode_index=n_ep - 1, save_s=round(time.time() - t0, 1),
                     path=os.path.join(ROOT, job["name"]))
                elog(f"saved take {job['take']} as episode {n_ep - 1} ({job['label']}, {len(job['frames'])} frames, "
                     f"{time.time() - t0:.1f} s)")
            except Exception as e:
                import traceback; elog("session save failed:\n" + traceback.format_exc())
                emit(type="error", msg=f"⚠ Speichern von Versuch {job['take']} fehlgeschlagen: {e}")
                if ds is not None:
                    try:
                        if ds.has_pending_frames(): ds.clear_episode_buffer()
                        ds.finalize()
                    except Exception:
                        pass
            finally:
                job["frames"] = None
                with self.save_lock:
                    self.saving_n -= 1

    def tick_session(self):
        s = self.tel.fresh()
        if s is None:
            self.skipped += 1
            if not self.take_paused and self.skipped_streak() > FPS // 2:
                self.take_paused = True; self.msg = "Session pausiert: keine Gelenkdaten (Teleop gestoppt?)"
            return
        self._streak = 0
        imgs = []
        for c, f in zip(self.cams, self.feeds):
            img, age = c.frame()
            if img is None or age > CAM_LOST_S:
                if not self.take_paused:
                    self.take_paused = True; self.msg = f"Session pausiert: {f.name} ohne Bild"
                return
            if age > CAM_STALE_S:
                self.reused += 1
            imgs.append(img)
        if self.take_paused:
            self.take_paused = False; self.msg = f"Session läuft – Versuch {self.take_no}"
        obs, act = s["obs"], s["act"]
        try:
            fr = (np.array([obs[k] for k in POS], dtype=np.float32),
                  np.array([obs.get(k, 0.0) for k in VEL], dtype=np.float32),
                  np.array([obs.get(k, 0.0) for k in TRQ], dtype=np.float32),
                  np.array([act[k] for k in POS], dtype=np.float32),
                  [self._jpeg(img) for img in imgs])
        except KeyError as e:
            self.msg = f"⚠ Gelenkdaten unvollständig: {e}"; self.skipped += 1; return
        self.take.append(fr)
        if len(self.take) >= TAKE_MAX_S * FPS:
            self._end_take("play", self.task); self._new_take()
            self.msg += f" (automatisch geteilt nach {TAKE_MAX_S:.0f} s)"

    @staticmethod
    def _jpeg(rgb):
        import cv2
        ok, b = cv2.imencode(".jpg", np.ascontiguousarray(rgb[:, :, ::-1]), [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        return b.tobytes()

    def cmd_save(self):
        if self.ds is None or self.frames == 0:
            self.msg = "Nichts zu speichern"; self.state = "idle"; return
        if self.frames < FPS:          # < 1 s
            self.ds.clear_episode_buffer(); self.msg = "Zu kurz (< 1 s) – verworfen"; self.state = "idle"; return
        self.t_save = time.time()
        self.state = "saving"; self.msg = f"Speichere Episode {self.episodes + 1} ({self.frames} Bilder) …"; self.status()
        t0 = time.time()
        before = self._count(self.name)
        try:
            self.ds.save_episode(parallel_encoding=PAR_ENC)
            self._close()               # finalize -> dataset valid on disk
            self.episodes = self._count(self.name)
            if self.episodes != before + 1:   # verify on disk (meta/info.json)
                raise RuntimeError(f"info.json zeigt {self.episodes} Episoden statt {before + 1}")
            self.msg = f"Episode {self.episodes} gespeichert ({self.frames} Bilder, {time.time() - t0:.1f} s)"
            try:
                append_label(self.name, {"episode_index": self.episodes - 1, "label": "play", "task": self.task,
                                         "frames": self.frames, "seconds": round(self.frames / FPS, 2),
                                         "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.t_start))})
            except Exception:
                pass
            emit(type="saved", dataset=self.name, episodes=self.episodes, frames=self.frames, label="play",
                 wall_s=round(self.t_save - self.t_start, 2),
                 reused=self.reused, skipped=self.skipped, path=os.path.join(ROOT, self.name))
            self.state = "idle"; self.frames = 0
        except Exception as e:
            import traceback; elog("save failed:\n" + traceback.format_exc())
            self.msg = f"⚠ SPEICHERN FEHLGESCHLAGEN: {e}"
            emit(type="error", msg=self.msg)
            pend = self.ds is not None and self.ds.has_pending_frames()
            self.state = "paused" if pend else "idle"
            if not pend:
                self.frames = 0

    def cmd_discard(self):
        if self.ds is not None and self.ds.has_pending_frames():
            self.ds.clear_episode_buffer()
        self.msg = "Episode verworfen" if self.frames else "Nichts zu verwerfen"
        self.state = "idle"; self.frames = 0

    def cmd_pause(self, why="pausiert"):
        if self.session:      # v3.8: GUI Stop -> session pauses (frames of the take are kept)
            self.take_paused = True; self.msg = f"Session {why}"; return
        if self.state == "recording":
            self.state = "paused"; self.msg = f"Aufnahme {why} – speichern, verwerfen oder weiter"

    def _close(self):
        if self.ds is not None:
            try:
                if self.ds.has_pending_frames():
                    self.ds.clear_episode_buffer()
                self.ds.finalize()
            except Exception as e:
                emit(type="error", msg=f"⚠ finalize fehlgeschlagen: {e}")
                print(f"finalize failed: {e}", file=sys.stderr, flush=True)
            self.ds = None

    # -- loop --
    def status(self):
        key = (self.state, self.msg)
        if key != getattr(self, "_last_key", None):
            self._last_key = key; elog(f"state {self.state} frames {self.frames} | {self.msg}")
        el = (time.time() - self.t_start) if self.state in ("recording", "paused") and self.t_start else 0.0
        if not self.session and self.state == "saving" and self.saving_n == 0 and self.ds is None:
            self.state = "idle"
        emit(type="status", state=self.state, dataset=self.name, episodes=self.episodes, frames=self.frames,
             seconds=round(self.frames / FPS, 1), wall=round(el, 1), reused=self.reused, skipped=self.skipped,
             joints=self.tel.fresh() is not None,
             pending=bool(self.ds is not None and self.ds.has_pending_frames()),
             cams=[(c.frame()[1] < CAM_STALE_S) for c in self.cams], msg=self.msg,
             session=self.session, take=self.take_no, take_frames=len(self.take) if self.session else 0,
             take_seconds=round(len(self.take) / FPS, 1) if self.session else 0.0, take_paused=self.take_paused,
             saving=self.saving_n, counts=dict(self.counts), last_label=self.last_label)

    def tick(self):
        s = self.tel.fresh()
        if s is None:
            self.skipped += 1
            if self.skipped_streak() > FPS // 2:
                self.cmd_pause("angehalten: keine Gelenkdaten (Teleop gestoppt?)")
            return
        self._streak = 0
        imgs = []
        for c, f in zip(self.cams, self.feeds):
            img, age = c.frame()
            if img is None or age > CAM_LOST_S:
                self.cmd_pause(f"angehalten: {f.name} ohne Bild"); return
            if age > CAM_STALE_S:
                self.reused += 1
            imgs.append(img)
        obs, act = s["obs"], s["act"]
        try:
            frame = {"observation.state": np.array([obs[k] for k in POS], dtype=np.float32),
                     "observation.velocity": np.array([obs.get(k, 0.0) for k in VEL], dtype=np.float32),
                     "observation.effort": np.array([obs.get(k, 0.0) for k in TRQ], dtype=np.float32),
                     "action": np.array([act[k] for k in POS], dtype=np.float32), "task": self.task}
        except KeyError as e:
            self.msg = f"⚠ Gelenkdaten unvollständig: {e}"; self.skipped += 1; return
        for k, img in zip(CAM_KEYS, imgs):
            frame[k] = img
        self.ds.add_frame(frame); self.frames += 1

    _streak = 0
    def skipped_streak(self):
        self._streak += 1; return self._streak

    def run(self):
        def reader():
            for line in sys.stdin:
                self.cmds.put(line.strip())
            self.cmds.put("quit")       # stdin closed (GUI gone) -> finalize and exit
        threading.Thread(target=reader, daemon=True).start()
        period = 1.0 / FPS; nxt = time.perf_counter(); last_status = 0.0
        def preload():   # import LeRobot's dataset code early so the first "start" is instant
            try:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
            except Exception as e:
                emit(type="log", msg=f"LeRobot dataset import failed: {e}")
        threading.Thread(target=preload, daemon=True).start()
        emit(type="ready", fps=FPS, size=[W, H], port=PORT, root=ROOT)
        elog(f"recorder ready pid {os.getpid()} root {ROOT}")
        while True:
            while not self.cmds.empty():
                line = self.cmds.get(); cmd, _, arg = line.partition(" ")
                elog(f"cmd {line[:200]!r} (state {self.state}, frames {self.frames})")
                if cmd == "start": self.cmd_start(arg)
                elif cmd == "save": self.cmd_save()
                elif cmd == "discard": self.cmd_discard()
                elif cmd == "pause": self.cmd_pause()
                elif cmd == "session_start": self.cmd_session_start(arg)
                elif cmd == "mark": self.cmd_mark(arg)
                elif cmd == "session_stop": self.cmd_session_stop(arg)
                elif cmd == "quit":
                    if self.session:
                        self.cmd_session_stop()
                    t_q = time.time()
                    while self.saving_n and time.time() - t_q < 600:   # finish background saves (max 10 min)
                        self.status(); time.sleep(0.5)
                    if self.ds is not None and self.ds.has_pending_frames() and self.frames >= FPS:
                        elog(f"quit with buffered episode ({self.frames} frames) -> auto-save")
                        self.cmd_save()
                    self._close(); emit(type="bye"); elog("bye"); return
                self.status()
            t_tick = time.perf_counter()
            if self.state == "recording":
                self.tick()
            elif self.session and not (self.take_paused and self.tel.fresh() is None):
                self.tick_session()
            if self.state == "recording" or self.session:   # v3.8: loop timing stats -> recorder.log every 10 s
                dt = time.perf_counter() - t_tick
                ls = self._loopstat
                ls[0] += 1; ls[1] += dt; ls[2] = max(ls[2], dt)
                if time.perf_counter() - ls[4] > 10:
                    elog(f"loop: {ls[0] / (time.perf_counter() - ls[4]):.1f} Hz, tick avg {1000 * ls[1] / max(1, ls[0]):.1f} ms, "
                         f"max {1000 * ls[2]:.1f} ms, resyncs {ls[3]}")
                    self._loopstat = [0, 0.0, 0.0, 0, time.perf_counter()]
            now = time.perf_counter()
            if now - last_status > 0.5:
                last_status = now; self.status()
            nxt += period
            d = nxt - time.perf_counter()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.perf_counter()     # overrun -> resync (frame counted by wall time anyway)
                self._loopstat[3] += 1


if __name__ == "__main__":
    Recorder().run()
