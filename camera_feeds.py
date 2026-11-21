#!/usr/bin/env python3
"""Live camera feeds for the teleop GUI (v3.1.1).

Design goals
- Never block or slow teleoperation: teleop runs in its own process (teleop.sh -> lerobot-teleoperate);
  the feeds run in daemon threads inside the GUI process only. Each reader thread keeps only the newest
  frame; decoding/scaling happens at most display_fps times per second, and only at preview size.
- Sources (per feed, from ~/humanoid/cameras.json, overridable via env TELEOP_CAM1 / TELEOP_CAM2):
    "http://..." / "https://..."  MJPEG stream (multipart/x-mixed-replace), e.g. the iPhone browser bridge
                                  http://127.0.0.1:8090/stream.mjpg. Parsed by hand (JPEG SOI/EOI markers),
                                  so frames that are not shown are never decoded. A plain image URL
                                  (image/jpeg, e.g. /latest.jpg) is polled instead.
    "0", "1", ...                 OpenCV camera index (AVFoundation on macOS), e.g. a USB webcam.
    "" or "none"                  not configured -> panel shows "Kein Signal" (no signal).
- Auto-reconnect: on error or when no data arrives for STALL_S seconds the source is reopened every RETRY_S s.
- Status per feed: measured incoming fps, frame size, connected / no signal.

Self-test without GUI:  ~/humanoid/.venv/bin/python camera_feeds.py [seconds]
"""
import io, json, os, struct, threading, time, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(HERE, "cameras.json")
DEFAULT_CONFIG = {
    "display_fps": 15,
    "feeds": [
        {"name": "Kamera 1: iPhone", "source": "http://127.0.0.1:8090/stream.mjpg",
         "hint": "iPhone: in Safari https://<lan-ip>:8444 öffnen, Bildschirm an lassen"},
        {"name": "Kamera 2: USB", "source": "",
         "hint": "Noch keine Kamera. Quelle in ~/humanoid/cameras.json eintragen (z.B. \"0\")"},
    ],
}
STALL_S = 3.0      # no data for this long -> reconnect
BUILTIN_PAT = ("facetime", "built-in", "integrated")   # v3.7: never used unless asked for by name / CAM_ALLOW_BUILTIN=1
_CAM_LIST = {"t": 0.0, "list": None}


def list_cameras(max_age=2.0):
    """v3.7: video devices in OpenCV's AVFoundation index order: [(index, name, unique_id), ...].
    OpenCV (cap_avfoundation_mac.mm) enumerates AVCaptureDevice video+muxed devices and sorts them by uniqueID,
    so the same sort over system_profiler's camera list gives the same indices. Cached for max_age seconds."""
    now = time.time()
    if _CAM_LIST["list"] is not None and now - _CAM_LIST["t"] < max_age:
        return _CAM_LIST["list"]
    import subprocess
    devs = []
    try:
        r = subprocess.run(["system_profiler", "SPCameraDataType", "-json"], capture_output=True, text=True, timeout=15)
        for d in json.loads(r.stdout or "{}").get("SPCameraDataType", []):
            name = d.get("_name") or d.get("spcamera_model-id") or "?"
            uid = d.get("spcamera_unique-id") or ""
            devs.append((uid, name))
    except Exception:
        devs = []
    devs.sort(key=lambda x: x[0])
    out = [(i, n, u) for i, (u, n) in enumerate(devs)]
    _CAM_LIST.update(t=now, list=out)
    return out


def is_builtin(name):
    n = (name or "").lower()
    return any(p in n for p in BUILTIN_PAT)


def resolve_source(src, devices=None):
    """v3.7: 'name:Innomaker|U20CAM' -> (index, device name) of the first camera whose name contains one of the
    |-separated patterns (case-insensitive). Built-in cameras (FaceTime) are never matched unless a pattern names
    them. Numeric sources are checked too: an index that points at the built-in camera is refused (indices shift
    when USB cameras reconnect) unless CAM_ALLOW_BUILTIN=1. Returns (None, reason) if nothing suitable is attached."""
    devices = list_cameras() if devices is None else devices
    names = ", ".join(n for _, n, _ in devices) or "keine"
    if src.lower().startswith("name:"):
        pats = [p.strip().lower() for p in src[5:].split("|") if p.strip()]
        for idx, name, _ in devices:
            ln = name.lower()
            for p in pats:
                if p in ln and (not is_builtin(name) or any(b in p for b in BUILTIN_PAT)):
                    return idx, name
        return None, f"'{src[5:]}' nicht angeschlossen (gefunden: {names})"
    if src.isdigit():
        idx = int(src)
        name = next((n for i, n, _ in devices if i == idx), None)
        if name is not None and is_builtin(name) and os.environ.get("CAM_ALLOW_BUILTIN", "0") != "1":
            return None, f"Index {idx} ist die eingebaute Kamera ({name}) – nicht verwendet; Quelle 'name:…' nutzen"
        return idx, name or f"Index {idx}"
    return src, src
RETRY_S = 2.0      # wait between reconnect attempts (first retry)
RETRY_MAX_S = 30.0  # v3.7: back-off cap - failing sources (e.g. no camera permission) are retried 2, 4, 8 ... 30 s
SIGNAL_S = 1.5     # newest frame older than this -> show "no signal"


def load_config():
    """Read cameras.json (created with defaults if missing); env TELEOP_CAM1/2 override the sources."""
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    try:
        with open(CONFIG_FILE) as f:
            user = json.load(f)
        cfg["display_fps"] = user.get("display_fps", cfg["display_fps"])
        for i, fd in enumerate(user.get("feeds", [])[:2]):
            cfg["feeds"][i].update(fd)
    except FileNotFoundError:
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(DEFAULT_CONFIG, f, indent=2, ensure_ascii=False)
        except OSError:
            pass
    except Exception as e:  # broken JSON -> defaults, but say so
        cfg["error"] = f"cameras.json: {e}"
    for i in range(2):
        v = os.environ.get(f"TELEOP_CAM{i + 1}")
        if v is not None:
            cfg["feeds"][i]["source"] = v
    return cfg


class Feed:
    """One camera source read in a daemon thread. Thread-safe getters for the GUI."""

    def __init__(self, name, source, box=(360, 240), display_fps=15, hint=""):
        self.name, self.source, self.hint = name, str(source or "").strip(), hint
        self.box, self.min_dt = box, 1.0 / max(1, display_fps)
        self._lock = threading.Lock()
        self._jpeg = None          # newest raw JPEG (URL sources)
        self._bgr = None           # newest raw frame (OpenCV sources)
        self._seq = 0              # incremented per incoming frame
        self._last_ts = 0.0
        self._fps = 0.0
        self._size = (0, 0)
        self._img, self._img_seq, self._img_ts = None, -1, 0.0
        self.state = "verbinde …" if self.configured else "nicht konfiguriert"
        self._stop = threading.Event()
        self._t = None
        self.device = None         # v3.7: resolved camera ('#1 HD Pro Webcam C920') for name/index sources
        self.role = ""             # v3.7: 'overview' / 'gripper' (make_feeds)
        self.publisher = None      # v3.8: FramePublisher -> shared memory for recorder/play_policy (GUI only)
        self._small = None         # v3.8: newest frame at shared size (BGR), cheap source for the preview
        self.overlay = None        # v3.8: {"dets": [(label, conf, x1, y1, x2, y2)], "size": (w, h), "ts": t}

    @property
    def configured(self):
        return self.source.lower() not in ("", "none", "off")

    # ---------------- life cycle ----------------
    def start(self):
        if self.configured and self._t is None:
            self._t = threading.Thread(target=self._run, name=f"feed-{self.name}", daemon=True)
            self._t.start()
        return self

    def stop(self):
        self._stop.set()

    # ---------------- reader thread ----------------
    def _got_frame(self, jpeg=None, bgr=None, small=None):
        now = time.time()
        if self.publisher is not None and small is None:
            try:   # v3.8: scale once per frame in the reader thread, share it with the recorder and the preview
                if bgr is None and jpeg is not None:
                    import cv2, numpy as np
                    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    jpeg = None
                if bgr is not None:
                    small = self.publisher.publish(bgr, now)
            except Exception:
                small = None
        with self._lock:
            self._jpeg, self._bgr = jpeg, bgr
            self._small = small
            self._seq += 1
            dt = now - self._last_ts if self._last_ts else 0
            if 0 < dt < 2:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt) if self._fps else 1.0 / dt
            else:
                self._fps = 0.0
            self._last_ts = now

    def retry_wait(self, fails):
        """v3.7: exponential back-off between reconnects (2, 4, 8, 16, 30, 30 ... s)."""
        return min(RETRY_MAX_S, RETRY_S * (2 ** max(0, fails - 1)))

    def _run(self):
        fails = 0
        while not self._stop.is_set():
            seq0 = self._seq
            try:
                if self.source.startswith(("http://", "https://")):
                    self._read_url()
                else:
                    self._read_opencv()
            except ConnectionError as e:
                self.state = f"getrennt ({e}), neuer Versuch …" if str(e) else "getrennt, neuer Versuch …"
            except Exception as e:
                self.state = f"getrennt ({type(e).__name__}), neuer Versuch …"
            fails = 0 if self._seq != seq0 else fails + 1   # frames arrived -> reset the back-off
            wait = self.retry_wait(max(1, fails))
            if fails > 1:
                self.state = f"{self.state.split(', neuer Versuch')[0]}, neuer Versuch in {wait:.0f} s"
            self._stop.wait(wait)

    def _read_url(self):
        self.state = "verbinde …"
        with urllib.request.urlopen(self.source, timeout=STALL_S) as r:
            if r.headers.get("Content-Type", "").startswith("image/"):   # single image URL -> poll it
                self._poll_image(r.read())
                return
            self.state = "verbunden"
            buf = b""
            while not self._stop.is_set():
                chunk = r.read1(65536)          # socket timeout STALL_S -> exception -> reconnect
                if not chunk:
                    raise ConnectionError("stream ended")
                buf += chunk
                end = buf.rfind(b"\xff\xd9")    # keep only the newest complete JPEG
                if end < 0:
                    if len(buf) > 8_000_000:
                        buf = b""
                    continue
                start = buf.rfind(b"\xff\xd8", 0, end)
                if start >= 0:
                    self._got_frame(jpeg=buf[start:end + 2])
                buf = buf[end + 2:]

    def _poll_image(self, first):
        self.state = "verbunden"
        self._got_frame(jpeg=first)
        while not self._stop.is_set():
            time.sleep(self.min_dt)
            with urllib.request.urlopen(self.source, timeout=STALL_S) as r:
                self._got_frame(jpeg=r.read())

    def _read_opencv(self):
        import cv2  # lazy: only needed for USB cameras
        src = self.source
        if src.lower().startswith("name:") or src.isdigit():
            src, info = resolve_source(src)          # v3.7: re-resolved on every (re)connect
            if src is None:
                self.device = None
                raise ConnectionError(info)
            self.device = f"#{src} {info}"
        self.state = "verbinde …"
        cap = cv2.VideoCapture(src, cv2.CAP_AVFOUNDATION) if isinstance(src, int) else cv2.VideoCapture(src)
        try:
            if not cap.isOpened():
                raise ConnectionError("cannot open")
            self.state = "verbunden"
            fails = 0
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    fails += 1
                    if fails > 30:
                        raise ConnectionError("no frames")
                    time.sleep(0.05)
                    continue
                fails = 0
                self._got_frame(bgr=frame)
        finally:
            cap.release()

    # ---------------- GUI side ----------------
    def has_signal(self):
        return bool(self._last_ts) and (time.time() - self._last_ts) < SIGNAL_S

    def status_text(self):
        if not self.configured:
            return "Kein Signal (nicht konfiguriert)"
        if not self.has_signal():
            return f"Kein Signal ({self.state})"
        w, h = self._size
        dev = f"  {self.device}" if self.device else ""
        return (f"● {self._fps:4.1f} fps  {w}×{h}{dev}" if w else f"● {self._fps:4.1f} fps{dev}")

    def frame_fit(self, box, upscale_max=(1920, 1080)):
        """v3.7 (enlarge window): newest frame scaled to fit box (aspect kept, at most upscale_max), or None.
        Uses the same newest-frame buffer as preview(); the caller decides how often to call it."""
        with self._lock:
            seq, jpeg, bgr = self._seq, self._jpeg, self._bgr
        if jpeg is None and bgr is None:
            return None, seq
        from PIL import Image
        bw = max(1, min(int(box[0]), upscale_max[0])); bh = max(1, min(int(box[1]), upscale_max[1]))
        try:
            if jpeg is not None:
                im = Image.open(io.BytesIO(jpeg)); im.draft("RGB", (bw, bh)); im = im.convert("RGB")
                w, h = im.size
            else:
                h, w = bgr.shape[:2]
            s = min(bw / w, bh / h)
            size = (max(1, int(w * s)), max(1, int(h * s)))
            if jpeg is not None:
                im = im.resize(size, Image.BILINEAR) if size != im.size else im
                return draw_overlay(im, self.overlay, (w, h)), seq
            else:
                import cv2
                small = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
                im = Image.fromarray(small[:, :, ::-1].copy())
            return draw_overlay(im, self.overlay, (w, h)), seq
        except Exception:
            return None, seq

    def preview(self):
        """Return (PIL.Image, seq) scaled to fit self.box, or (None, seq). Decodes at most display_fps
        times per second and only when a new frame arrived. JPEGs are decoded with PIL's draft mode
        (DCT downscaling straight to about preview size), so this stays cheap."""
        now = time.time()
        with self._lock:
            seq, jpeg, bgr, small = self._seq, self._jpeg, self._bgr, self._small
        if seq == self._img_seq or now - self._img_ts < self.min_dt or (jpeg is None and bgr is None):
            return self._img, self._img_seq
        full = (bgr.shape[1], bgr.shape[0]) if bgr is not None else None
        if small is not None:
            bgr = small        # v3.8: already scaled to the shared size -> much cheaper resize
        from PIL import Image
        try:
            if jpeg is not None:
                im = Image.open(io.BytesIO(jpeg))
                self._size = im.size
                im.draft("RGB", self.box)
                im = im.convert("RGB")
            else:
                import cv2
                h, w = bgr.shape[:2]
                self._size = full or (w, h)
                s = min(self.box[0] / w, self.box[1] / h)
                small = cv2.resize(bgr, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_AREA)
                im = Image.fromarray(small[:, :, ::-1].copy())
            im.thumbnail(self.box)
            im = draw_overlay(im, self.overlay, self._size)   # v3.8 detection boxes
            self._img, self._img_seq, self._img_ts = im, seq, now
        except Exception:
            pass   # corrupt frame -> keep previous image
        return self._img, self._img_seq


def guess_role(fd, i):
    """v3.7: 'overview' or 'gripper' from cameras.json "role", else from the name, else by position."""
    r = str(fd.get("role", "")).strip().lower()
    if r in ("overview", "gripper"):
        return r
    n = str(fd.get("name", "")).lower()
    if any(k in n for k in ("greifer", "gripper", "wrist", "handgelenk")):
        return "gripper"
    if any(k in n for k in ("übersicht", "uebersicht", "overview", "top")):
        return "overview"
    return ("overview", "gripper")[i] if i < 2 else ""


def make_feeds(box=(360, 240), shared=None):
    """shared=None: use the GUI's shared frames if env HUMANOID_CAM_SHM=1 (set by the GUI for its children:
    recorder.py, play_policy) - then the cameras are opened only once, by the GUI. Otherwise open them directly."""
    cfg = load_config()
    if shared is None:
        shared = os.environ.get("HUMANOID_CAM_SHM", "0") == "1"
    feeds = []
    for i, fd in enumerate(cfg["feeds"]):
        role = guess_role(fd, i)
        if shared and role and str(fd.get("source", "")).strip().lower() not in ("", "none", "off"):
            f = ShmFeed(fd["name"], role, box, cfg.get("display_fps", 15), fd.get("hint", ""))
        else:
            f = Feed(fd["name"], fd.get("source", ""), box, cfg.get("display_fps", 15), fd.get("hint", ""))
        f.role = role
        feeds.append(f)
    return feeds, cfg


# ---------------- shared frames (v3.8) ----------------
# The GUI owns the cameras. Each reader thread scales the newest frame once to SHM_W x SHM_H (aspect kept) and
# writes it into a POSIX shared-memory block "<prefix><role>" (seqlock: odd sequence = being written). recorder.py
# and play_policy.py read it through ShmFeed, so the 1080p cameras are opened and decoded only once.
SHM_PREFIX = os.environ.get("HUMANOID_CAM_SHM_PREFIX", "humanoid_cam_")
SHM_W = int(os.environ.get("REC_WIDTH", "640")); SHM_H = int(os.environ.get("REC_HEIGHT", "360"))
_HDR = struct.Struct("<8sQdIII")          # magic, seq, ts, w, h, full_w (padded to 64 bytes)
_HDR_SIZE = 64
_MAGIC = b"HCAM0001"


def _shm_open(name, size=None):
    """Create (size given) or attach a shared memory block without Python's resource tracker
    (it would unlink the block when any process using it exits)."""
    from multiprocessing import shared_memory, resource_tracker
    if size is None:
        shm = shared_memory.SharedMemory(name=name)
    else:
        try:
            shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shm = shared_memory.SharedMemory(name=name)
            if shm.size < size:                # left over with another size -> recreate
                shm.close(); shm.unlink()
                shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass
    return shm


class FramePublisher:
    def __init__(self, role, w=None, h=None, prefix=None):
        self.w, self.h = int(w or SHM_W), int(h or SHM_H)
        self.name = (prefix or SHM_PREFIX) + role
        self.shm = _shm_open(self.name, _HDR_SIZE + self.w * self.h * 3)
        self.seq = 0
        _HDR.pack_into(self.shm.buf, 0, _MAGIC, 0, 0.0, 0, 0, 0)

    def publish(self, bgr, ts=None):
        """Scale bgr to fit w x h (aspect kept), write it, return the scaled frame."""
        import cv2, numpy as np
        h0, w0 = bgr.shape[:2]
        s = min(self.w / w0, self.h / h0)
        size = (max(1, int(round(w0 * s))), max(1, int(round(h0 * s))))
        small = bgr if size == (w0, h0) else cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
        small = np.ascontiguousarray(small)
        n = small.size
        buf = self.shm.buf
        self.seq += 1
        _HDR.pack_into(buf, 0, _MAGIC, 2 * self.seq - 1, ts or time.time(), size[0], size[1], w0)   # odd: writing
        buf[_HDR_SIZE:_HDR_SIZE + n] = small.reshape(-1).data
        _HDR.pack_into(buf, 0, _MAGIC, 2 * self.seq, ts or time.time(), size[0], size[1], w0)       # even: done
        return small

    def close(self):
        try: self.shm.close()
        except Exception: pass


class ShmReader:
    def __init__(self, role, prefix=None):
        self.name = (prefix or SHM_PREFIX) + role
        self.shm = _shm_open(self.name)

    def read(self, last_seq=-1):
        """-> (seq, ts, bgr copy) for a new frame, or None (nothing new / being written / no frame yet)."""
        import numpy as np
        buf = self.shm.buf
        magic, s1, ts, w, h, _fw = _HDR.unpack_from(buf, 0)
        if magic != _MAGIC or s1 == 0 or s1 % 2 or s1 == last_seq or w == 0:
            return None
        n = w * h * 3
        if _HDR_SIZE + n > self.shm.size:
            return None
        img = np.frombuffer(buf, np.uint8, n, _HDR_SIZE).reshape(h, w, 3).copy()
        if _HDR.unpack_from(buf, 0)[1] != s1:      # overwritten while copying -> try again next time
            return None
        return s1, ts, img

    def close(self):
        try: self.shm.close()
        except Exception: pass


class ShmFeed(Feed):
    """Feed that reads the GUI's shared frames (no camera access). Same getters as Feed."""

    def __init__(self, name, role, box=(360, 240), display_fps=15, hint="", prefix=None):
        super().__init__(name, f"shm:{role}", box, display_fps, hint)
        self.role, self._prefix = role, prefix
        self.device = "geteilt vom GUI"

    def _run(self):
        rd, last = None, -1
        while not self._stop.is_set():
            if rd is None:
                try:
                    rd = ShmReader(self.role, self._prefix); self.state = "verbunden (GUI)"
                except Exception:
                    self.state = "warte auf Kamerabilder vom GUI …"
                    self._stop.wait(1.0); continue
            r = rd.read(last)
            if r is None:
                self._stop.wait(0.004); continue
            last, ts, img = r
            self._got_frame(bgr=img, small=img)
            self._last_ts = min(self._last_ts, ts) if ts else self._last_ts   # age = age of the camera frame


def draw_overlay(im, ov, full_size, max_age=1.0):
    """v3.8: draw detection boxes (original pixel coords) on a scaled PIL image."""
    if not ov or not ov.get("dets") or time.time() - ov.get("ts", 0) > max_age or not full_size or not full_size[0]:
        return im
    try:
        from PIL import ImageDraw
        im = im.copy(); d = ImageDraw.Draw(im)
        sx, sy = im.size[0] / full_size[0], im.size[1] / full_size[1]
        lw = max(1, int(round(min(im.size) / 180)))
        for lab, conf, x1, y1, x2, y2 in ov["dets"][:20]:
            box = [x1 * sx, y1 * sy, x2 * sx, y2 * sy]
            d.rectangle(box, outline=(255, 210, 0), width=lw)
            t = f"{lab} {conf:.2f}"
            d.rectangle([box[0], max(0, box[1] - 12), box[0] + 7 * len(t), box[1]], fill=(255, 210, 0))
            d.text((box[0] + 2, max(0, box[1] - 12)), t, fill=(0, 0, 0))
    except Exception:
        pass
    return im


def by_role(feeds):
    """v3.7: [overview feed, gripper feed] whatever the display order in cameras.json is (the recorder and
    play_policy store feed 0 as observation.images.overview and feed 1 as .gripper)."""
    ov = next((f for f in feeds if getattr(f, "role", "") == "overview"), None)
    gr = next((f for f in feeds if getattr(f, "role", "") == "gripper" and f is not ov), None)
    if ov is None or gr is None:
        return list(feeds[:2])
    return [ov, gr]


# ---------------- Tk widget ----------------
_BIG = {}   # feed name -> open enlarge window (one per feed)


def open_large(parent, feed):
    """v3.7: large resizable live window for one feed (fit to window, aspect kept, up to 1920x1080).
    ✕ button or Esc closes it; the feed itself keeps running (the panel and the recorder use the same thread)."""
    import tkinter as tk
    old = _BIG.get(feed.name)
    if old is not None:
        try:
            old.deiconify(); old.lift(); old.focus_force(); return old
        except tk.TclError:
            _BIG.pop(feed.name, None)
    top = tk.Toplevel(parent)
    top.title(feed.name)
    sw, sh = top.winfo_screenwidth(), top.winfo_screenheight()
    w0, h0 = min(1280, sw - 80), min(760, sh - 120)
    top.geometry(f"{w0}x{h0}+40+40"); top.minsize(320, 220); top.maxsize(1920, 1080 + 60)
    top.configure(bg="black")
    bar = tk.Frame(top, bg="#222222"); bar.pack(fill="x")
    tk.Label(bar, text=feed.name, fg="white", bg="#222222", font=("Helvetica", 14, "bold")).pack(side="left", padx=8)
    st = tk.Label(bar, text="", fg="#aaaaaa", bg="#222222", font=("Menlo", 11)); st.pack(side="left", padx=8)
    cv = tk.Canvas(top, bg="black", highlightthickness=0); cv.pack(fill="both", expand=True)
    img_id = cv.create_image(0, 0, anchor="center")
    txt_id = cv.create_text(0, 0, fill="#bbbbbb", font=("Helvetica", 18, "bold"), text="Kein Signal")
    s = {"seq": -2, "size": (0, 0), "photo": None, "job": None, "alive": True}
    ms = max(30, int(1000 * feed.min_dt))   # display_fps (15 fps -> 66 ms)

    def close(_e=None):
        s["alive"] = False
        if s["job"]:
            try: top.after_cancel(s["job"])
            except Exception: pass
        _BIG.pop(feed.name, None)
        try: top.destroy()
        except Exception: pass

    tk.Button(bar, text="✕", command=close, font=("Helvetica", 14, "bold"), width=3).pack(side="right", padx=6, pady=3)
    top.bind("<Escape>", close); top.protocol("WM_DELETE_WINDOW", close)

    def tick():
        if not s["alive"]:
            return
        try:
            cw, ch = max(1, cv.winfo_width()), max(1, cv.winfo_height())
            cv.coords(img_id, cw // 2, ch // 2); cv.coords(txt_id, cw // 2, ch // 2)
            sig = feed.has_signal()
            if sig:
                seq = feed._seq
                if seq != s["seq"] or (cw, ch) != s["size"]:     # decode only for a new frame or a resize
                    im, seq = feed.frame_fit((cw, ch))
                    if im is not None:
                        from PIL import ImageTk
                        s["photo"] = ImageTk.PhotoImage(im)
                        cv.itemconfigure(img_id, image=s["photo"]); cv.itemconfigure(txt_id, text="")
                        s["seq"], s["size"] = seq, (cw, ch)
            elif s["seq"] != -1:
                cv.itemconfigure(img_id, image=""); s["photo"] = None
                cv.itemconfigure(txt_id, text="Kein Signal\n" + feed.status_text()); s["seq"] = -1
            st.config(text=feed.status_text())
        except tk.TclError:
            s["alive"] = False; _BIG.pop(feed.name, None); return
        s["job"] = top.after(ms, tick)

    _BIG[feed.name] = top
    top.focus_force()
    tick()
    return top


def make_panel(parent, feed, box=(360, 240)):
    """LabelFrame with a fixed-size black image area and a status line. Returns (frame, update)."""
    import tkinter as tk
    fr = tk.LabelFrame(parent, text=feed.name)
    cv = tk.Canvas(fr, width=box[0], height=box[1], bg="black", highlightthickness=0)
    cv.pack(padx=4, pady=(2, 0))
    img_id = cv.create_image(box[0] // 2, box[1] // 2, anchor="center")
    txt_id = cv.create_text(box[0] // 2, box[1] // 2, fill="#bbbbbb", font=("Helvetica", 13, "bold"),
                            width=box[0] - 20, justify="center", text="Kein Signal")
    row = tk.Frame(fr); row.pack(fill="x", padx=4)
    tk.Button(row, text="⤢", width=2, command=lambda: open_large(parent, feed)).pack(side="right")   # v3.7 enlarge
    st = tk.Label(row, text="", fg="gray", font=("Menlo", 10), anchor="w")
    st.pack(side="left", fill="x", expand=True)
    cv.bind("<Double-Button-1>", lambda _e: open_large(parent, feed))
    cv.configure(cursor="hand2")
    state = {"seq": -2, "photo": None}

    def update():
        im, seq = feed.preview()
        sig = feed.has_signal()
        if sig and im is not None and seq != state["seq"]:
            from PIL import ImageTk
            state["photo"] = ImageTk.PhotoImage(im)
            cv.itemconfigure(img_id, image=state["photo"])
            cv.itemconfigure(txt_id, text="")
            state["seq"] = seq
        elif not sig and state["seq"] != -1:
            cv.itemconfigure(img_id, image="")
            state["photo"] = None
            cv.itemconfigure(txt_id, text="Kein Signal\n\n" + feed.hint)
            state["seq"] = -1
        st.config(text=feed.status_text(), fg="#1a9c3a" if sig else "gray")

    return fr, update


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--list":     # v3.7: show cameras in OpenCV index order
        for i, n, u in list_cameras():
            print(f"{i}: {n}  ({u}){'  [eingebaut – wird nie automatisch benutzt]' if is_builtin(n) else ''}")
        for fd in load_config()["feeds"]:
            s = str(fd.get("source", ""))
            print(f"{fd['name']}: source={s!r} -> {resolve_source(s) if (s.lower().startswith('name:') or s.isdigit()) else s}")
        raise SystemExit(0)
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5
    feeds, cfg = make_feeds()
    for f in feeds:
        f.start()
    t0 = time.time(); n_prev = [0, 0]
    while time.time() - t0 < secs:
        for i, f in enumerate(feeds):
            im, seq = f.preview()
            if im is not None and seq != n_prev[i]:
                n_prev[i] = seq
        time.sleep(0.02)
    for i, f in enumerate(feeds):
        print(f"feed{i + 1} {f.name!r} source={f.source!r}: {f.status_text()} | frames_in={f._seq}"
              f" preview={'%dx%d' % f._img.size if f._img else None}")
        if f._img is not None:
            os.makedirs(os.path.join(HERE, "outputs"), exist_ok=True)
            out = os.path.join(HERE, "outputs", f"feed{i + 1}_preview.jpg")
            f._img.save(out); print("  saved", out)
