#!/usr/bin/env python3
"""GUI 3.8 add-ons for teleop_gui.py (labels German, code/docs English).

1. Session recording with live markers ("Session aufnehmen"): recorder.py records takes back to back;
   ✓ geklappt (S) / ✗ fehlgeschlagen (F) / ↻ Neuer Versuch (N) / 🗑 Verwerfen (D) end the current take with
   label success / fail / play / (dropped) and start the next one at once. Saving runs in the recorder's
   background thread. Keyboard shortcuts work while no text field has the focus.
2. Vision tools (all optional - the app works without them):
   - "📐 Neu kalibrieren": vision.calibrate.recalibrate_table(<newest full-size frame of the overview camera>),
     shows message, shift_mm and warnings. The frame comes from the GUI's own feed (no second camera open).
   - "🔍 Erkennung" toggle + prompt field: vision.detector.Detector (MPS) on the overview feed in a worker
     thread, at most DETECT_MAX_FPS (8); boxes are drawn on the overview preview and its enlarge window.
   - "🤝 Arm mit Tisch verbinden": step (c) robot-link capture with on-screen prompts (only while teleop runs and
     no recording/session is active). Joint angles come from the telemetry that recorder.py forwards to
     127.0.0.1:8766 (TELEMETRY_FWD_PORT), so there is no port conflict with the recorder (it keeps 8765).
     Read-only: no motor commands.
"""
import json, os, queue, sys, threading, time, tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
VISION_DIR = os.environ.get("HUMANOID_VISION_DIR", os.path.join(HERE, "vision"))
DETECT_MAX_FPS = float(os.environ.get("DETECT_MAX_FPS", "8"))
FWD_PORT = int(os.environ.get("TELEMETRY_FWD_PORT", "8766"))
DEFAULT_PROMPTS = os.environ.get("DETECT_PROMPTS", "blue ball, trash bin")
KEYS = {"s": "success", "f": "fail", "n": "play", "d": "discard"}


def _vision_import(mod):
    if VISION_DIR not in sys.path:
        sys.path.insert(0, VISION_DIR)
    return __import__(mod)


def overview_feed(feeds):
    for f in feeds or []:
        if getattr(f, "role", "") == "overview":
            return f
    return None


def fmt_shift(r):
    s = r.get("shift_mm")
    return f"Verschiebung: Mittel {s['mean']:.1f} mm, max {s['max']:.1f} mm" if s else "Verschiebung: – (erste Kalibrierung)"


class SessionUI:
    """Toggle + 4 marker buttons + status line. ctx: rec_cmd(str), rec (dict with 'state'), ds_name, task_text
    (tk vars), log(str)."""

    def __init__(self, root, parent, ctx):
        self.root, self.ctx = root, ctx
        self.active = False; self.last = {}
        fr = tk.Frame(parent); fr.pack(fill="x", pady=(4, 0))
        self.btog = tk.Button(fr, text="⏺  Session aufnehmen", font=("Helvetica", 13, "bold"), fg="#c0392b",
                              command=self.toggle)
        self.btog.pack(side="left", padx=2)
        self.info = tk.Label(fr, text="Dauer-Aufnahme: aus", fg="gray", font=("Menlo", 11), anchor="w")
        self.info.pack(side="left", padx=8, fill="x", expand=True)
        fb = tk.Frame(parent); fb.pack(fill="x", pady=(2, 2))
        spec = [("✓ geklappt  (S)", "success", "#1e8449"), ("✗ fehlgeschlagen  (F)", "fail", "#c0392b"),
                ("↻ Neuer Versuch  (N)", "play", "#2e4053"), ("🗑 Verwerfen  (D)", "discard", "#7f8c8d")]
        self.bmark = {}
        for i, (txt, lab, col) in enumerate(spec):
            b = tk.Button(fb, text=txt, font=("Helvetica", 14, "bold"), fg=col, height=2, state="disabled",
                          command=lambda l=lab: self.mark(l))
            b.grid(row=0, column=i, sticky="ew", padx=2); fb.grid_columnconfigure(i, weight=1)
            self.bmark[lab] = b
        root.bind_all("<KeyPress>", self._key, add="+")

    def _task(self):
        t = self.ctx["task_text"].get().strip()
        return t or "play"

    def toggle(self):
        if self.active:
            self.ctx["rec_cmd"]("session_stop " + json.dumps({"task": self._task()}))
            self.btog.config(text="◐ beendet …", state="disabled")
        else:
            if self.ctx["rec"].get("state") in ("recording", "paused", "saving"):
                self.ctx["log"]("Erst die laufende Episode speichern oder verwerfen.\n"); return
            self.ctx["rec_cmd"]("session_start " + json.dumps({"dataset": self.ctx["ds_name"].get().strip(),
                                                               "task": self._task()}))

    def mark(self, label):
        if not self.active:
            return
        self.ctx["rec_cmd"](f"mark {label} " + json.dumps({"task": self._task()}))
        b = self.bmark[label]; b.config(relief="sunken")
        self.root.after(250, lambda: b.config(relief="raised"))

    def _key(self, e):
        try:
            w = e.widget
            if isinstance(w, (tk.Entry, tk.Text, tk.Spinbox)) or w.winfo_class() in ("Entry", "Text", "Spinbox", "TEntry"):
                return
        except Exception:
            pass
        ch = (e.char or (e.keysym if len(e.keysym or "") == 1 else "") or "").lower()
        lab = KEYS.get(ch)
        if lab and self.active and not (e.state & 0x000C):   # no Ctrl/Cmd combos
            self.mark(lab)

    def on_status(self, m):
        self.last = m
        self.active = bool(m.get("session"))
        st = "normal" if self.active else "disabled"
        for b in self.bmark.values():
            b.config(state=st)
        busy = m.get("state") in ("recording", "paused") and not self.active
        self.btog.config(text="■  Session beenden" if self.active else "⏺  Session aufnehmen",
                         state="disabled" if busy else "normal")
        c = m.get("counts") or {}
        cnt = f"✓{c.get('success', 0)} ✗{c.get('fail', 0)} ↻{c.get('play', 0)} 🗑{c.get('discard', 0)}"
        sv = f"  ⏳ speichert {m.get('saving')}" if m.get("saving") else ""
        if self.active:
            sec = m.get("take_seconds", 0.0)
            ico = "❚❚" if m.get("take_paused") else "●"
            self.info.config(text=f"{ico} Versuch {m.get('take')}  {int(sec // 60):02d}:{sec % 60:04.1f}   {cnt}{sv}",
                             fg="#d68910" if m.get("take_paused") else "#c0392b")
        else:
            self.info.config(text=f"Dauer-Aufnahme: aus   {cnt}{sv}" if (c or sv) else "Dauer-Aufnahme: aus",
                             fg="#d68910" if sv else "gray")

    def busy(self):
        return self.active or bool(self.last.get("saving"))


class VisionUI:
    """Recalibrate, detection overlay, robot-link capture. ctx: feeds, log, teleop_active(), rec (dict),
    session_active()."""

    def __init__(self, root, parent, ctx):
        self.root, self.ctx = root, ctx
        self.q = queue.Queue()
        fr = tk.LabelFrame(parent, text="Kamera-Werkzeuge (Übersichtskamera)"); fr.pack(fill="x", padx=12, pady=(4, 0))
        r1 = tk.Frame(fr); r1.pack(fill="x")
        self.bcal = tk.Button(r1, text="📐 Neu kalibrieren", command=self.recalibrate); self.bcal.pack(side="left", padx=2)
        self.blink = tk.Button(r1, text="🤝 Arm mit Tisch verbinden", command=self.robot_link)
        self.blink.pack(side="left", padx=2)
        self.det_on = tk.BooleanVar(value=False)
        tk.Checkbutton(r1, text="🔍 Erkennung", variable=self.det_on, command=self.toggle_detect).pack(side="left", padx=(10, 2))
        self.prompts = tk.StringVar(value=DEFAULT_PROMPTS)
        pe = tk.Entry(r1, textvariable=self.prompts, width=26); pe.pack(side="left", padx=2, fill="x", expand=True)
        pe.bind("<Return>", lambda _e: self._prompts_changed())
        self.msg = tk.Label(fr, text="", fg="#555", anchor="w", justify="left", wraplength=740)
        self.msg.pack(fill="x", padx=4)
        self.det = {"thread": None, "stop": threading.Event(), "prompts": None, "fps": 0.0, "n": 0,
                    "text": self.prompts.get()}
        self.prompts.trace_add("write", lambda *_a: self.det.__setitem__("text", self.prompts.get()))  # no Tk in threads
        self.link = None
        root.after(200, self._poll)
        root.after(1000, self._refresh)

    # ---- helpers ----
    def say(self, text, col="#555"):
        self.q.put(("msg", text, col))

    def _poll(self):
        try:
            while True:
                k, *a = self.q.get_nowait()
                if k == "msg":
                    self.msg.config(text=a[0], fg=a[1])
                elif k == "call":
                    a[0]()
        except queue.Empty:
            pass
        self.root.after(150, self._poll)

    def _refresh(self):
        ok = self.ctx["teleop_active"]() and not self.ctx["session_active"]() and \
            self.ctx["rec"].get("state") not in ("recording", "paused")
        self.blink.config(state="normal" if (ok or self.link) else "disabled")
        self.root.after(1000, self._refresh)

    def _frame(self):
        f = overview_feed(self.ctx["feeds"])
        if f is None:
            return None, "Keine Übersichtskamera (role 'overview') in cameras.json"
        if not f.has_signal():
            return None, f"{f.name}: kein Bild"
        with f._lock:
            bgr = f._bgr
        if bgr is None:
            return None, f"{f.name}: kein Vollbild (nur URL-Quelle?)"
        return bgr.copy(), None

    # ---- (a) recalibrate ----
    def recalibrate(self):
        frame, err = self._frame()
        if err:
            self.say("📐 " + err, "#c0392b"); return
        self.bcal.config(state="disabled"); self.say("📐 Kalibriere … (Brett flach an seinem Platz)", "#d68910")

        def work():
            try:
                cal = _vision_import("calibrate")
                r = cal.recalibrate_table(frame)
                txt = ("✓ " if r.get("ok") else "⚠ ") + str(r.get("message", ""))
                if r.get("ok"):
                    txt += "\n" + fmt_shift(r)
                for w in r.get("warnings") or []:
                    txt += "\nWarnung: " + str(w)
                self.say("📐 " + txt, "#1e8449" if r.get("ok") and not r.get("warnings") else "#c0392b" if not r.get("ok") else "#d68910")
                self.q.put(("call", lambda: self.ctx["log"](f"Neu kalibrieren: {txt}\n")))
            except Exception as e:
                self.say(f"📐 Kalibrieren fehlgeschlagen: {e}", "#c0392b")
            finally:
                self.q.put(("call", lambda: self.bcal.config(state="normal")))
        threading.Thread(target=work, daemon=True).start()

    # ---- (b) detection overlay ----
    def _prompts_changed(self):
        self.det["prompts"] = None    # worker re-reads

    def toggle_detect(self):
        f = overview_feed(self.ctx["feeds"])
        if self.det_on.get():
            if f is None:
                self.say("🔍 Keine Übersichtskamera", "#c0392b"); self.det_on.set(False); return
            self.det["stop"] = threading.Event()
            self.det["thread"] = threading.Thread(target=self._detect_loop, args=(f, self.det["stop"]), daemon=True)
            self.det["thread"].start()
            self.say("🔍 Erkennung startet (Modell laden …)", "#d68910")
        else:
            self.det["stop"].set()
            if f is not None:
                f.overlay = None
            self.say("🔍 Erkennung aus")

    def _detect_loop(self, feed, stop):
        try:
            Detector = _vision_import("detector").Detector
            det = Detector()
        except Exception as e:
            self.say(f"🔍 Detector nicht verfügbar: {e}", "#c0392b")
            self.q.put(("call", lambda: self.det_on.set(False))); return
        period = 1.0 / max(0.5, DETECT_MAX_FPS); last_seq = -1; t_fps = time.time(); n = 0
        cur = None
        while not stop.is_set():
            t0 = time.time()
            p = [x.strip() for x in str(self.det.get("text", "")).split(",") if x.strip()]
            if p != cur:
                try:
                    det.set_classes(p); cur = p
                except Exception as e:
                    self.say(f"🔍 Prompts ungültig: {e}", "#c0392b"); stop.wait(1.0); continue
            with feed._lock:
                seq, bgr = feed._seq, feed._bgr
            if bgr is None or seq == last_seq or not feed.has_signal():
                stop.wait(0.05); continue
            last_seq = seq
            try:
                dets = det.detect(bgr)
                feed.overlay = {"dets": dets, "size": (bgr.shape[1], bgr.shape[0]), "ts": time.time()}
                feed._img_seq = -1            # re-render the preview with the boxes
            except Exception as e:
                self.say(f"🔍 Fehler: {e}", "#c0392b"); stop.wait(1.0); continue
            n += 1
            if time.time() - t_fps > 2:
                fps = n / (time.time() - t_fps); n = 0; t_fps = time.time()
                top = ", ".join(f"{d[0]} {d[1]:.2f}" for d in dets[:4]) or "nichts gefunden"
                self.say(f"🔍 {fps:.1f} fps · {top}")
            stop.wait(max(0.0, period - (time.time() - t0)))
        feed.overlay = None
        try:
            del det
            import torch
            if hasattr(torch, "mps") and torch.backends.mps.is_available():
                torch.mps.empty_cache()
        except Exception:
            pass

    # ---- (c) robot link ----
    def robot_link(self):
        if self.link:
            try: self.link["win"].lift()
            except Exception: pass
            return
        if not self.ctx["teleop_active"]():
            self.say("🤝 Erst ▶ START (Teleop) – die Gelenkwinkel kommen aus der Teleop-Telemetrie", "#c0392b"); return
        if self.ctx["session_active"]() or self.ctx["rec"].get("state") in ("recording", "paused"):
            self.say("🤝 Erst die Aufnahme beenden", "#c0392b"); return
        win = tk.Toplevel(self.root); win.title("Arm mit Tisch verbinden (Schritt c)"); win.geometry("640x420")
        tk.Label(win, text="Nur lesen: es werden keine Motorbefehle gesendet. Führe die Greiferspitze mit dem "
                           "Leader-Arm auf die Ecke, halte still, dann ‚Weiter‘ (oder Enter).",
                 wraplength=600, justify="left").pack(fill="x", padx=10, pady=(10, 4))
        prompt = tk.Label(win, text="…", font=("Helvetica", 16, "bold"), fg="#1f4e79", wraplength=600, justify="left")
        prompt.pack(fill="x", padx=10, pady=4)
        out = tk.Text(win, height=10, font=("Menlo", 11)); out.pack(fill="both", expand=True, padx=10)
        bf = tk.Frame(win); bf.pack(fill="x", padx=10, pady=8)
        ev = threading.Event()
        st = {"cancel": False, "done": False}
        bnext = tk.Button(bf, text="Weiter (Enter)", font=("Helvetica", 14, "bold"), command=ev.set, state="disabled")
        bnext.pack(side="left")

        def close():
            st["cancel"] = True; ev.set()
            self.link = None
            try: win.destroy()
            except Exception: pass
        tk.Button(bf, text="Abbrechen / Schließen", command=close).pack(side="right")
        win.bind("<Return>", lambda _e: ev.set()); win.protocol("WM_DELETE_WINDOW", close)
        self.link = {"win": win}

        def ui(fn):
            self.q.put(("call", fn))

        def write(s):
            def f():
                try:
                    out.insert("end", s); out.see("end")
                except tk.TclError:
                    pass
            ui(f)

        def input_fn(text):
            ev.clear()
            ui(lambda: (prompt.config(text=text.strip()), bnext.config(state="normal")))
            while not ev.wait(0.2):
                pass
            if st["cancel"]:
                raise KeyboardInterrupt("abgebrochen")
            ui(lambda: (bnext.config(state="disabled"), prompt.config(text="lese Gelenkwinkel … (still halten)")))
            return ""

        def work():
            # Same steps as calibrate.robot_capture(), but without print()/stdout redirection (that is process-wide
            # and would swallow other threads' output): corner prompts -> read_tap_joints -> fit_robot_from_joints.
            try:
                cal = _vision_import("calibrate")
                tg = cal.corner_targets()
                try:
                    fk = _vision_import("robot_fk").B601FK()
                except Exception:
                    fk = None
                pts, js = [], []
                write("Greiferspitze nacheinander auf die Brett-Ecken A, B, C, D setzen (innere Ecke am Brettrand).\n")
                for c in ("A", "B", "C", "D"):
                    x, y = tg[c]
                    input_fn(f"Ecke {c} = Tisch ({x:.0f}, {y:.0f}) mm: Spitze hinführen, still halten, dann Weiter")
                    q = cal.read_tap_joints(port=FWD_PORT)
                    write(f"Ecke {c}: joints {[round(v, 1) for v in q]}\n")
                    if fk is not None:
                        bad = fk.check_limits(q)
                        if bad:
                            write(f"  WARNUNG außerhalb der URDF-Grenzen: {bad}\n")
                    pts.append((x, y)); js.append(q)
                ui(lambda: prompt.config(text="rechne …"))
                r = cal.fit_robot_from_joints(pts, js)
                write(f"Fit: {r['rms_mm']:.1f} mm RMS, Höhenstreuung {r['z_std_mm']:.1f} mm, pro Punkt "
                      f"{r.get('per_point_error_mm')} mm, gespiegelt={r.get('mirrored')}, Vorzeichen={r.get('signs')}\n")
                good = r["rms_mm"] < 5 and r["z_std_mm"] < 5
                warn = r["rms_mm"] > 10 or r["z_std_mm"] > 10
                if warn:
                    write(f"WARNUNG > 10 mm: Spitzen-Offset (HUMANOID_TIP_OFFSET_MM), Ecken genau getroffen?, "
                          f"Vorzeichen. Alternativen: {r.get('alternatives')}\n")
                txt = (f"Fertig: Restfehler {r['rms_mm']:.1f} mm RMS, Höhenstreuung {r['z_std_mm']:.1f} mm "
                       f"({'gut' if good else 'zu groß – Spitze/Vorzeichen prüfen' if warn else 'brauchbar'})")
                ui(lambda: prompt.config(text=txt, fg="#1e8449" if good else "#c0392b" if warn else "#d68910"))
                self.say("🤝 " + txt, "#1e8449" if good else "#c0392b" if warn else "#d68910")
                ui(lambda: self.ctx["log"](f"Arm mit Tisch verbunden: {txt}\n"))
            except KeyboardInterrupt:
                pass
            except Exception as e:
                write(f"\nFEHLER: {e}\n")
                ui(lambda: prompt.config(text=f"Fehler: {e}", fg="#c0392b"))
            finally:
                st["done"] = True
        threading.Thread(target=work, daemon=True).start()


def build(root, rec_parent, ctx, parent=None):
    """Create both add-ons (vision tools go into parent, default root). Returns (SessionUI, VisionUI)."""
    s = SessionUI(root, rec_parent, ctx)
    ctx.setdefault("session_active", s.busy)
    v = VisionUI(root, parent if parent is not None else root, ctx)
    return s, v
