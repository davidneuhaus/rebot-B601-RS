#!/usr/bin/env python3
"""GUI 3.9 add-on for teleop_gui.py: automatic hand-eye calibration ("🤖 Auto-Kalibrierung"). Labels German.

Flow (docs/OPERATING_PROCEDURE.md "Auto calibration"):
 v3.9.1: the taught pose is OPTIONAL. Default = automatic start: the arm moves slowly (6 deg/s) from wherever it is to
    a high search pose, finds the board in the wrist camera, moves 200 mm above the board centre and plans the sweep
    from there (`handeye.sh run --yes --auto`). "📍 Ziel-Pose übernehmen" + the tick "Gespeicherte Ziel-Pose
    verwenden" is the manual override (old flow below).
 1. "📍 Ziel-Pose übernehmen" (only while teleop runs): David aims the wrist camera at the taped ChArUco board with
    the leader and holds still; the joints are read from the telemetry that recorder.py forwards to 127.0.0.1:8766
    (read-only, no motor command) -> vision/calibration/handeye_teach.json.
 2. "🤖 Auto-Kalibrierung …" (only while teleop, policy and recording are stopped): plans the poses
    (`handeye.py plan`, no hardware, ~40 s) and opens a confirmation window with the pose list + plot.
    Nothing moves until all three checks are ticked and "▶ Jetzt bewegen" is clicked.
 3. The run is `handeye.sh run --yes` in its own process group. "■ STOPP Auto-Kalibrierung" (or Esc) = SIGTERM ->
    motors off immediately; if the process is not gone after 1.5 s it is killed and disable_motors.py runs.
 4. At the end the result (quality, mm / deg numbers) is shown; a GOOD/OK result is applied automatically.
"""
import os, queue, signal, subprocess, sys, threading, time, tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
VISION_DIR = os.environ.get("HUMANOID_VISION_DIR", os.path.join(HERE, "vision"))
FWD_PORT = int(os.environ.get("TELEMETRY_FWD_PORT", "8766"))
HANDEYE_SH = os.path.join(HERE, "handeye.sh")
EXTRA = os.environ.get("HANDEYE_GUI_EXTRA_ARGS", "").split()      # test hook only, e.g. "--sim"


def _vision():
    if VISION_DIR not in sys.path:
        sys.path.insert(0, VISION_DIR)
    import handeye, handeye_plan
    return handeye, handeye_plan


class AutoCalUI:
    """ctx: log(str), teleop_active() -> bool, busy() -> str|None (reason the arm is in use: teleop/policy/recording)."""

    def __init__(self, root, parent, ctx):
        self.root, self.ctx = root, ctx
        self.proc, self.stopping, self.q = None, False, queue.Queue()
        self.planning = False; self.dlg = None; self.result_lines = []
        fr = tk.LabelFrame(parent, text="🤖 Auto-Kalibrierung (Hand-Auge: Greiferkamera, Arm, Tisch, Übersichtskamera)")
        fr.pack(fill="x", padx=12, pady=(4, 0))
        r1 = tk.Frame(fr); r1.pack(fill="x")
        self.bteach = tk.Button(r1, text="📍 Ziel-Pose übernehmen", command=self.teach)
        self.bteach.pack(side="left", padx=2)
        self.brun = tk.Button(r1, text="🤖 Auto-Kalibrierung …", font=("Helvetica", 13, "bold"), command=self.plan)
        self.brun.pack(side="left", padx=2)
        self.bstop = tk.Button(r1, text="■ STOPP Auto-Kalibrierung", font=("Helvetica", 13, "bold"), fg="#c0392b",
                               state="disabled", command=self.stop)
        self.bstop.pack(side="left", padx=2)
        self.bapply = tk.Button(r1, text="✓ Übernehmen", command=self.apply_partial, state="disabled")
        self.bapply.pack(side="left", padx=2)            # v3.9.2: apply an OK result of an aborted / partial run
        self.apply_folder = None; self.motors_off = False
        self.use_teach = tk.BooleanVar(value=False)         # v3.9.1: default automatic start
        self.cteach = tk.Checkbutton(r1, text="Gespeicherte Ziel-Pose verwenden", variable=self.use_teach,
                                     command=lambda: self._set(self._idle_text()))
        self.cteach.pack(side="left", padx=6)
        self.mode_auto = True
        self.msg = tk.Label(fr, text=self._idle_text(), fg="#555", anchor="w", justify="left", wraplength=760)
        self.msg.pack(fill="x")
        root.bind_all("<Escape>", lambda e: self.stop() if self.running() else None, add="+")
        root.after(500, self._poll)

    # ---------------------------------------------------------------- state
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def busy(self):
        return self.running() or self.planning

    def _idle_text(self):
        try:
            he, hp = _vision()
            t = he.load_teach() if hp.TEACH.exists() else None
        except Exception:
            t = None
        if t is None or not self.use_teach.get():
            return ("Automatischer Start: Teleop aus, Arm in Ruhelage (oder grob über dem Brett), Brett frei sichtbar → "
                    "🤖 Auto-Kalibrierung … Der Arm fährt langsam (6°/s) in eine hohe Suchpose, sucht das Brett mit der "
                    "Greiferkamera und richtet sich selbst darüber aus. Optional: 📍 Ziel-Pose übernehmen (bei laufendem "
                    "Teleop) und 'Gespeicherte Ziel-Pose verwenden' anhaken.")
        return (f"Ziel-Pose vom {t['time']} gespeichert ({t['height_over_table_mm']:.0f} mm über dem Brett). "
                "Teleop STOPP, dann 🤖 Auto-Kalibrierung …")

    def _set(self, text, color="#555"):
        self.msg.config(text=text, fg=color)

    def refresh(self):
        tele = self.ctx["teleop_active"]()
        self.bteach.config(state="normal" if tele and not self.busy() else "disabled")
        self.brun.config(state="disabled" if (self.busy() or tele) else "normal")
        try:
            he, hp = _vision()
            has_teach = hp.TEACH.exists()
        except Exception:
            has_teach = False
        self.cteach.config(state="normal" if has_teach and not self.busy() else "disabled")
        self.bstop.config(state="normal" if self.running() and not self.stopping else "disabled")
        self.bapply.config(state="normal" if self.apply_folder and not self.busy() else "disabled")

    # ---------------------------------------------------------------- 1. teach (read-only)
    def teach(self):
        if not self.ctx["teleop_active"]():
            self._set("Erst ▶ START (Teleop) – die Pose wird aus den Teleop-Daten gelesen.", "#c0392b"); return
        self._set("◐ Lese Gelenkwinkel – Arm ruhig halten …", "#d68910")
        self.bteach.config(state="disabled")
        def work():
            try:
                he, _ = _vision()
                d = he.teach(port=FWD_PORT)
                self.q.put(("teach_ok", None, None))      # tick 'Gespeicherte Ziel-Pose verwenden' (Tk thread)
                self.q.put(("msg", f"✓ Ziel-Pose übernommen: Zielpunkt {[round(v) for v in d['look_point_mm']]} mm, "
                                   f"Greifer {d['height_over_table_mm']:.0f} mm über dem Brett. Jetzt ■ STOPP, dann "
                                   "🤖 Auto-Kalibrierung …", "#1a7a32"))
            except Exception as e:
                self.q.put(("msg", f"Ziel-Pose nicht übernommen: {e}", "#c0392b"))
        threading.Thread(target=work, daemon=True).start()

    # ---------------------------------------------------------------- 2. plan + confirm
    def plan(self):
        reason = self.ctx["busy"]()
        if reason:
            self._set(f"Geht gerade nicht: {reason}", "#c0392b"); return
        if not self.use_teach.get():
            return self.plan_auto()
        try:
            he, hp = _vision()
            he.load_teach()
        except Exception as e:
            self._set(str(e), "#c0392b"); return
        self.planning = True; self.refresh()
        self._set("◐ Plane die Posen (Reichweite, Gelenkgrenzen, Mindesthöhe, Wege) … ca. 40 s, nichts bewegt sich", "#d68910")
        def work():
            try:
                r = subprocess.run([sys.executable, os.path.join(VISION_DIR, "handeye.py"), "plan"],
                                   capture_output=True, text=True, timeout=300, cwd=HERE)
                if r.returncode != 0:
                    self.q.put(("msg", "Planung fehlgeschlagen: " + ((r.stdout + r.stderr).strip().splitlines() or ["?"])[-1], "#c0392b"))
                else:
                    self.q.put(("plan", None, None))
            except Exception as e:
                self.q.put(("msg", f"Planung fehlgeschlagen: {e}", "#c0392b"))
            finally:
                self.planning = False
        threading.Thread(target=work, daemon=True).start()

    def plan_auto(self):
        """v3.9.1 automatic start: offline check of the search pose (~1 s, no hardware), then the confirmation window."""
        self.planning = True; self.refresh()
        self._set("◐ Automatischer Start: prüfe Suchpose und Weg aus der Ruhelage … (nichts bewegt sich)", "#d68910")
        def work():
            try:
                r = subprocess.run([sys.executable, os.path.join(VISION_DIR, "handeye.py"), "plan", "--auto"],
                                   capture_output=True, text=True, timeout=120, cwd=HERE)
                if r.returncode != 0:
                    self.q.put(("msg", "Suchpose nicht möglich: " + ((r.stdout + r.stderr).strip().splitlines() or ["?"])[-1], "#c0392b"))
                else:
                    self.q.put(("auto", None, None))
            except Exception as e:
                self.q.put(("msg", f"Suchpose nicht möglich: {e}", "#c0392b"))
            finally:
                self.planning = False
        threading.Thread(target=work, daemon=True).start()

    def confirm_dialog(self, auto=False):
        import json
        he, hp = _vision()
        self.mode_auto = auto
        d = self.dlg = tk.Toplevel(self.root)
        d.transient(self.root)
        if auto:
            a = json.loads(open(he.AUTO_JSON).read())
            sp = a["search_pose"]
            d.title("Auto-Kalibrierung – automatischer Start")
            tk.Label(d, text="Der Arm fährt gleich SELBSTÄNDIG (langsam, Kollisionsschutz AN).",
                     font=("Helvetica", 14, "bold"), fg="#c0392b").pack(padx=10, pady=(8, 2), anchor="w")
            info = (f"1. Von der aktuellen Position langsam (6°/s) in die Suchpose: Greifer {sp['height_mm']:.0f} mm über "
                    f"dem erwarteten Brettpunkt {[round(v) for v in a['guess_mm']]} mm ({a['guess_source']}), "
                    f"Gelenke {[round(v, 1) for v in sp['q']]}.\n"
                    "2. Brett in der Greiferkamera suchen (mind. 10 Ecken), sonst Abbruch 'Brett nicht gefunden'.\n"
                    "3. 200 mm über die Brettmitte fahren, nachmessen, dann 18 Posen planen (~40 s, Arm hält still; "
                    "gleiche Prüfungen: Gelenkgrenzen, Mindesthöhe 80 mm, alle Wege).\n"
                    "4. Posen abfahren (max 12°/s), zurück, Ruheposition, Motoren aus.")
            tk.Label(d, text=info, justify="left", anchor="w", wraplength=760).pack(padx=10, pady=4, anchor="w")
        else:
            plan = json.loads(open(he.PLAN_JSON).read())
            d.title("Auto-Kalibrierung – Posen prüfen")
            tk.Label(d, text="Der Arm fährt gleich SELBSTÄNDIG diese Posen ab (langsam, Kollisionsschutz AN).",
                     font=("Helvetica", 14, "bold"), fg="#c0392b").pack(padx=10, pady=(8, 2), anchor="w")
            try:
                self._img = tk.PhotoImage(file=str(he.PLAN_PNG))
                tk.Label(d, image=self._img).pack(padx=10)
            except Exception:
                pass
            txt = tk.Text(d, height=14, width=110, font=("Menlo", 10))
            txt.insert("1.0", hp.summary_text(plan)); txt.config(state="disabled"); txt.pack(padx=10, pady=4, fill="x")
        self.chk = [tk.BooleanVar(value=False) for _ in range(3)]
        labels = ["Brett liegt flach und ist festgeklebt (es darf sich nicht bewegen)",
                  "Arbeitsbereich über dem Brett ist frei (keine Hände, Kabel, Gegenstände)",
                  "Ich bin am Arm: Hand am ■ STOPP (oder Esc) und an der Steckdosenleiste"]
        for v, t in zip(self.chk, labels):
            tk.Checkbutton(d, text=t, variable=v, command=self._chk_changed).pack(anchor="w", padx=10)
        br = tk.Frame(d); br.pack(fill="x", padx=10, pady=8)
        tk.Button(br, text="Abbrechen", command=d.destroy).pack(side="left")
        self.bgo = tk.Button(br, text="▶ Jetzt bewegen", font=("Helvetica", 14, "bold"), fg="#1a7a32", state="disabled",
                             command=self.go)
        self.bgo.pack(side="right")
        tk.Label(d, text="■ STOPP = Motoren sofort aus (der Arm sackt ab, er ist ≥ 80 mm über dem Brett). "
                         "Am Ende fährt er in die Ruheposition. Antwortet der Arm nicht oder folgt er nicht, werden nur "
                         "die Motoren ausgeschaltet (keine weitere Bewegung).", fg="#555", wraplength=760,
                 justify="left").pack(padx=10, pady=(0, 8), anchor="w")

    def _chk_changed(self):
        self.bgo.config(state="normal" if all(v.get() for v in self.chk) else "disabled")

    # ---------------------------------------------------------------- 3. run / stop
    def go(self):
        if self.dlg is not None:
            self.dlg.destroy(); self.dlg = None
        reason = self.ctx["busy"]()
        if reason:
            self._set(f"Geht gerade nicht: {reason}", "#c0392b"); return
        self.ctx["log"]("\n=== Auto-Kalibrierung startet – ■ STOPP Auto-Kalibrierung / Esc = Motoren sofort aus ===\n")
        self.stopping = False; self.result_lines = []; self.apply_folder = None; self.motors_off = False
        self.proc = subprocess.Popen(["/bin/bash", HANDEYE_SH, "run", "--yes"] + (["--auto"] if self.mode_auto else []) + EXTRA, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True, cwd=HERE)
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
        self._set("◐ Auto-Kalibrierung startet (Kameras, Arm verbinden) …", "#d68910")
        self.refresh()

    def _reader(self, p):
        for line in iter(p.stdout.readline, ""):
            self.q.put(("line", line, None))

    def stop(self):
        if not self.running() or self.stopping:
            return
        self.stopping = True
        self.ctx["log"]("\n=== Auto-Kalibrierung STOPP – Motoren sofort aus ===\n")
        self._set("■ STOPP – Motoren aus …", "#c0392b")
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
        except Exception:
            pass
        self.root.after(1500, self._force, self.proc)
        self.refresh()

    def apply_partial(self):
        """v3.9.2: write the result of the last (aborted) run into handeye.json + table.json (no hardware)."""
        folder = self.apply_folder
        if not folder or self.busy():
            return
        self._set("◐ Übernehme das Ergebnis …", "#d68910"); self.bapply.config(state="disabled")
        def work():
            try:
                r = subprocess.run([sys.executable, os.path.join(VISION_DIR, "handeye.py"), "apply", folder],
                                   capture_output=True, text=True, timeout=120, cwd=HERE)
                last = ((r.stdout + r.stderr).strip().splitlines() or ["?"])[-1]
                if r.returncode == 0:
                    self.apply_folder = None
                    self.q.put(("msg", "✓ übernommen (handeye.json + table.json). Jetzt 📐 Neu kalibrieren drücken.", "#1a7a32"))
                else:
                    self.q.put(("msg", f"Übernehmen fehlgeschlagen: {last}", "#c0392b"))
            except Exception as e:
                self.q.put(("msg", f"Übernehmen fehlgeschlagen: {e}", "#c0392b"))
        threading.Thread(target=work, daemon=True).start()

    def _force(self, p):
        if p.poll() is None and self.motors_off:
            # v3.9.2: motors are already off; the process is solving the partial sweep - let it finish
            self.ctx["log"]("Motoren aus – werte die bisherigen Posen noch aus (erneut STOPP = abbrechen)\n")
            self.stopping = False
            return
        if p.poll() is None:
            self.ctx["log"]("Auto-Kalibrierung reagiert nicht – erzwinge Beenden und schalte Motoren aus\n")
            try: os.killpg(p.pid, signal.SIGKILL)
            except Exception: pass
            def _off():
                try:
                    r = subprocess.run([sys.executable, os.path.join(HERE, "disable_motors.py")], capture_output=True,
                                       text=True, timeout=20)
                    self.q.put(("line", (((r.stdout + r.stderr).strip().splitlines()[-1:]) or ["?"])[0] + "\n", None))
                except Exception as e:
                    self.q.put(("line", f"Motoren-Aus fehlgeschlagen: {e}\n", None))
            threading.Thread(target=_off, daemon=True).start()

    # ---------------------------------------------------------------- poll
    def _poll(self):
        try:
            while True:
                kind, a, b = self.q.get_nowait()
                if kind == "msg":
                    self._set(a, b)
                elif kind == "plan":
                    self._set("Posen geplant – bitte im Fenster prüfen und bestätigen.", "#d68910")
                    self.confirm_dialog()
                elif kind == "teach_ok":
                    self.use_teach.set(True)
                elif kind == "auto":
                    self._set("Automatischer Start geprüft – bitte im Fenster bestätigen.", "#d68910")
                    self.confirm_dialog(auto=True)
                elif kind == "line":
                    self.ctx["log"](a)
                    s = a.split("] ", 1)[-1].strip()
                    if s.startswith(("STOP: motors off", "motors off (no further", "motors off: no answer",
                                     "rest reached, motors off")) or (s.startswith("rest move failed") and "motors off" in s):
                        self.motors_off = True
                    if s.startswith("apply available: "):
                        self.apply_folder = s[len("apply available: "):].strip()
                    if s.startswith(("HINWEIS", "resume possible", "not applied", "aborted run")):
                        self.result_lines.append(s)
                    if s.startswith(("pose ", "RUNNING", "start:", "board seen", "moving to", "start pose found")):
                        self._set("● LÄUFT: " + s[:110] + "   (■ STOPP / Esc = Motoren sofort aus)", "#1a9c3a")
                    elif s.startswith(("ABORT", "ERROR", "refused", "quality POOR")):
                        self._set("⚠ " + s[:140], "#c0392b")
                    elif s.startswith("Hand-eye result") or s.startswith("  board pose spread") or s.startswith("  chain repro") \
                            or s.startswith("  board centre"):
                        self.result_lines.append(s)
                    elif s.startswith("applied"):
                        self.result_lines.append("✓ übernommen (handeye.json + table.json)")
        except queue.Empty:
            pass
        p = self.proc
        if p is not None and p.poll() is not None:
            rc = p.returncode; self.proc = None
            self.ctx["log"](f"=== Auto-Kalibrierung beendet (Code {rc}) ===\n")
            if self.result_lines:
                res = [l for l in self.result_lines if l.startswith("Hand-eye result")]
                good = bool(res) and ("GOOD" in res[0] or "OK" in res[0])
                txt = "\n".join(self.result_lines)
                if self.apply_folder:
                    txt += "\n→ Ergebnis nicht automatisch übernommen: prüfen, dann ✓ Übernehmen"
                self._set(txt, "#1a7a32" if good and rc == 0 else ("#d68910" if good else "#c0392b"))
            elif rc != 0 and not str(self.msg.cget("text")).startswith("⚠"):
                self._set(f"⚠ Auto-Kalibrierung beendet (Code {rc}) – siehe Log / logs/handeye_*.log", "#c0392b")
            elif rc == 0:
                self._set("Auto-Kalibrierung beendet.", "#555")
        self.refresh()
        self.root.after(200, self._poll)


def build(root, parent, ctx):
    return AutoCalUI(root, parent, ctx)
