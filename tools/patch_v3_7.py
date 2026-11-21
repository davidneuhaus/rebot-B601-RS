"""Guard v3.3 / app 3.7 patch (anchor-based, idempotent). Usage: python patch_v3_7.py <dir with the 5 files>"""
import sys, os
D = sys.argv[1]
def rep(fn, old, new, count=1):
    p = os.path.join(D, fn); s = open(p).read()
    if new in s:
        print(f"  {fn}: schon gepatcht ({new[:40]!r})"); return
    n = s.count(old)
    assert n == count, f"{fn}: anchor found {n}x, expected {count}: {old[:80]!r}"
    open(p, "w").write(s.replace(old, new)); print(f"  {fn}: ok")

F = "seeed_b601_follower.py"
# ---------- header comment ----------
rep(F, """# GRAVITY_FF_SCALE (default 0.8) * g(q) as MIT torque feedforward (default off).
# ---------------------------------------------------------------------------""",
"""# GRAVITY_FF_SCALE (default 0.8) * g(q) as MIT torque feedforward (default off).
# guard v3.3: (a) gravity-explained stall: for COLLISION_GRAV_RES_JOINTS (default shoulder_lift,
# elbow_flex; teleop.sh adds wrist_flex) a lag AGAINST gravity is not a collision when the joint pushes less
# than COLLISION_GRAV_RES_<JOINT> / COLLISION_GRAV_RES_NM (default 4.0; teleop.sh: wrist_flex 1.5) beyond the
# expected gravity torque in the direction of motion
# (measured torque minus g). That is exactly the force such a stall can put on an obstacle, so a
# lift that is only too weak for the load (11 Nm gravity vs 13 Nm kp*5deg) no longer freezes, while a
# real block with feedforward on (push ~ kp*5deg - 0.2*g >= 10 Nm) still does. (b) GRAVITY_FF feedforward
# ramps in over GRAVITY_FF_RAMP_S (default 1 s) and is capped tighter (lift 14, elbow 9, wrist_flex 3,
# rotary joints 1 Nm; logged max model torque today: lift 11.4, elbow 7.1, wrist_flex 2.4 Nm).
# teleop.sh v3.7 turns GRAVITY_FF=1 on by default.
# ---------------------------------------------------------------------------""")
# ---------- caps ----------
rep(F, """GRAVITY_FF_MAX_NM = {"shoulder_pan": 18.0, "shoulder_lift": 18.0, "elbow_flex": 12.0,
                     "wrist_flex": 4.0, "wrist_yaw": 4.0, "wrist_roll": 4.0}""",
"""# guard v3.3: tighter caps (model max in today's logs: lift 11.4, elbow 7.1, wrist_flex 2.4 Nm; the rotary
# joints have ~0 gravity torque, so anything above 1 Nm there would be a model error).
GRAVITY_FF_MAX_NM = {"shoulder_pan": 1.0, "shoulder_lift": 14.0, "elbow_flex": 9.0,
                     "wrist_flex": 3.0, "wrist_yaw": 1.0, "wrist_roll": 1.0}


def gravity_ff_ramp_s() -> float:
    \"\"\"guard v3.3: seconds over which the gravity feedforward ramps from 0 to full after connect.\"\"\"
    return max(0.0, _env_float("GRAVITY_FF_RAMP_S", 1.0))


_GRAV_RES_JOINTS_DEFAULT = ("shoulder_lift", "elbow_flex")


def _grav_res_joints_env():
    import os as _os
    v = _os.environ.get("COLLISION_GRAV_RES_JOINTS", "")
    return tuple(x.strip() for x in v.split(",") if x.strip()) if v.strip() else _GRAV_RES_JOINTS_DEFAULT""")
# ---------- class docstring ----------
rep(F, '''    """Detect blocked arm joints during teleop and freeze the arm (v3.2, gravity-aware)."""''',
'''    """Detect blocked arm joints during teleop and freeze the arm (v3.3, gravity-aware)."""''')
# ---------- __init__ params ----------
rep(F, """                 kp=None, clamp_deg=None, ff_scale=0.0, grav_lag_gain=1.5, grav_sat_frac=0.85,
                 grav_min_nm=0.3):
        self.joints = list(joints)""",
"""                 kp=None, clamp_deg=None, ff_scale=0.0, grav_lag_gain=1.5, grav_sat_frac=0.85,
                 grav_min_nm=0.3, grav_res_nm=4.0, grav_res_joints=_GRAV_RES_JOINTS_DEFAULT):
        self.joints = list(joints)
        # v3.3 gravity-explained stall (see header): push beyond g below grav_res_nm -> no collision
        if isinstance(grav_res_nm, dict):
            self.grav_res_nm = {j: float(grav_res_nm.get(j, 4.0)) for j in self.joints}
        else:
            self.grav_res_nm = {j: float(grav_res_nm) for j in self.joints}
        self.grav_res_joints = tuple(grav_res_joints or ())
        self.n_grav_explained = 0
        self._grav_expl_logged = set()""")
# ---------- from_env ----------
rep(F, """            grav_sat_frac=_env_float("COLLISION_GRAV_SAT_FRAC", 0.85),
            log=log,""",
"""            grav_sat_frac=_env_float("COLLISION_GRAV_SAT_FRAC", 0.85),
            grav_res_nm={j: _env_float(f"COLLISION_GRAV_RES_{j.upper()}", _env_float("COLLISION_GRAV_RES_NM", 4.0)) for j in joints},
            grav_res_joints=_grav_res_joints_env(),
            log=log,""")
# ---------- describe ----------
rep(F, """                   f"{self.grav_lag_gain:g}x g/kp, Feedforward-Anteil {self.ff_scale:g}")""",
"""                   f"{self.grav_lag_gain:g}x g/kp, Feedforward-Anteil {self.ff_scale:g}")
            if self.grav_res_joints:
                ar += ("; v3.3 Schwerkraft-Stillstand: kein Alarm, solange das Gelenk weniger als ["
                       + ", ".join(f"{j}={self.grav_res_nm.get(j, 0.0):g}" for j in self.grav_res_joints if j in self.grav_res_nm)
                       + "] Nm ueber der Schwerkraft drueckt")""")
# ---------- helper after _tq_hit ----------
rep(F, """    def _freeze(self, present, now, j, disc, trk, tq, progress, why):""",
"""    def _grav_explained(self, j, disc, tq):
        \"\"\"v3.3: True when a lag AGAINST gravity is explained by gravity alone: the joint pushes less than
        grav_res_nm beyond the expected gravity torque g in the direction of the lag. That residual push is
        the most such a stall can press on an obstacle, so freezing would add nothing (and with gravity
        feedforward on, a real block shows a push of ~kp*clamp, far above grav_res_nm).\"\"\"
        if (self.grav_res_nm.get(j, 0.0) <= 0 or j not in self.grav_res_joints or tq is None or j not in self.grav
                or self.kp.get(j, 0.0) <= 0):
            return False
        g = self.grav[j]
        if abs(g) < self.grav_min_nm or disc == 0 or (disc > 0) != (g > 0):
            return False
        push = (tq - g) * (1.0 if disc > 0 else -1.0)
        return push < self.grav_res_nm[j]

    def _note_grav_explained(self, j, disc, tq):
        self.n_grav_explained += 1
        if j not in self._grav_expl_logged:
            self._grav_expl_logged.add(j)
            g = self.grav.get(j, 0.0)
            self.log.warning("Schwerkraft: %s haengt %.1f deg hinterher, drueckt aber nur %.1f Nm ueber der Schwerkraft "
                             "(g=%.1f Nm, gemessen %.1f Nm) -> keine Kollision (v3.3)", JOINT_DE.get(j, j), abs(disc),
                             (tq - g) * (1.0 if disc > 0 else -1.0), g, tq)

    def _freeze(self, present, now, j, disc, trk, tq, progress, why):""")
# ---------- update(): normal path ----------
rep(F, """            extra, sat = self._grav_tol(j, disc)
            blocked = abs(disc) > self.err_deg[j] + extra and progress < self.min_progress_deg and not sat
            if sat and abs(disc) > self.err_deg[j]:
                self._note_sat(j, self._grav_unc(j))""",
"""            extra, sat = self._grav_tol(j, disc)
            blocked = abs(disc) > self.err_deg[j] + extra and progress < self.min_progress_deg and not sat
            if sat and abs(disc) > self.err_deg[j]:
                self._note_sat(j, self._grav_unc(j))
            if blocked and self._grav_explained(j, disc, tq):     # v3.3
                blocked = False
                self._note_grav_explained(j, disc, tq)""")
# ---------- creep path ----------
rep(F, """            extra, sat = self._grav_tol(j, e)
            blocked = lag >= self.creep_lag[j] + extra and progress < self.min_progress_deg and not sat""",
"""            extra, sat = self._grav_tol(j, e)
            blocked = lag >= self.creep_lag[j] + extra and progress < self.min_progress_deg and not sat
            if blocked and self._grav_explained(j, e, tq):        # v3.3
                blocked = False
                self._note_grav_explained(j, e, tq)""")
# ---------- FF ramp in send_action ----------
rep(F, """                _sc = gravity_ff_scale()
                gravity_tau = {n: max(-GRAVITY_FF_MAX_NM.get(n, 4.0), min(GRAVITY_FF_MAX_NM.get(n, 4.0), _sc * g))
                               for n, g in (_grav_cycle or {}).items()}""",
"""                _sc = gravity_ff_scale()
                # v3.3: ramp in over GRAVITY_FF_RAMP_S after the first feedforward cycle (no torque step at start)
                _now = time.monotonic()
                if getattr(self, "_grav_ff_t0", None) is None and _grav_cycle:
                    self._grav_ff_t0 = _now
                _rs = gravity_ff_ramp_s()
                _ramp = 1.0 if _rs <= 0 or getattr(self, "_grav_ff_t0", None) is None else min(1.0, (_now - self._grav_ff_t0) / _rs)
                gravity_tau = {n: max(-GRAVITY_FF_MAX_NM.get(n, 1.0), min(GRAVITY_FF_MAX_NM.get(n, 1.0), _ramp * _sc * g))
                               for n, g in (_grav_cycle or {}).items()}""")
rep(F, """                    logger.info("Schwerkraft-Feedforward AN (Anteil %.2f): %s", _sc,
                                ", ".join(f"{n}={v:.2f}" for n, v in gravity_tau.items()) or "kein Modell")""",
"""                    logger.info("Schwerkraft-Feedforward AN (Anteil %.2f, Rampe %.1f s, Grenzen %s): Start-Werte %s", _sc, _rs,
                                ", ".join(f"{n}={v:g}" for n, v in GRAVITY_FF_MAX_NM.items()),
                                ", ".join(f"{n}={v:.2f}" for n, v in gravity_tau.items()) or "kein Modell")""")

# ======================= rs_follower: gripper open-stall =======================
R = "seeed_b601_rs_follower.py"
rep(R, """import logging

from .config""", """import logging
import math

from .config""")
rep(R, """        if self._gripper_open_stall_count >= 20:  # PATCH: was 5
            logger.warning("gripper open-stall detected, cutting torque")
            motor.request_feedback()
            return 0.0
""", """        # PATCH v3.7 (Humanoid Project): the cut-off latched mid-stroke. In 'vorsichtig' (1.5 Nm opening) the
        # gripper stalled at 75-175 deg (open stop ~270 deg) and, once cut, never got torque again while the leader
        # stayed open (1276 'open-stall' lines in session 13:43). Now the cut only happens near the open stop
        # (within GRIPPER_OPEN_STALL_ZONE_DEG, default 35 deg, of joint_limits max); a mid-stroke stall keeps the
        # opening torque, and the warning is logged once per stall instead of every cycle.
        if self._gripper_open_stall_count >= 20:  # PATCH: was 5
            try:
                import os as _os
                _zone = float(_os.environ.get("GRIPPER_OPEN_STALL_ZONE_DEG", "35") or 35)
            except ValueError:
                _zone = 35.0
            _open_max = float(getattr(self.config, "joint_limits", {}).get("gripper", (0.0, 270.0))[1])
            near_open = math.degrees(state.pos) >= _open_max - _zone
            if near_open:
                if self._gripper_open_stall_count == 20:
                    logger.warning("gripper open-stall at the open stop (%.0f deg), cutting torque", math.degrees(state.pos))
                motor.request_feedback()
                return 0.0
            if self._gripper_open_stall_count == 20:
                logger.warning("gripper opening stalls mid-stroke at %.0f deg (open stop ~%.0f deg) - keeping the opening "
                               "torque (limit %.1f Nm), no cut-off", math.degrees(state.pos), _open_max,
                               float(self.config.gripper_mit_torque_limit))
""")
print("fertig")

# ======================= teleop.sh =======================
T = "teleop.sh"
rep(T, """  vorsichtig) KP=10.0; TQ=1.5; HOLD=1.0;  CLOSE=1.1 ;;   # soft, for cardboard/fragile items
  mittel)     KP=12.0; TQ=2.0; HOLD=1.2;  CLOSE=1.2 ;;""",
"""  # v3.7: TQ (opening limit) 1.5/2.0 -> 3.5 Nm: at 1.5 Nm the gripper stalled half open (75-175 of ~270 deg,
  # session 13:43); at 3.5 Nm ('kraeftig', session 13:52) it opened to ~245-270 deg. Closing stays capped by CLOSE.
  vorsichtig) KP=10.0; TQ=3.5; HOLD=1.0;  CLOSE=1.1 ;;   # soft closing, for cardboard/fragile items
  mittel)     KP=12.0; TQ=3.5; HOLD=1.2;  CLOSE=1.2 ;;""")
rep(T, """export COLLISION_TORQ_WRIST_YAW=${COLLISION_TORQ_WRIST_YAW:-1.25}        # v3.0: 1.0 – false alarms at 1.00–1.14 Nm""",
"""# v3.7: wrist_yaw 1.25 -> 2.4 Nm: 11 false freezes on 2026-11-21 at 1.37-2.10 Nm during normal moves (max 2.16 Nm in
# the 5 s stats). At kp 20 a static block can only press kp*5deg = 1.75 Nm, so a held wrist is now caught by the
# lag trigger (COLLISION_ERR_WRIST_YAW=8 deg, 6 cycles); the torque trigger only catches hard dynamic hits.
export COLLISION_TORQ_WRIST_YAW=${COLLISION_TORQ_WRIST_YAW:-2.4}         # v3.0: 1.0, v3.1-v3.6: 1.25""")
rep(T, """# GRAVITY_FF=1 adds GRAVITY_FF_SCALE * g(q) as MIT torque feedforward (default OFF, still being tested).""",
"""# GRAVITY_FF=1 adds GRAVITY_FF_SCALE * g(q) as MIT torque feedforward. v3.7: ON by default (ramped in over 1 s, capped
# per joint: lift 14, elbow 9, wrist_flex 3, rotary 1 Nm). Without it, lifting overhead needs up to 11.4 Nm from the
# P-term alone, which kp 150 can only just deliver inside the 5 deg step limit (13.1 Nm) -> the arm stalled.
# v3.7 (guard v3.3): COLLISION_GRAV_RES_NM - a lag against gravity on shoulder_lift/elbow_flex/wrist_flex only counts
# as a collision when the joint pushes more than this beyond the expected gravity torque (false freezes today:
# shoulder_lift 1.57/1.72 Nm, wrist_flex 1.38 Nm). With GRAVITY_FF on, a real block pushes ~kp*5deg - 0.2*g
# (lift/elbow >= 10 Nm, wrist_flex ~3 Nm), so it is still detected.""")
rep(T, """export GRAVITY_FF=${GRAVITY_FF:-0}
export GRAVITY_FF_SCALE=${GRAVITY_FF_SCALE:-0.8}""",
"""export GRAVITY_FF=${GRAVITY_FF:-1}
export GRAVITY_FF_SCALE=${GRAVITY_FF_SCALE:-0.8}
export GRAVITY_FF_RAMP_S=${GRAVITY_FF_RAMP_S:-1.0}
export COLLISION_GRAV_RES_NM=${COLLISION_GRAV_RES_NM:-4.0}                 # shoulder_lift / elbow_flex (kp 150: kp*5deg = 13.1 Nm)
export COLLISION_GRAV_RES_WRIST_FLEX=${COLLISION_GRAV_RES_WRIST_FLEX:-1.5}  # kp 40: kp*5deg = 3.5 Nm; false freeze 13:44:42 pushed 1.38 Nm
export COLLISION_GRAV_RES_JOINTS=${COLLISION_GRAV_RES_JOINTS:-shoulder_lift,elbow_flex,wrist_flex}""")
rep(T, """echo "Schwerkraft-Schutz (v3.2): $([ "$COLLISION_GRAVITY" = 1 ] && echo AN || echo AUS), Feedforward: $([ "$GRAVITY_FF" = 1 ] && echo "AN (x$GRAVITY_FF_SCALE)" || echo AUS)\"""",
"""echo "Schwerkraft-Schutz (v3.3): $([ "$COLLISION_GRAVITY" = 1 ] && echo AN || echo AUS), Feedforward: $([ "$GRAVITY_FF" = 1 ] && echo "AN (x$GRAVITY_FF_SCALE)" || echo AUS), Stillstand-Schwelle ${COLLISION_GRAV_RES_NM} Nm\"""")

# ======================= camera_feeds.py: retry back-off =======================
C = "camera_feeds.py"
rep(C, """RETRY_S = 2.0      # wait between reconnect attempts""",
"""RETRY_S = 2.0      # wait between reconnect attempts (first retry)
RETRY_MAX_S = 30.0  # v3.7: back-off cap - failing sources (e.g. no camera permission) are retried 2, 4, 8 ... 30 s""")
rep(C, """    def _run(self):
        while not self._stop.is_set():
            try:
                if self.source.startswith(("http://", "https://")):
                    self._read_url()
                else:
                    self._read_opencv()
            except Exception as e:
                self.state = f"getrennt ({type(e).__name__}), neuer Versuch …"
            self._stop.wait(RETRY_S)""",
"""    def retry_wait(self, fails):
        \"\"\"v3.7: exponential back-off between reconnects (2, 4, 8, 16, 30, 30 ... s).\"\"\"
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
            self._stop.wait(wait)""")
print("fertig 2")

# ======================= teleop_gui.py =======================
G = "teleop_gui.py"
rep(G, """def log(msg):
    if logfile and not logfile.closed:
        logfile.write(msg); logfile.flush()
    out.configure(state="normal"); out.insert("end", msg); out.see("end"); out.configure(state="disabled")
""", """OUT_MAX_LINES = 1500   # v3.7: keep the log window small (a 20k+ line Tk text widget burned ~90% CPU)
_ANSI = None
_loop_n = {"n": 0}

def show_in_window(line):
    \"\"\"v3.7: which teleop output lines go into the log WINDOW (the session log file still gets every line).
    The 30 Hz 'Teleop loop time' lines (only every 150th, ~5 s), the clamp warnings and their dict dumps are
    skipped - they made up >90% of the output and flooded the Tk text widget.\"\"\"
    s = clean_line(line).strip()
    if s.startswith("Teleop loop time"):
        _loop_n["n"] += 1
        return _loop_n["n"] % 150 == 1
    if "had to be clamped to be safe" in s:
        return False
    if s.startswith(("{'", "{ ", "'", "}")) and ("goal_pos" in s or s.startswith(("'", "}"))):
        return False
    return True

def clean_line(line):
    global _ANSI
    if _ANSI is None:
        import re
        _ANSI = re.compile(r"\\x1b\\[[0-9;]*[A-Za-z]")
    return _ANSI.sub("", line)

def log(msg, window=True):
    if logfile and not logfile.closed:
        logfile.write(msg); logfile.flush()
    if not window or not msg:
        return
    out.configure(state="normal"); out.insert("end", clean_line(msg))
    n = int(out.index("end-1c").split(".")[0])
    if n > OUT_MAX_LINES:
        out.delete("1.0", f"{n - OUT_MAX_LINES}.0")
    out.see("end"); out.configure(state="disabled")
""")
rep(G, """def poll():
    # runs in the main thread: show output and detect process end
    while True:
        try: item = q.get_nowait()
        except queue.Empty: break
        if isinstance(item, tuple) and item[0] == "off": finished_off(item[1])
        else:
            log(item)
            collision_status(item)""",
"""def poll():
    # runs in the main thread: show output and detect process end
    # v3.7: batched - all queued lines go to the log file at once, the window gets one filtered insert
    buf_file, buf_win = [], []
    def _flush():
        if buf_file:
            if logfile and not logfile.closed:
                logfile.write("".join(buf_file)); logfile.flush()
            if buf_win:
                _win("".join(buf_win))
            buf_file.clear(); buf_win.clear()
    for _ in range(5000):
        try: item = q.get_nowait()
        except queue.Empty: break
        if isinstance(item, tuple) and item[0] == "off":
            _flush(); finished_off(item[1])
        else:
            buf_file.append(item)
            if show_in_window(item):
                buf_win.append(item)
            collision_status(item)
    _flush()""")
rep(G, """def collision_status(line):""", """def _win(msg):
    \"\"\"v3.7: window-only insert (the file was already written by poll()).\"\"\"
    out.configure(state="normal"); out.insert("end", clean_line(msg))
    n = int(out.index("end-1c").split(".")[0])
    if n > OUT_MAX_LINES:
        out.delete("1.0", f"{n - OUT_MAX_LINES}.0")
    out.see("end"); out.configure(state="disabled")

def collision_status(line):""")
rep(G, """stopping = {"on": False}
def stop():
    if proc and proc.poll() is None and not stopping["on"]:
        stopping["on"] = True
        log("\\n=== Stopp angefordert – Arm fährt in Ruheposition ===\\n")
        if rec.get("state") == "recording":
            rec_cmd("pause")   # v3.3: rest move is not recorded
        status.config(text="◐ FÄHRT IN RUHEPOSITION…", fg="#d68910"); bstop.config(state="disabled")
        os.killpg(proc.pid, signal.SIGINT)          # einmal Ctrl+C: sauberes Beenden
        root.after(25000, force_kill, proc)

def force_kill(p):
    if p.poll() is None:
        log("\\n=== Erzwinge Beenden ===\\n"); os.killpg(p.pid, signal.SIGKILL)""",
"""stopping = {"on": False, "t0": 0.0}
STOP_TERM_S = 25.0   # v3.7: SIGINT (clean stop + rest move) -> after 25 s SIGTERM -> after 32 s SIGKILL
STOP_KILL_S = 32.0
def stop():
    if proc and proc.poll() is None and not stopping["on"]:
        import time as _t
        stopping["on"] = True; stopping["t0"] = _t.monotonic()
        # v3.7: immediate visible feedback first, then signal
        bstop.config(state="disabled", text="◐ stoppt…")
        status.config(text="◐ STOPPT… fährt in Ruheposition", fg="#d68910")
        try: root.update_idletasks()
        except Exception: pass
        log("\\n=== Stopp angefordert – Arm fährt in Ruheposition ===\\n")
        try:
            if rec.get("state") == "recording":
                rec_cmd("pause")   # v3.3: rest move is not recorded
        except Exception: pass
        _signal(proc, signal.SIGINT)          # einmal Ctrl+C: sauberes Beenden
        root.after(1000, stop_watch, proc)

def _signal(p, sig):
    try: os.killpg(p.pid, sig)
    except ProcessLookupError: pass
    except Exception:
        try: p.send_signal(sig)
        except Exception: pass

def stop_watch(p):
    \"\"\"v3.7: escalate SIGINT -> SIGTERM -> SIGKILL and show the elapsed time while the arm drives to rest.\"\"\"
    import time as _t
    if p.poll() is not None or not stopping["on"]:
        return
    el = _t.monotonic() - stopping["t0"]
    if el >= STOP_KILL_S and not stopping.get("killed"):
        stopping["killed"] = True
        log("\\n=== Erzwinge Beenden (SIGKILL) ===\\n"); _signal(p, signal.SIGKILL)
    elif el >= STOP_TERM_S and not stopping.get("termed"):
        stopping["termed"] = True
        log("\\n=== Teleop reagiert nicht – sende SIGTERM ===\\n"); _signal(p, signal.SIGTERM)
    status.config(text=f"◐ STOPPT… fährt in Ruheposition ({el:.0f} s)", fg="#d68910")
    root.after(1000, stop_watch, p)

def force_kill(p):
    if p.poll() is None:
        log("\\n=== Erzwinge Beenden ===\\n"); _signal(p, signal.SIGKILL)""")
rep(G, """def finished(code):
    stopping["on"] = False""", """def finished(code):
    stopping["on"] = False; stopping.pop("termed", None); stopping.pop("killed", None)
    bstop.config(text="■  STOPP")""")
rep(G, """                               capture_output=True, text=True, timeout=20)
            msg = (r.stdout + r.stderr).strip().splitlines()[-1:] or ["?"]""",
"""                               capture_output=True, text=True, timeout=90)   # v3.7: 20 -> 90 s (laptop import ~35-55 s)
            msg = (r.stdout + r.stderr).strip().splitlines()[-1:] or ["?"]""")
print("fertig 3")

# ======================= safe_zero v3.7: closed loop, feedback check, timeout, soft release =======================
OLD_SZ_START = """            def _frame_count(
                starts: dict[str, float],"""
OLD_SZ_END = """            logger.info("safe_zero done.")
            time.sleep(2.0)
            if exit_on_complete:"""
p = os.path.join(D, F); s = open(p).read()
if "PATCH v3.7 safe_zero" not in s:
    a = s.index(OLD_SZ_START); b = s.index(OLD_SZ_END)
    NEW = '''            # PATCH v3.7 safe_zero (Humanoid Project): closed loop instead of an open-loop time ramp.
            # 2026-11-21 14:09 the old ramp ran on frozen feedback (power/CAN lost: every joint's present value
            # identical to 14 decimals for 9 s), declared "safe_zero done" and left the arm overhead. Now:
            #  - every frame re-reads the motors; the command advances max SAFE_ZERO_STEP_DEG (0.5) per frame and
            #    never more than SAFE_ZERO_LEAD_DEG (3) ahead of the MEASURED position (no stale goal as ramp base)
            #  - frozen feedback (every motor's full state bit-identical for SAFE_ZERO_STALE_S = 0.6 s while moving) -> abort
            #  - per-stage timeout SAFE_ZERO_TIMEOUT_S (20 s) and stall report (no progress for 3 s) with joint,
            #    position and gravity torque in the log
            #  - gravity feedforward is used during the rest move (SAFE_ZERO_GRAVITY_FF=1, default)
            #  - self._safe_zero_result tells disconnect() whether the arm reached rest (soft release otherwise)
            _sz_step = max(0.05, _env_float("SAFE_ZERO_STEP_DEG", 0.5))
            _sz_lead = max(0.5, _env_float("SAFE_ZERO_LEAD_DEG", 3.0))
            _sz_timeout = max(1.0, _env_float("SAFE_ZERO_TIMEOUT_S", 20.0))
            _sz_stale_s = max(0.1, _env_float("SAFE_ZERO_STALE_S", 0.6))
            _sz_stall_s = max(0.5, _env_float("SAFE_ZERO_STALL_S", 3.0))
            _sz_tol = _env_float("SAFE_ZERO_TOL_DEG", 2.0)
            _dt = step_interval_s if step_interval_s > 0 else 0.02
            self._safe_zero_result = {"feedback": True, "reached": False, "positions": {}}

            def _poll_all() -> dict[str, float]:
                for _m in self.motors.values():
                    try:
                        _m.request_feedback()
                    except Exception:
                        pass
                try:
                    self.bus.poll_feedback_once()
                except Exception:
                    logger.debug("safe_zero poll failed")
                out = {}
                for _n, _m in self.motors.items():
                    _s = _m.get_state()
                    if _s is not None:
                        out[_n] = math.degrees(_s.pos)
                return out

            def _send(cmd_motor_deg: dict[str, float]) -> None:
                action: RobotAction = {}
                for joint, v in cmd_motor_deg.items():
                    d = self.config.joint_directions.get(joint, 1.0) or 1.0
                    action[f"{joint}.pos"] = v / d
                self.send_action(action)

            def _interp_to_zero(
                active_starts: dict[str, float],
                hold_joints: dict[str, float],
                targets: dict[str, float] | None = None,
                stage: str = "",
            ) -> str:
                """Closed-loop move of active joints to targets. Returns 'ok', 'timeout', 'stalled',
                'nofeedback' or 'overtemp'."""
                if not active_starts:
                    return "ok"
                targets = targets or {}
                cmd = dict(active_starts)
                hold = dict(hold_joints)
                emergency_disable_threshold_c = self.config.temp_emergency_disable_threshold_c
                t0 = time.monotonic()
                prev_all, stale_t0 = None, None
                best = {j: abs(targets.get(j, 0.0) - v) for j, v in active_starts.items()}
                best_t = {j: t0 for j in active_starts}
                frames = 0
                while True:
                    frames += 1
                    temperatures = self._read_motor_temperatures()
                    for motor_name, temp_c in temperatures.items():
                        if temp_c > emergency_disable_threshold_c:
                            logger.error(
                                "Auto-disable on overtemperature during safe_zero: %s t_mos=%.2fC > %.2fC.",
                                motor_name, temp_c, emergency_disable_threshold_c,
                            )
                            self._emergency_disable_requested = True
                            self.disable_torque()
                            logger.error("safe_zero aborted: emergency overtemperature.")
                            return "overtemp"
                    pres = _poll_all()
                    now = time.monotonic()
                    self._safe_zero_result["positions"] = dict(pres)
                    # frozen feedback: every reported position bit-identical while we command motion
                    # full motor state (position, velocity, torque, temperature): a live motor's torque reading
                    # fluctuates even when its joint is blocked; after a power cut the driver keeps the last frame
                    snap = tuple((n, tuple(getattr(_m.get_state(), a, None) for a in ("pos", "vel", "torq", "t_mos"))
                                  if _m.get_state() is not None else None) for n, _m in sorted(self.motors.items()))
                    moving = any(abs(targets.get(j, 0.0) - pres.get(j, cmd[j])) > _sz_tol for j in cmd)
                    if snap == prev_all and moving:
                        stale_t0 = now if stale_t0 is None else stale_t0
                    else:
                        stale_t0 = None
                    prev_all = snap
                    if stale_t0 is not None and now - stale_t0 >= _sz_stale_s and frames > 3:
                        self._safe_zero_result["feedback"] = False
                        logger.error("safe_zero %s ABGEBROCHEN: keine Motor-Rueckmeldung seit %.1f s (alle Positionen "
                                     "eingefroren: %s) - Strom aus oder CAN getrennt? Arm NICHT in Ruheposition.", stage,
                                     now - stale_t0, ", ".join(f"{j}={pres.get(j, float('nan')):.1f}" for j in cmd))
                        return "nofeedback"
                    done = True
                    for j in list(cmd):
                        tgt = targets.get(j, 0.0)
                        p = pres.get(j, cmd[j])
                        dist = abs(tgt - p)
                        tol = 5.0 if j == FOLLOWER_GRIPPER_MOTOR else _sz_tol
                        if dist > tol:
                            done = False
                        if dist < best[j] - 0.2:
                            best[j], best_t[j] = dist, now
                        step = max(-_sz_step, min(_sz_step, tgt - cmd[j]))
                        nc = cmd[j] + step
                        sg = 1.0 if step >= 0 else -1.0
                        if (nc - p) * sg > _sz_lead:      # never run more than LEAD ahead of the measured position
                            nc = p + sg * _sz_lead
                        if (tgt - nc) * sg < 0:
                            nc = tgt
                        cmd[j] = nc
                    if done:
                        logger.info("safe_zero %s erreicht nach %.1f s", stage, now - t0)
                        return "ok"
                    out = dict(hold); out.update(cmd)
                    _send(out)
                    open_j = [j for j in cmd if abs(targets.get(j, 0.0) - pres.get(j, cmd[j])) > (5.0 if j == FOLLOWER_GRIPPER_MOTOR else _sz_tol)]
                    if open_j and all(now - best_t[j] >= _sz_stall_s for j in open_j):
                        g = self._gravity_torques_deg(pres) if hasattr(self, "_gravity_torques_deg") else {}
                        logger.warning("safe_zero %s STILLSTAND: %s bewegen sich seit %.0f s nicht mehr (%s)", stage,
                                       ", ".join(open_j), _sz_stall_s,
                                       ", ".join(f"{j}={pres.get(j, float('nan')):.1f} deg (Ziel {targets.get(j, 0.0):.0f}, "
                                                 f"Schwerkraft {g.get(j, 0.0):.1f} Nm)" for j in open_j))
                        return "stalled"
                    if now - t0 >= _sz_timeout:
                        logger.warning("safe_zero %s TIMEOUT nach %.0f s: %s", stage, _sz_timeout,
                                       ", ".join(f"{j}={pres.get(j, float('nan')):.1f} deg (Ziel {targets.get(j, 0.0):.0f})" for j in open_j))
                        return "timeout"
                    if step_interval_s > 0.0:
                        time.sleep(step_interval_s)

            stage_1_start = {joint: _read_motor_deg(joint) for joint in stage_1}
            stage_2_start = {joint: _read_motor_deg(joint) for joint in stage_2}

            logger.info("safe_zero stage1 start: joints=%s (Start: %s)", stage_1,
                        ", ".join(f"{j}={v:.1f}" for j, v in {**stage_1_start, **stage_2_start}.items()))
            r1 = _interp_to_zero(stage_1_start, stage_2_start, stage="stage1")
            if r1 in ("overtemp", "nofeedback"):
                return
            stage_1_hold = {j: self._safe_zero_result["positions"].get(j, 0.0) if r1 != "ok" else 0.0 for j in stage_1}

            # Stage 2: move CAN ID 2/3 back to zero, and bring the gripper back to
            # 170° if it is currently past 180° (avoids leaving it wide open).
            stage_2_active = {j: self._safe_zero_result["positions"].get(j, v) for j, v in stage_2_start.items()}
            stage_2_targets: dict[str, float] = {}
            if FOLLOWER_GRIPPER_MOTOR in self.motors:
                try:
                    gripper_motor_deg = _read_motor_deg(FOLLOWER_GRIPPER_MOTOR)
                except RuntimeError as e:
                    logger.warning("safe_zero: could not read gripper position: %s", e)
                    gripper_motor_deg = None

                if gripper_motor_deg is not None and abs(gripper_motor_deg) > 180.0:
                    # Wide open: bring it back toward 170°, preserving sign.
                    target_motor_deg = math.copysign(170.0, gripper_motor_deg)
                    stage_2_active[FOLLOWER_GRIPPER_MOTOR] = gripper_motor_deg
                    stage_2_targets[FOLLOWER_GRIPPER_MOTOR] = target_motor_deg
                    logger.info(
                        "safe_zero gripper: %.2f° (motor, abs>180°), returning to %.2f°",
                        gripper_motor_deg,
                        target_motor_deg,
                    )

            logger.info("safe_zero stage2 start: joints=%s", stage_2)
            r2 = _interp_to_zero(stage_2_active, stage_1_hold, stage_2_targets, stage="stage2")
            if r2 in ("overtemp", "nofeedback"):
                return
            pos = self._safe_zero_result["positions"]
            near = all(abs(pos.get(j, 0.0)) <= _env_float("SAFE_ZERO_REST_TOL_DEG", 8.0) for j in controlled_joints)
            self._safe_zero_result["reached"] = near and r1 == "ok" and r2 == "ok"
            if not self._safe_zero_result["reached"]:
                logger.warning("safe_zero: Arm NICHT ganz in Ruheposition (%s) - beim Ausschalten wird weich abgesenkt",
                               ", ".join(f"{j}={pos.get(j, float('nan')):.1f}" for j in controlled_joints))
'''
    s = s[:a] + NEW + s[b:]
    s = s.replace('''            logger.info("safe_zero done.")
            time.sleep(2.0)
            if exit_on_complete:''', '''            logger.info("safe_zero done (Ruheposition %s).", "erreicht" if self._safe_zero_result["reached"] else "NICHT erreicht")
            time.sleep(0.5 if self._safe_zero_result["reached"] else 0.1)
            if exit_on_complete:''')
    open(p, "w").write(s); print("  safe_zero: ok")
else:
    print("  safe_zero: schon gepatcht")

# FF during safe_zero even when GRAVITY_FF=0 (SAFE_ZERO_GRAVITY_FF=1 default)
rep(F, """        if not gravity_tau and self.motor_type == "rs" and not self.config.gravity_compensation and gravity_ff_enabled():""",
"""        if not gravity_tau and self.motor_type == "rs" and not self.config.gravity_compensation and (
                gravity_ff_enabled() or (self._in_safe_zero and _env_on("SAFE_ZERO_GRAVITY_FF", "1"))):""")

# soft release in disconnect()
rep(F, """        for motor in self.motors.values():
            if self.config.disable_torque_on_disconnect:
                motor.disable()
            motor.clear_error()
            motor.close()""", """        # PATCH v3.7: gentle torque-off when the rest move did not reach rest (and feedback is alive)
        _szr = getattr(self, "_safe_zero_result", None)
        if (_szr and _szr.get("feedback") and not _szr.get("reached") and self.config.disable_torque_on_disconnect
                and not self._emergency_disable_requested and self.motor_type == "rs"):
            try:
                self._soft_release()
            except Exception:
                logger.exception("soft release failed (ignored)")
        elif _szr and not _szr.get("feedback"):
            logger.error("Keine Motor-Rueckmeldung: Motoren koennen evtl. nicht ausgeschaltet werden - Strom pruefen.")
        for motor in self.motors.values():
            try:
                if self.config.disable_torque_on_disconnect:
                    motor.disable()
                motor.clear_error()
                motor.close()
            except Exception:
                logger.exception("disable/close of a motor failed (power off?)")""")
rep(F, """    def disconnect(self):
        \"\"\"Disconnect from robot.\"\"\"""", """    def _soft_release(self) -> None:
        \"\"\"PATCH v3.7: lower the arm gently instead of a hard torque-off away from rest. Over SOFT_RELEASE_S
        (default 3 s) the position gain goes to 0 and the gravity feedforward fades out, with strong damping
        on shoulder_lift/elbow_flex (kd >= 20) so the arm sinks slowly onto its rest; then 1.5 s damping only.\"\"\"
        dur = max(0.5, _env_float("SOFT_RELEASE_S", 3.0))
        arm = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
        kp0 = self._effective_kp()
        kd0 = dict(getattr(self.config, "mit_kd", {})); kd0.update(self._mit_gain_overrides_get()[1])
        pos = {}
        for n in arm:
            s = self.motors[n].get_state()
            if s is not None:
                pos[n] = math.degrees(s.pos)
        g = self._gravity_torques_deg(pos) if len(pos) == len(arm) else {}
        ff0 = {n: max(-GRAVITY_FF_MAX_NM.get(n, 1.0), min(GRAVITY_FF_MAX_NM.get(n, 1.0), 0.8 * g.get(n, 0.0))) for n in arm}
        logger.warning("Weiches Absenken (%.1f s): Arm nicht in Ruheposition (%s)", dur,
                       ", ".join(f"{n}={v:.0f}" for n, v in pos.items()))
        steps = max(1, int(dur / 0.02)); tail = int(1.5 / 0.02)
        for k in range(steps + tail):
            f = max(0.0, 1.0 - k / steps)
            try:
                self.bus.poll_feedback_once()
            except Exception:
                pass
            for n in arm:
                m = self.motors.get(n); s = m.get_state() if m is not None else None
                if s is None:
                    continue
                kd = max(kd0.get(n, 2.0), 20.0) if n in ("shoulder_lift", "elbow_flex") else kd0.get(n, 2.0)
                m.send_mit(s.pos, 0.0, kp0.get(n, 0.0) * f, kd, ff0.get(n, 0.0) * f)
            time.sleep(0.02)

    def disconnect(self):
        \"\"\"Disconnect from robot.\"\"\"""")
print("fertig 4")

# ======================= camera_feeds.py: name-based sources (v3.7) =======================
rep(C, """STALL_S = 3.0      # no data for this long -> reconnect""",
"""STALL_S = 3.0      # no data for this long -> reconnect
BUILTIN_PAT = ("facetime", "built-in", "integrated")   # v3.7: never used unless asked for by name / CAM_ALLOW_BUILTIN=1
_CAM_LIST = {"t": 0.0, "list": None}


def list_cameras(max_age=2.0):
    \"\"\"v3.7: video devices in OpenCV's AVFoundation index order: [(index, name, unique_id), ...].
    OpenCV (cap_avfoundation_mac.mm) enumerates AVCaptureDevice video+muxed devices and sorts them by uniqueID,
    so the same sort over system_profiler's camera list gives the same indices. Cached for max_age seconds.\"\"\"
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
    \"\"\"v3.7: 'name:Innomaker|U20CAM' -> (index, device name) of the first camera whose name contains one of the
    |-separated patterns (case-insensitive). Built-in cameras (FaceTime) are never matched unless a pattern names
    them. Numeric sources are checked too: an index that points at the built-in camera is refused (indices shift
    when USB cameras reconnect) unless CAM_ALLOW_BUILTIN=1. Returns (None, reason) if nothing suitable is attached.\"\"\"
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
    return src, src""")
rep(C, """    def _read_opencv(self):
        import cv2  # lazy: only needed for USB cameras
        src = int(self.source) if self.source.isdigit() else self.source
        self.state = "verbinde …"
""", """    def _read_opencv(self):
        import cv2  # lazy: only needed for USB cameras
        src = self.source
        if src.lower().startswith("name:") or src.isdigit():
            src, info = resolve_source(src)          # v3.7: re-resolved on every (re)connect
            if src is None:
                self.device = None
                raise ConnectionError(info)
            self.device = f"#{src} {info}"
        self.state = "verbinde …"
""")
rep(C, """        self._stop = threading.Event()
        self._t = None""", """        self._stop = threading.Event()
        self._t = None
        self.device = None         # v3.7: resolved camera ('#1 HD Pro Webcam C920') for name/index sources""")
rep(C, """        w, h = self._size
        return f"● {self._fps:4.1f} fps  {w}×{h}" if w else f"● {self._fps:4.1f} fps\"""",
"""        w, h = self._size
        dev = f"  {self.device}" if self.device else ""
        return (f"● {self._fps:4.1f} fps  {w}×{h}{dev}" if w else f"● {self._fps:4.1f} fps{dev}")""")
rep(C, """if __name__ == "__main__":
    import sys
    secs""", """if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--list":     # v3.7: show cameras in OpenCV index order
        for i, n, u in list_cameras():
            print(f"{i}: {n}  ({u}){'  [eingebaut – wird nie automatisch benutzt]' if is_builtin(n) else ''}")
        for fd in load_config()["feeds"]:
            s = str(fd.get("source", ""))
            print(f"{fd['name']}: source={s!r} -> {resolve_source(s) if (s.lower().startswith('name:') or s.isdigit()) else s}")
        raise SystemExit(0)
    secs""")
print("fertig 5")

# ======================= camera roles + enlarge window (v3.7) =======================
rep(C, """def make_feeds(box=(360, 240)):
    cfg = load_config()
    feeds = [Feed(fd["name"], fd.get("source", ""), box, cfg.get("display_fps", 15), fd.get("hint", ""))
             for fd in cfg["feeds"]]
    return feeds, cfg
""", """def guess_role(fd, i):
    \"\"\"v3.7: 'overview' or 'gripper' from cameras.json "role", else from the name, else by position.\"\"\"
    r = str(fd.get("role", "")).strip().lower()
    if r in ("overview", "gripper"):
        return r
    n = str(fd.get("name", "")).lower()
    if any(k in n for k in ("greifer", "gripper", "wrist", "handgelenk")):
        return "gripper"
    if any(k in n for k in ("übersicht", "uebersicht", "overview", "top")):
        return "overview"
    return ("overview", "gripper")[i] if i < 2 else ""


def make_feeds(box=(360, 240)):
    cfg = load_config()
    feeds = [Feed(fd["name"], fd.get("source", ""), box, cfg.get("display_fps", 15), fd.get("hint", ""))
             for fd in cfg["feeds"]]
    for i, (f, fd) in enumerate(zip(feeds, cfg["feeds"])):
        f.role = guess_role(fd, i)
    return feeds, cfg


def by_role(feeds):
    \"\"\"v3.7: [overview feed, gripper feed] whatever the display order in cameras.json is (the recorder and
    play_policy store feed 0 as observation.images.overview and feed 1 as .gripper).\"\"\"
    ov = next((f for f in feeds if getattr(f, "role", "") == "overview"), None)
    gr = next((f for f in feeds if getattr(f, "role", "") == "gripper" and f is not ov), None)
    if ov is None or gr is None:
        return list(feeds[:2])
    return [ov, gr]
""")
rep(C, """    def preview(self):
        \"\"\"Return (PIL.Image, seq) scaled to fit self.box""", """    def frame_fit(self, box, upscale_max=(1920, 1080)):
        \"\"\"v3.7 (enlarge window): newest frame scaled to fit box (aspect kept, at most upscale_max), or None.
        Uses the same newest-frame buffer as preview(); the caller decides how often to call it.\"\"\"
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
            else:
                import cv2
                small = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
                im = Image.fromarray(small[:, :, ::-1].copy())
            return im, seq
        except Exception:
            return None, seq

    def preview(self):
        \"\"\"Return (PIL.Image, seq) scaled to fit self.box""")
rep(C, """# ---------------- Tk widget ----------------
def make_panel(parent, feed, box=(360, 240)):""", """# ---------------- Tk widget ----------------
_BIG = {}   # feed name -> open enlarge window (one per feed)


def open_large(parent, feed):
    \"\"\"v3.7: large resizable live window for one feed (fit to window, aspect kept, up to 1920x1080).
    ✕ button or Esc closes it; the feed itself keeps running (the panel and the recorder use the same thread).\"\"\"
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
                cv.itemconfigure(txt_id, text="Kein Signal\\n" + feed.status_text()); s["seq"] = -1
            st.config(text=feed.status_text())
        except tk.TclError:
            s["alive"] = False; _BIG.pop(feed.name, None); return
        s["job"] = top.after(ms, tick)

    _BIG[feed.name] = top
    top.focus_force()
    tick()
    return top


def make_panel(parent, feed, box=(360, 240)):""")
rep(C, """    st = tk.Label(fr, text="", fg="gray", font=("Menlo", 10), anchor="w")
    st.pack(fill="x", padx=4)""", """    row = tk.Frame(fr); row.pack(fill="x", padx=4)
    tk.Button(row, text="⤢", width=2, command=lambda: open_large(parent, feed)).pack(side="right")   # v3.7 enlarge
    st = tk.Label(row, text="", fg="gray", font=("Menlo", 10), anchor="w")
    st.pack(side="left", fill="x", expand=True)
    cv.bind("<Double-Button-1>", lambda _e: open_large(parent, feed))
    cv.configure(cursor="hand2")""")
rep("recorder.py", """        self.feeds = feeds[:2]""", """        self.feeds = camera_feeds.by_role(feeds)   # v3.7: [overview, gripper] by role, not display order""")
rep("recorder.py", """- images: the two MJPEG streams of the iPhone bridge (cameras.json feed 1 = overview, feed 2 = gripper),""",
"""- images: the two cameras from cameras.json, mapped by "role" (overview / gripper; v3.7, display order free),""")
rep("play_policy.py", """    feeds = feeds[:2]
    if len(feeds) < 2:""", """    feeds = camera_feeds.by_role(feeds)   # v3.7: [overview, gripper] by role, not display order
    if len(feeds) < 2:""")
rep("play_policy.py", """    # gripper level "vorsichtig" from teleop.sh (gentle)
    for k, env, dflt in (("gripper_mit_kp", "PLAY_GRIPPER_KP", 10.0), ("gripper_mit_torque_limit", "PLAY_GRIPPER_TQ", 1.5),""",
"""    # gripper level "vorsichtig" from teleop.sh (gentle). v3.7: opening limit 3.5 Nm (1.5 opened only halfway),
    # closing stays capped at 1.1 Nm via GRIPPER_CLOSE_TQ (same as teleop.sh 'vorsichtig')
    os.environ.setdefault("GRIPPER_CLOSE_TQ", os.environ.get("PLAY_GRIPPER_CLOSE_TQ", "1.1"))
    for k, env, dflt in (("gripper_mit_kp", "PLAY_GRIPPER_KP", 10.0), ("gripper_mit_torque_limit", "PLAY_GRIPPER_TQ", 3.5),""")
print("fertig 6")
