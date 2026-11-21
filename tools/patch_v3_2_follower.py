"""Apply the guard v3.2 (gravity-aware) patch to seeed_b601_follower.py (v3.1). Usage: python patch_v3_2_follower.py <file>"""
import sys
path = sys.argv[1]
s = open(path).read()
assert "guard v3.2" not in s, "already patched"

def rep(old, new, count=1):
    global s
    n = s.count(old)
    assert n == count, f"expected {count}x, found {n}x: {old[:80]!r}"
    s = s.replace(old, new)

# 1) header comment
rep("""# < COLLISION_TORQ_MAX_PROGRESS_DEG.
# ---------------------------------------------------------------------------
""", """# < COLLISION_TORQ_MAX_PROGRESS_DEG.
# guard v3.2: gravity-aware. The expected gravity torque g(q) per joint comes from the
# Seeed URDF (gravity.py, validated: lift 152/elbow 120 -> model -14.2 Nm, measured
# -14.9 Nm). Torque triggers use the residual |measured - g(q)|. The lag ("blockiert")
# triggers get a load-dependent tolerance: + COLLISION_GRAV_LAG_GAIN * g_unc/kp (deg)
# when the joint lags against gravity, and are skipped when the uncompensated gravity
# exceeds COLLISION_GRAV_SAT_FRAC of what kp can deliver inside max_relative_target
# (the joint physically cannot follow; freezing would not help). COLLISION_GRAVITY=0
# switches this off (exact v3.1 behaviour). Optional feedforward GRAVITY_FF=1 adds
# GRAVITY_FF_SCALE (default 0.8) * g(q) as MIT torque feedforward (default off).
# ---------------------------------------------------------------------------
""")

# 2) env helpers
rep("""def collision_guard_enabled() -> bool:
    import os as _os
    return _os.environ.get("COLLISION_GUARD", "0").strip().lower() in ("1", "true", "on", "yes")
""", """def collision_guard_enabled() -> bool:
    import os as _os
    return _os.environ.get("COLLISION_GUARD", "0").strip().lower() in ("1", "true", "on", "yes")


def _env_on(name: str, default: str = "0") -> bool:
    import os as _os
    return _os.environ.get(name, default).strip().lower() in ("1", "true", "on", "yes")


def gravity_ff_enabled() -> bool:
    \"\"\"PATCH (guard v3.2): GRAVITY_FF=1 adds scaled gravity torque as MIT feedforward.\"\"\"
    return _env_on("GRAVITY_FF", "0")


def gravity_ff_scale() -> float:
    return max(0.0, min(1.0, _env_float("GRAVITY_FF_SCALE", 0.8)))


def guard_gravity_enabled() -> bool:
    \"\"\"PATCH (guard v3.2): COLLISION_GRAVITY=0 disables the gravity-aware guard (v3.1 behaviour).\"\"\"
    return _env_on("COLLISION_GRAVITY", "1")


_GRAVITY_MOD = None


def _gravity_module():
    \"\"\"PATCH (guard v3.2): the plugin's gravity.py (package import, or the sibling file when this
    module is loaded standalone, e.g. by the offline tests).\"\"\"
    global _GRAVITY_MOD
    if _GRAVITY_MOD is None:
        try:
            from . import gravity as _m
        except ImportError:
            import importlib.util as _ilu, os as _os
            _p = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "gravity.py")
            _spec = _ilu.spec_from_file_location("b601_gravity", _p)
            _m = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_m)
        _GRAVITY_MOD = _m
    return _GRAVITY_MOD


# Hard cap for the gravity feedforward per joint (N·m).
GRAVITY_FF_MAX_NM = {"shoulder_pan": 18.0, "shoulder_lift": 18.0, "elbow_flex": 12.0,
                     "wrist_flex": 4.0, "wrist_yaw": 4.0, "wrist_roll": 4.0}
""")

# 3) class doc + init params
rep('''    """Detect blocked arm joints during teleop and freeze the arm (v3.1)."""''',
    '''    """Detect blocked arm joints during teleop and freeze the arm (v3.2, gravity-aware)."""''')
rep("""                 retry_waits=(3.0, 5.0), torq_min_lead_deg=0.0, torq_max_progress_deg=1e9):
        self.joints = list(joints)""",
"""                 retry_waits=(3.0, 5.0), torq_min_lead_deg=0.0, torq_max_progress_deg=1e9,
                 kp=None, clamp_deg=None, ff_scale=0.0, grav_lag_gain=1.5, grav_sat_frac=0.85,
                 grav_min_nm=0.3):
        self.joints = list(joints)
        # v3.2 gravity awareness (only active when update() gets a grav dict)
        kp = kp or {}
        self.kp = {j: float(kp.get(j, 0.0)) for j in self.joints}            # Nm/rad
        if isinstance(clamp_deg, dict):
            self.clamp_deg = {j: float(clamp_deg.get(j, 1e9)) for j in self.joints}
        else:
            self.clamp_deg = {j: float(clamp_deg if clamp_deg is not None else 1e9) for j in self.joints}
        self.ff_scale = max(0.0, min(1.0, float(ff_scale)))
        self.grav_lag_gain = max(0.0, float(grav_lag_gain))
        self.grav_sat_frac = float(grav_sat_frac)
        self.grav_min_nm = float(grav_min_nm)
        self.grav = {}
        self.sat_warned = set()""")

# 4) from_env: pass-through kwargs
rep("""    def from_env(cls, joints, log=None):
        err, tq, lead, lag = {}, {}, {}, {}""",
"""    def from_env(cls, joints, log=None, gains=None, clamp_deg=None, ff_scale=0.0):
        err, tq, lead, lag = {}, {}, {}, {}""")
rep("""            stats_period_s=_env_float("COLLISION_STATS_S", 5.0),
            log=log,
        )""", """            stats_period_s=_env_float("COLLISION_STATS_S", 5.0),
            kp=gains, clamp_deg=clamp_deg, ff_scale=ff_scale,
            grav_lag_gain=_env_float("COLLISION_GRAV_LAG_GAIN", 1.5),
            grav_sat_frac=_env_float("COLLISION_GRAV_SAT_FRAC", 0.85),
            log=log,
        )""")

# 5) describe
rep("""        return (f"Kollisionsschutz AKTIV: Leader-Abweichung >[{thr}] deg und Fortschritt \"""",
"""        if any(v > 0 for v in self.kp.values()):
            ar += (f"; Schwerkraft-Modell: Rest-Drehmoment fuer Kraft-Trigger, Durchhang-Toleranz "
                   f"{self.grav_lag_gain:g}x g/kp, Feedforward-Anteil {self.ff_scale:g}")
        return (f"Kollisionsschutz AKTIV: Leader-Abweichung >[{thr}] deg und Fortschritt \"""")

# 6) stats with residual
rep("""            if torq and torq.get(j) is not None:
                self.st_tq[j] = max(self.st_tq[j], abs(torq[j]))""",
"""            if torq and torq.get(j) is not None:
                self.st_tq[j] = max(self.st_tq[j], abs(torq[j]))
                self.st_res[j] = max(self.st_res[j], abs(torq[j] - self.grav.get(j, 0.0)))
            if j in self.grav:
                self.st_g[j] = max(self.st_g[j], abs(self.grav[j]))""")
rep("""        self.st_tq = {j: 0.0 for j in self.joints}
""", """        self.st_tq = {j: 0.0 for j in self.joints}
        self.st_res = {j: 0.0 for j in self.joints}
        self.st_g = {j: 0.0 for j in self.joints}
""")
rep("""                f"{j} trk={self.st_trk[j]:.1f} lead={self.st_disc[j]:.1f} tq={self.st_tq[j]:.2f}" for j in self.joints))""",
"""                f"{j} trk={self.st_trk[j]:.1f} lead={self.st_disc[j]:.1f} tq={self.st_tq[j]:.2f}"
                + (f" g={self.st_g[j]:.2f} res={self.st_res[j]:.2f}" if self.grav else "") for j in self.joints))""")

# 7) residual torque trigger + gravity tolerance helper
rep("""    def _tq_hit(self, j, tq, lead, progress):
        if self.torq_limit[j] <= 0 or tq is None or abs(tq) <= self.torq_limit[j]:
            return False""",
"""    def _grav_unc(self, j):
        \"\"\"Gravity torque the P-term must still deliver (the feedforward covers ff_scale of it).\"\"\"
        return self.grav.get(j, 0.0) * (1.0 - self.ff_scale)

    def _grav_tol(self, j, disc):
        \"\"\"v3.2: (extra lag tolerance in deg, saturated) for a joint lagging by disc deg.
        Only when the joint lags AGAINST gravity (disc has the sign of the holding torque g):
        expected static sag = g_unc/kp; saturated = kp cannot deliver g_unc within the
        max_relative_target clamp, so the joint cannot follow at all (freezing would not help).\"\"\"
        gu = self._grav_unc(j)
        kp = self.kp.get(j, 0.0)
        if kp <= 0 or abs(gu) < self.grav_min_nm or disc == 0 or (disc > 0) != (gu > 0):
            return 0.0, False
        avail = kp * math.radians(min(self.clamp_deg.get(j, 1e9), 1e6))
        sat = abs(gu) >= self.grav_sat_frac * avail
        return self.grav_lag_gain * math.degrees(abs(gu) / kp), sat

    def _note_sat(self, j, gu):
        if j not in self.sat_warned:
            self.sat_warned.add(j)
            self.log.warning("Schwerkraft: %s braucht %.1f Nm Haltemoment – mehr als kp im 5-deg-Limit schafft; "
                             "Durchhang wird nicht als Kollision gewertet (Tipp: GRAVITY_FF=1)", JOINT_DE.get(j, j), abs(gu))

    def _tq_hit(self, j, tq, lead, progress):
        if tq is not None and self.grav:
            tq = tq - self.grav.get(j, 0.0)       # v3.2: residual torque (measured - gravity)
        if self.torq_limit[j] <= 0 or tq is None or abs(tq) <= self.torq_limit[j]:
            return False""")

# 8) update signature + normal path
rep("""    def update(self, goal, present, torq, now):
        \"\"\"goal/present in follower degrees (after directions+limits). Returns a
        dict of overriding arm-joint targets while frozen or creeping, else None.\"\"\"
        if self.t0 is None:""",
"""    def update(self, goal, present, torq, now, grav=None):
        \"\"\"goal/present in follower degrees (after directions+limits). grav (v3.2, optional):
        expected gravity holding torque per joint in N·m (motor convention). Returns a
        dict of overriding arm-joint targets while frozen or creeping, else None.\"\"\"
        self.grav = dict(grav) if grav else {}
        if self.t0 is None:""")
rep("""            blocked = abs(disc) > self.err_deg[j] and progress < self.min_progress_deg
            tq_hit = self._tq_hit(j, tq, disc, progress)""",
"""            extra, sat = self._grav_tol(j, disc)
            blocked = abs(disc) > self.err_deg[j] + extra and progress < self.min_progress_deg and not sat
            if sat and abs(disc) > self.err_deg[j]:
                self._note_sat(j, self._grav_unc(j))
            tq_hit = self._tq_hit(j, tq, disc, progress)""")

# 9) creep path
rep("""            blocked = lag >= self.creep_lag[j] and progress < self.min_progress_deg
            tq_hit = self._tq_hit(j, tq, goal[j] - present[j], progress)""",
"""            extra, sat = self._grav_tol(j, e)
            blocked = lag >= self.creep_lag[j] + extra and progress < self.min_progress_deg and not sat
            tq_hit = self._tq_hit(j, tq, goal[j] - present[j], progress)""")

# 10) log gravity at freeze (separate line, message format unchanged for the GUI)
rep("""                         j, disc, trk, "n/a" if tq is None else f"{tq:.2f}", progress, why, JOINT_DE.get(j, j))
        return dict(self.frozen)""",
"""                         j, disc, trk, "n/a" if tq is None else f"{tq:.2f}", progress, why, JOINT_DE.get(j, j))
        if self.grav:
            g = self.grav.get(j, 0.0)
            self.log.info("Schwerkraft beim Einfrieren: %s g=%.2f Nm, Rest-Drehmoment=%s Nm", j, g,
                          "n/a" if tq is None else f"{tq - g:.2f}")
        return dict(self.frozen)""")

# 11) follower: build guard with kp/clamp/ff
rep("""                joints = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
                self._collision_guard = CollisionGuard.from_env(joints)
                logger.info(self._collision_guard.describe())""",
"""                joints = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
                kp, clamp, ff = None, None, 0.0
                if guard_gravity_enabled() and self._gravity_model_get() is not None:
                    kp = self._effective_kp()
                    clamp = getattr(self.config, "max_relative_target", None)
                    if getattr(self.config, "gravity_compensation", False):
                        ff = 1.0
                    elif gravity_ff_enabled():
                        ff = gravity_ff_scale()
                self._collision_guard = CollisionGuard.from_env(joints, gains=kp, clamp_deg=clamp, ff_scale=ff)
                logger.info(self._collision_guard.describe())""")

# 12) follower helpers (gravity model independent of config.gravity_compensation)
rep("""    def _mit_gain_overrides_get(self):""",
"""    def _gravity_model_get(self):
        \"\"\"PATCH (guard v3.2): lazily load the URDF gravity model + motor pairing; None on failure.\"\"\"
        if not hasattr(self, "_grav_model"):
            self._grav_model = None
            try:
                if self.motor_type != "rs":
                    raise RuntimeError("gravity model only for the RS arm")
                urdf = getattr(self.config, "gravity_urdf_path", None)
                if not urdf:
                    raise RuntimeError("gravity_urdf_path not set")
                model = _gravity_module().load_dynamics_model(urdf_path=urdf)
                arm = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
                qidx = [j.q_idx for j in model.joints if j.jtype in ("revolute", "continuous")]
                if len(arm) != len(qidx):
                    raise RuntimeError(f"arm motors {len(arm)} != URDF revolute joints {len(qidx)}")
                self._grav_model = (model, dict(zip(arm, qidx)))
                logger.info("Schwerkraft-Modell geladen (%s)", urdf)
            except Exception as exc:
                logger.warning("Schwerkraft-Modell nicht verfuegbar, Schutz ohne Schwerkraft (v3.1-Verhalten): %s", exc)
        return self._grav_model

    def _gravity_torques_deg(self, pos_deg):
        \"\"\"PATCH (guard v3.2): gravity holding torque (N·m) per arm joint from positions in degrees.
        Returns {} unless all arm joints are known.\"\"\"
        gm = self._gravity_model_get()
        if gm is None:
            return {}
        model, pairing = gm
        if any(n not in pos_deg for n in pairing):
            return {}
        import numpy as np
        q = np.zeros(model.nq)
        for n, qi in pairing.items():
            q[qi] = math.radians(pos_deg[n])
        tau = _gravity_module().compute_generalized_gravity(model=model, q=q)
        return {n: float(tau[qi]) for n, qi in pairing.items()}

    def _effective_kp(self):
        \"\"\"MIT kp per arm joint as actually sent (config + env overrides).\"\"\"
        if getattr(self.config, "gravity_compensation", False):
            return dict(getattr(self.config, "gravity_mit_kp", {}))
        kp = dict(getattr(self.config, "mit_kp", {}))
        kp.update(self._mit_gain_overrides_get()[0])
        return kp

    def _mit_gain_overrides_get(self):""")

# 13) send_action: compute gravity once, pass to guard
rep("""        _guard = self._collision_guard_get()
        if _guard is not None:""",
"""        _guard = self._collision_guard_get()
        _grav_cycle = None   # v3.2: gravity torques of this cycle (computed at most once)
        if _guard is not None:""")
rep("""                    _override = _guard.update(goal_pos, _gp, _gt, time.monotonic())""",
"""                    _gg = None
                    if any(v > 0 for v in _guard.kp.values()):
                        _gg = self._gravity_torques_deg(_gp)
                        _grav_cycle = _gg
                    _override = _guard.update(goal_pos, _gp, _gt, time.monotonic(), grav=_gg)""")

# 14) feedforward
rep("""        gravity_tau = (
            self._gravity_feedforward()
            if (self.config.gravity_compensation and self.motor_type == "rs")
            else {}
        )
""", """        gravity_tau = (
            self._gravity_feedforward()
            if (self.config.gravity_compensation and self.motor_type == "rs")
            else {}
        )
        # PATCH (guard v3.2): optional gravity feedforward GRAVITY_FF=1 (keeps the tuned kp/kd)
        if not gravity_tau and self.motor_type == "rs" and not self.config.gravity_compensation and gravity_ff_enabled():
            try:
                if not _grav_cycle:
                    _pd = {}
                    for _n in self.motor_names:
                        _m = self.motors.get(_n)
                        _s = _m.get_state() if _m is not None else None
                        if _s is not None:
                            _pd[_n] = math.degrees(_s.pos)
                    _grav_cycle = self._gravity_torques_deg(_pd)
                _sc = gravity_ff_scale()
                gravity_tau = {n: max(-GRAVITY_FF_MAX_NM.get(n, 4.0), min(GRAVITY_FF_MAX_NM.get(n, 4.0), _sc * g))
                               for n, g in (_grav_cycle or {}).items()}
                if not getattr(self, "_grav_ff_logged", False):
                    self._grav_ff_logged = True
                    logger.info("Schwerkraft-Feedforward AN (Anteil %.2f): %s", _sc,
                                ", ".join(f"{n}={v:.2f}" for n, v in gravity_tau.items()) or "kein Modell")
            except Exception:
                logger.warning("gravity feedforward error (ignored, tau=0)", exc_info=True)
                gravity_tau = {}
""")
open(path, "w").write(s)
print("patched", path)
