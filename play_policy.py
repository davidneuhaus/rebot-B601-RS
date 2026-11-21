#!/usr/bin/env python3
"""Play a trained ACT policy (or zero-shot SmolVLA) on the B601-RS follower (v3.5). No leader arm needed.

  ./play_policy.sh [checkpoint_dir] [--seconds 60] [--dry-run] [--fake-cams] [--device mps|cpu] [--check]
  ./play_policy.sh --policy smolvla --task "pick up the box and put it in the bin" [--dry-run] [--seconds 60]

--policy smolvla (v3.5, experimental, "just for fun"): runs the public base model lerobot/smolvla_base ZERO-SHOT.
  It was pretrained on community SO-100/SO-101 arms (6 joints, other cameras, other joint zero points) and has never
  seen this arm, so do not expect it to solve the task. Mapping (see docs/SMOLVLA_ZERO_SHOT.md):
  - images: overview -> observation.images.camera1, gripper -> observation.images.camera2 (camera3 left out/masked);
    the model itself resizes+pads them to 512x512.
  - state/action (--smolvla-map so100, default): the 6 SO-100-like joints shoulder_pan, shoulder_lift, elbow_flex,
    wrist_flex, wrist_roll, gripper; wrist_yaw (which SO-100 does not have) is held at its start value.
    --smolvla-map pad7: all 7 joints (the model zero-pads to 32 internally either way).
  - normalization: the base model has no statistics for this robot, so MEAN_STD (un)normalization uses OUR dataset
    statistics (datasets/pick_place_all7/meta/stats.json, --smolvla-stats). This is an approximation.
  - actions are clipped to the min/max joint range of that dataset (margin PLAY_RANGE_MARGIN_DEG, default 0 here).
  - SmolVLA predicts chunks of 50 actions; one (slow) inference per chunk, the chunk is then executed at 30 Hz.

checkpoint_dir: a .../pretrained_model folder, a .../checkpoints/<step> folder or a run folder. Default: the newest
  checkpoint (highest step) in ~/humanoid/outputs/train/act_pick_place_clean_night1*/checkpoints (env PLAY_RUN_GLOB).

Observation = exactly what the recorder stored for training (recorder.py): observation.state = follower joint angles
<motor>.pos in degrees (motor order shoulder_pan .. gripper), images from the iPhone bridge (cameras.json feed 1 =
overview, feed 2 = gripper) resized with the recorder's own Cam class (same size, same padding). Feature names and
image sizes are checked against the checkpoint's config.json before anything moves.

Safety:
- refuses to run on the arm while teleop (lerobot-teleoperate, CAN bus) or a training (lerobot-train, MPS/memory) runs;
- collision guard v3.2 stays active (COLLISION_GUARD=1 and the teleop.sh guard settings, exported by play_policy.sh);
- conservative step limit max_relative_target (default 2 deg/cycle arm joints, 10 deg gripper; env PLAY_MAX_STEP_DEG,
  PLAY_MAX_STEP_GRIPPER_DEG) -> at most 60 deg/s per joint;
- actions are clipped to the joint range seen in the training data (+/- PLAY_RANGE_MARGIN_DEG, default 5);
- time limit (--seconds, default 60, max 300); when it ends (or a camera drops out) the arm drives to rest and
  becomes torque-free;
- SIGTERM / SIGINT (GUI "STOPP", Ctrl+C) = IMMEDIATE torque off (no rest move: the arm sags under gravity - keep it
  over the table). The GUI additionally kills the process after 1.5 s and runs disable_motors.py as a backup.
Exit codes: 0 ok, 2 bad arguments/checkpoint, 3 refused (teleop/training running), 5 cameras, 6 robot, 7 policy error.
"""
import argparse, glob, json, os, re, signal, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TRAIN_DIR = os.path.join(HERE, "outputs", "train")
LOG_DIR = os.path.join(HERE, "logs")
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
POS = [f"{m}.pos" for m in MOTORS]
TASK = "Pick up the object and place it in the box"
SMOLVLA_TASK = "pick up the box and put it in the bin"
SMOLVLA_MODEL = os.environ.get("PLAY_SMOLVLA_MODEL", "lerobot/smolvla_base")
SMOLVLA_STATS = os.path.join(HERE, "datasets", "pick_place_all7", "meta", "stats.json")
SO100_IDX = [0, 1, 2, 3, 5, 6]          # our joints without wrist_yaw (index 4) = SO-100 joint layout


_LOGF = None


def say(msg):
    """Status line to stdout (GUI/terminal) and to the log file; never raises (GUI may be gone)."""
    line = f"[policy {time.strftime('%H:%M:%S')}] {msg}"
    for f in (sys.stdout, _LOGF):
        if f is None:
            continue
        try:
            f.write(line + "\n"); f.flush()
        except Exception:
            pass


# ---------------------------------------------------------------- checkpoint
def _step_of(path):
    m = re.search(r"checkpoints/(\d+)(/|$)", path)
    return int(m.group(1)) if m else -1


def find_checkpoint(arg=None):
    """Return the pretrained_model folder to load."""
    if arg:
        p = os.path.abspath(os.path.expanduser(arg))
        for cand in (p, os.path.join(p, "pretrained_model")):
            if os.path.isfile(os.path.join(cand, "config.json")) and os.path.isfile(os.path.join(cand, "model.safetensors")):
                return cand
        cands = glob.glob(os.path.join(p, "checkpoints", "*", "pretrained_model"))
    else:
        pat = os.environ.get("PLAY_RUN_GLOB", "act_pick_place_clean_night1*")
        cands = glob.glob(os.path.join(TRAIN_DIR, pat, "checkpoints", "*", "pretrained_model"))
    cands = [c for c in cands if "/last/" not in c and os.path.isfile(os.path.join(c, "model.safetensors"))]
    if not cands:
        return None
    # newest = highest step; ties (several runs) -> newest file time
    return max(cands, key=lambda c: (_step_of(c), os.path.getmtime(os.path.join(c, "model.safetensors"))))


def checkpoint_features(ckpt):
    cfg = json.load(open(os.path.join(ckpt, "config.json")))
    return cfg, cfg.get("input_features", {}), cfg.get("output_features", {})


def training_action_range(ckpt):
    """Per-joint (min, max) of the action in the training dataset (meta/stats.json), or None."""
    try:
        tc = json.load(open(os.path.join(ckpt, "train_config.json")))
        root = tc["dataset"]["root"]
        st = json.load(open(os.path.join(root, "meta", "stats.json")))["action"]
        return [float(x) for x in st["min"]], [float(x) for x in st["max"]], root
    except Exception:
        return None


# ---------------------------------------------------------------- conflicts
def running(pattern):
    try:
        out = subprocess.run(["pgrep", "-fl", pattern], capture_output=True, text=True).stdout
    except Exception:
        return []
    me = str(os.getpid())
    return [l for l in out.splitlines() if l.strip() and not l.startswith(me + " ") and "pgrep" not in l]


def conflicts():
    c = []
    if running("lerobot-teleoperate|lerobot_teleoperate"):
        c.append("Teleop läuft (CAN-Bus belegt) – erst STOPP im Teleop-Teil")
    if running("lerobot-train"):
        c.append("Training läuft (Apple-GPU/Speicher belegt) – erst Training beenden lassen")
    if running("handeye.py run"):
        c.append("Auto-Kalibrierung läuft (Arm belegt) – erst deren Ende / STOPP")   # v3.9
    others = [l for l in running("play_policy.py") if "--check" not in l]
    if others:
        c.append("Policy läuft schon in einem anderen Prozess")
    return c


# ---------------------------------------------------------------- cameras
class FakeCam:
    def __init__(self, w, h, seed):
        import numpy as np
        self.rng = np.random.default_rng(seed); self.w, self.h = w, h

    def frame(self):
        return self.rng.integers(0, 255, (self.h, self.w, 3), dtype="uint8"), 0.0


def make_cams(w, h, fake):
    if fake:
        return [FakeCam(w, h, 1), FakeCam(w, h, 2)], None
    os.environ["REC_WIDTH"], os.environ["REC_HEIGHT"] = str(w), str(h)   # recorder.Cam reads W/H at import
    import camera_feeds
    import recorder
    if (recorder.W, recorder.H) != (w, h):
        raise RuntimeError(f"recorder size {recorder.W}x{recorder.H} != checkpoint {w}x{h}")
    feeds, _cfg = camera_feeds.make_feeds((w, h))
    feeds = camera_feeds.by_role(feeds)   # v3.7: [overview, gripper] by role, not display order
    if len(feeds) < 2:
        raise RuntimeError("cameras.json hat keine 2 Kameras")
    for f in feeds:
        f.start()
    return [recorder.Cam(f) for f in feeds], feeds


def wait_cams(cams, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if all(c.frame()[0] is not None and c.frame()[1] < 0.5 for c in cams):
            return True
        time.sleep(0.1)
    return False


# ---------------------------------------------------------------- robot
class FakeRobot:
    """Dry-run stand-in: perfect tracking with the same step limit, nothing is sent anywhere."""
    def __init__(self, start, mrt):
        self.q = dict(zip(POS, start)); self.mrt = mrt; self.sent = 0

    def get_observation(self):
        d = dict(self.q)
        for m in MOTORS:
            d[f"{m}.vel"] = 0.0; d[f"{m}.torque"] = 0.0
        return d

    def send_action(self, a):
        for k, v in a.items():
            lim = self.mrt[k.split(".")[0]]
            self.q[k] += max(-lim, min(lim, v - self.q[k]))
        self.sent += 1
        return a


def make_robot(mrt):
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    import dataclasses
    import lerobot_robot_seeed_b601 as P
    cfgcls = [getattr(P, n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
    robcls = [getattr(P, n) for n in dir(P) if n.endswith("RSFollower")][0]
    names = {f.name for f in dataclasses.fields(cfgcls)}
    kw = dict(port=os.environ.get("PLAY_CAN_PORT", "can0"), id="follower", max_relative_target=mrt)
    # gripper level "vorsichtig" from teleop.sh (gentle). v3.7: opening limit 3.5 Nm (1.5 opened only halfway),
    # closing stays capped at 1.1 Nm via GRIPPER_CLOSE_TQ (same as teleop.sh 'vorsichtig')
    os.environ.setdefault("GRIPPER_CLOSE_TQ", os.environ.get("PLAY_GRIPPER_CLOSE_TQ", "1.1"))
    for k, env, dflt in (("gripper_mit_kp", "PLAY_GRIPPER_KP", 10.0), ("gripper_mit_torque_limit", "PLAY_GRIPPER_TQ", 3.5),
                         ("gripper_mit_hold_torque_limit", "PLAY_GRIPPER_HOLD", 1.0)):
        if k in names:
            kw[k] = float(os.environ.get(env, dflt))
    return robcls(cfgcls(**kw))


def torque_off_now(robot):
    """Immediate torque off, no rest move."""
    try:
        robot._skip_safe_zero_on_disconnect = True
    except Exception:
        pass
    try:
        robot.bus.disable_all()
    except Exception as e:
        say(f"disable_all fehlgeschlagen: {e}")
    try:
        robot.disconnect()
    except Exception:
        pass


# ---------------------------------------------------------------- smolvla (v3.5)
def smolvla_dataset(stats_path):
    """(stats dict, image width, height) of our recording dataset, used for SmolVLA normalization and clipping."""
    st = json.load(open(stats_path))
    info = json.load(open(os.path.join(os.path.dirname(stats_path), "info.json")))
    h, w = info["features"]["observation.images.overview"]["shape"][:2]
    return st, int(w), int(h)


def load_smolvla(model_id, idx, w, h, stats, device):
    """Base SmolVLA with OUR features (len(idx)-dim state/action, 2 cameras) and OUR normalization stats."""
    import numpy as np
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.policies.smolvla.processor_smolvla import make_smolvla_pre_post_processors
    cfg = PreTrainedConfig.from_pretrained(model_id)
    d = len(idx)
    cfg.input_features = {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(d,)),
        "observation.images.camera1": PolicyFeature(type=FeatureType.VISUAL, shape=(3, h, w)),   # overview
        "observation.images.camera2": PolicyFeature(type=FeatureType.VISUAL, shape=(3, h, w)),   # gripper
    }
    cfg.output_features = {"action": PolicyFeature(type=FeatureType.ACTION, shape=(d,))}
    cfg.device = device
    # all weights (VLM included) are in the smolvla_base checkpoint itself -> do not download SmolVLM2 weights again
    cfg.load_vlm_weights = os.environ.get("PLAY_SMOLVLA_LOAD_VLM", "0") == "1"
    if os.environ.get("PLAY_N_ACTION_STEPS"):
        cfg.n_action_steps = int(os.environ["PLAY_N_ACTION_STEPS"])
    policy = SmolVLAPolicy.from_pretrained(model_id, config=cfg)
    policy.to(device).eval()
    # honesty check: how many model tensors were really filled from the checkpoint
    missing = "?"
    try:
        from huggingface_hub import hf_hub_download
        from safetensors import safe_open
        f = model_id if os.path.isdir(model_id) else os.path.dirname(hf_hub_download(model_id, "model.safetensors"))
        with safe_open(os.path.join(f, "model.safetensors"), "pt") as sf:
            ck = set(sf.keys())
        own = set(policy.state_dict().keys())
        missing = f"{len(own - ck)} of {len(own)} model tensors not in checkpoint"
    except Exception as e:
        missing = f"key check failed: {e}"
    sel = {}
    for key in ("observation.state", "action"):
        sel[key] = {k: np.asarray(stats[key][k], dtype=np.float32)[idx] for k in ("mean", "std", "min", "max")}
    pre, post = make_smolvla_pre_post_processors(cfg, dataset_stats=sel)
    policy.reset(); pre.reset(); post.reset()
    return policy, pre, post, missing


# ---------------------------------------------------------------- main
class Stop(BaseException):   # BaseException: the plugin's "except Exception" around safe_zero must not swallow it
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("checkpoint", nargs="?", default=None)
    ap.add_argument("--seconds", type=float, default=float(os.environ.get("PLAY_SECONDS", 60)))
    ap.add_argument("--dry-run", action="store_true", help="no robot: fake joint state, policy + cameras only")
    ap.add_argument("--fake-cams", action="store_true", help="synthetic images instead of the iPhone bridge")
    ap.add_argument("--device", default=None, help="mps (default on the arm) or cpu")
    ap.add_argument("--check", action="store_true", help="only check checkpoint/features/cameras, load nothing heavy")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--policy", choices=["act", "smolvla"], default=os.environ.get("PLAY_POLICY", "act"))
    ap.add_argument("--task", default=None, help="SmolVLA text command (default: %r)" % SMOLVLA_TASK)
    ap.add_argument("--smolvla-model", default=SMOLVLA_MODEL)
    ap.add_argument("--smolvla-stats", default=SMOLVLA_STATS)
    ap.add_argument("--smolvla-map", choices=["so100", "pad7"], default=os.environ.get("PLAY_SMOLVLA_MAP", "so100"))
    a = ap.parse_args()
    smol = a.policy == "smolvla"
    task = (a.task or "").strip() or (SMOLVLA_TASK if smol else TASK)
    seconds = max(1.0, min(300.0, a.seconds))
    os.makedirs(LOG_DIR, exist_ok=True)
    global _LOGF
    try:
        _LOGF = open(os.environ.get("PLAY_LOG") or os.path.join(LOG_DIR, time.strftime("play_%Y-%m-%d_%H-%M-%S.log")), "a")
    except OSError:
        _LOGF = None
    say("Start: " + " ".join(sys.argv[1:]))

    if smol:
        idx = SO100_IDX if a.smolvla_map == "so100" else list(range(7))
        try:
            sstats, w, h = smolvla_dataset(a.smolvla_stats)
            rng = ([float(x) for x in sstats["action"]["min"]], [float(x) for x in sstats["action"]["max"]],
                   os.path.dirname(os.path.dirname(a.smolvla_stats)))
        except Exception as e:
            say(f"SmolVLA: Datensatz-Statistik nicht lesbar ({a.smolvla_stats}): {e}"); return 2
        want = ["observation.images.overview", "observation.images.gripper"]
        smol_keys = {"observation.images.overview": "observation.images.camera1",
                     "observation.images.gripper": "observation.images.camera2"}
        say(f"SmolVLA zero-shot: Modell {a.smolvla_model}, Befehl \"{task}\", Gelenke {a.smolvla_map} "
            f"({len(idx)}), Normierung+Grenzen aus {rng[2]}, Kameras overview->camera1, gripper->camera2, {w}x{h}")
    ckpt = None if smol else find_checkpoint(a.checkpoint)
    if not smol and not ckpt:
        say("Kein Checkpoint gefunden (" + (a.checkpoint or f"{TRAIN_DIR}/act_pick_place_clean_night1*/checkpoints") + ")")
        return 2
    if not smol:
      cfg, inf, outf = checkpoint_features(ckpt)
      say(f"Checkpoint: {ckpt}  (Schritt {_step_of(ckpt)})")
      img_keys = [k for k, v in inf.items() if v.get("type") == "VISUAL"]
      want = ["observation.images.overview", "observation.images.gripper"]
      problems = []
      if cfg.get("type") != "act":
          problems.append(f"policy type {cfg.get('type')} (expected act)")
      if sorted(img_keys) != sorted(want):
          problems.append(f"camera keys {img_keys} (expected {want})")
      shapes = {tuple(inf[k]["shape"]) for k in img_keys}
      if len(shapes) != 1:
          problems.append(f"different image shapes {shapes}")
      if tuple(inf.get("observation.state", {}).get("shape", ())) != (7,) or tuple(outf.get("action", {}).get("shape", ())) != (7,):
          problems.append("state/action must have 7 values")
      if problems:
          say("Checkpoint passt nicht: " + "; ".join(problems)); return 2
      c, h, w = list(shapes)[0]
      rng = training_action_range(ckpt)
      say(f"Merkmale ok: state[7], action[7], Kameras overview+gripper {w}x{h}" +
          (f", Trainingsdaten {rng[2]}" if rng else ", (Trainings-Statistik nicht gefunden – keine Bereichsgrenze)"))

    conf = conflicts()
    if a.check:
        import urllib.request
        for i in (1, 2):
            try:
                st = json.load(urllib.request.urlopen(f"http://127.0.0.1:8090/status{i}", timeout=2))
                say(f"Kamera {i}: {json.dumps(st)[:160]}")
            except Exception as e:
                say(f"Kamera {i}: Bridge nicht erreichbar ({e})")
        say("Würde jetzt verweigern: " + "; ".join(conf) if conf else "Keine Konflikte (Teleop/Training aus)")
        say("CHECK OK")
        return 0
    if conf and not a.dry_run:
        for x in conf:
            say("VERWEIGERT: " + x)
        return 3
    device = a.device or ("cpu" if (a.dry_run and conf) else "mps")
    if device == "mps":
        import torch
        if not torch.backends.mps.is_available():
            device = "cpu"

    # ---- policy
    import numpy as np, torch
    t0 = time.time()
    if smol:
        try:
            policy, pre, post, missing = load_smolvla(a.smolvla_model, idx, w, h, sstats, device)
        except Exception as e:
            import traceback; traceback.print_exc()
            say(f"SmolVLA laden fehlgeschlagen: {e}"); return 7
        say(f"SmolVLA geladen auf {device} in {time.time() - t0:.1f} s (Aktionsblock {policy.config.n_action_steps} "
            f"Schritte, {sum(p.numel() for p in policy.parameters()) / 1e6:.0f} M Parameter; {missing})")
    else:
      from lerobot.policies.act.modeling_act import ACTPolicy
      from lerobot.policies.factory import make_pre_post_processors
      policy = ACTPolicy.from_pretrained(ckpt)
      policy.config.device = device
      policy.to(device).eval()
      if os.environ.get("PLAY_N_ACTION_STEPS"):
          policy.config.n_action_steps = int(os.environ["PLAY_N_ACTION_STEPS"])
      pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt,
                                           preprocessor_overrides={"device_processor": {"device": device}})
      policy.reset(); pre.reset(); post.reset()
      say(f"Policy geladen auf {device} in {time.time() - t0:.1f} s (Aktionsblock {policy.config.n_action_steps} Schritte)")

    arm = float(os.environ.get("PLAY_MAX_STEP_DEG", 2.0)); grip = float(os.environ.get("PLAY_MAX_STEP_GRIPPER_DEG", 10.0))
    mrt = {m: (grip if m == "gripper" else arm) for m in MOTORS}
    margin = float(os.environ.get("PLAY_RANGE_MARGIN_DEG", 0.0 if smol else 5.0))
    lo = np.array(rng[0]) - margin if rng else None
    hi = np.array(rng[1]) + margin if rng else None

    # ---- cameras
    try:
        cams, feeds = make_cams(w, h, a.fake_cams)
    except Exception as e:
        say(f"Kameras: {e}"); return 5
    if not wait_cams(cams):
        say("Kameras liefern kein Bild – iPhones prüfen (Safari offen, Start getippt)"); return 5

    # ---- signals: STOP = immediate torque off
    phase = {"p": "setup", "stop": False}
    def on_sig(signum, _frm):
        phase["stop"] = True
        if phase["p"] == "rest":
            raise Stop()
    signal.signal(signal.SIGTERM, on_sig); signal.signal(signal.SIGINT, on_sig)

    # ---- robot
    if a.dry_run:
        start = [0.0] * 7
        if rng:
            start = list((np.array(rng[0]) + np.array(rng[1])) / 2)
        robot = FakeRobot(start, mrt)
        say("DRY-RUN: keine Motoren, simulierte Gelenkwerte")
    else:
        say(f"Verbinde Folgearm (Kollisionsschutz {'AN' if os.environ.get('COLLISION_GUARD', '1') == '1' else 'AUS'}, "
            f"max {arm:g}°/Zyklus, Greifer {grip:g}°)")
        try:
            robot = make_robot(mrt)
            robot.connect()
        except Exception as e:
            say(f"Folgearm nicht erreichbar: {e}"); return 6
    if phase["stop"]:
        if not a.dry_run: torque_off_now(robot)
        say("STOPP vor dem Start"); return 0

    # ---- loop
    period = 1.0 / a.fps; n = 0; t_start = time.time(); nxt = time.perf_counter(); reason = "Zeitlimit"
    infer_ms = []; acts = []; rc = 0; chunk_ms = []; hold = None
    phase["p"] = "run"
    if smol:
        say("HINWEIS: SmolVLA zero-shot kennt diesen Arm nicht – Bewegungen können sinnlos sein; Hand am Not-Aus")
    say(f"LÄUFT: Policy steuert den Arm für max. {seconds:.0f} s – STOPP = Motoren sofort aus")
    try:
        while time.time() - t_start < seconds:
            if phase["stop"]:
                reason = "STOPP"; break
            obs = robot.get_observation()
            state = np.array([obs[k] for k in POS], dtype=np.float32)
            if smol:
                if hold is None:
                    hold = state.copy()      # joints the model does not control (wrist_yaw in so100 mode)
                batch = {"observation.state": torch.from_numpy(state[idx])[None], "task": [task]}
            else:
                batch = {"observation.state": torch.from_numpy(state)[None], "task": [TASK]}
            stale = False
            for k, cam in zip(want, cams):
                img, age = cam.frame()
                if img is None or age > 2.0:
                    stale = True; break
                batch[smol_keys[k] if smol else k] = torch.from_numpy(np.array(img, copy=True)).permute(2, 0, 1)[None].float() / 255.0
            if stale:
                reason = "Kamera ausgefallen"; rc = 5; break
            new_chunk = smol and len(policy._queues["action"]) == 0
            ti = time.perf_counter()
            with torch.inference_mode():
                act = post(policy.select_action(pre(batch)))
            infer_ms.append((time.perf_counter() - ti) * 1000)
            if new_chunk:
                chunk_ms.append(infer_ms[-1])
                say(f"SmolVLA Aktionsblock {len(chunk_ms)}: Inferenz {chunk_ms[-1]:.0f} ms")
            if smol:
                full = hold.astype(np.float64).copy()
                full[idx] = act.detach().to("cpu").numpy().reshape(-1)[:len(idx)]
                act = full
            else:
                act = act.detach().to("cpu").numpy().reshape(-1)[:7].astype(np.float64)
            if not np.all(np.isfinite(act)):
                reason = "ungültige Aktion (NaN)"; rc = 7; break
            if lo is not None:
                act = np.clip(act, lo, hi)
            acts.append(act)
            if phase["stop"]:
                reason = "STOPP"; break
            robot.send_action({k: float(v) for k, v in zip(POS, act)})
            n += 1
            if n % (a.fps * 5) == 0:
                say(f"{n / a.fps:.0f} s, Inferenz max {max(infer_ms[-a.fps * 5:]):.0f} ms")
            nxt += period
            d = nxt - time.perf_counter()
            if d > 0: time.sleep(d)
            else: nxt = time.perf_counter()
    except Stop:
        reason = "STOPP"
    except Exception as e:
        import traceback; traceback.print_exc()
        reason = f"Fehler: {e}"; rc = 7

    # ---- end
    dur = time.time() - t_start
    if acts:
        A = np.array(acts)
        say(f"{n} Aktionen in {dur:.1f} s ({n / max(dur, 1e-6):.1f} Hz), Inferenz Ø {np.mean(infer_ms):.1f} ms / max "
            f"{np.max(infer_ms):.0f} ms" + (f", pro SmolVLA-Block Ø {np.mean(chunk_ms):.0f} ms (n={len(chunk_ms)})" if chunk_ms else "") +
            f"; Aktion min {np.round(A.min(0), 1).tolist()} max {np.round(A.max(0), 1).tolist()}")
    if a.dry_run:
        say(f"DRY-RUN Ende ({reason}) – keine Motoren bewegt"); return rc
    if reason == "STOPP":
        torque_off_now(robot); say("STOPP: Motoren sofort aus"); return rc
    say(f"Ende ({reason}) – Arm fährt in Ruheposition, dann Motoren aus (STOPP = sofort aus)")
    phase["p"] = "rest"
    try:
        robot.disconnect()           # plugin: safe_zero (rest), then torque off
        say("Ruheposition erreicht, Motoren aus")
    except Stop:
        torque_off_now(robot); say("STOPP: Motoren sofort aus")
    except Exception as e:
        say(f"Ruhefahrt fehlgeschlagen ({e}) – Motoren sofort aus"); torque_off_now(robot)
    return rc


if __name__ == "__main__":
    sys.exit(main())
