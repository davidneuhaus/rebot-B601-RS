#!/usr/bin/env python3
"""Post-run drop analysis (v3.9.4): merges a handeye run's motor trace (logs/handeye_<ts>_motors.jsonl, written by
vision/handeye.py MotorTrace) with the passive monitor (logs/dropmon_*/events.jsonl, tools/drop_monitor.sh) into a
timeline around the failure (+-5 s) and prints a verdict:
  A  CAN adapter lost on USB (detach / re-enumerate)      -> USB hub chain / cable / power of the hub
  B  whole bus silent, adapter still on USB               -> motor power (supply, e-stop, connector) or CAN cable to
                                                             the first motor
  C  only distal motors silent, proximal still answer     -> cable / plug between the last answering and the first
                                                             silent motor (wrist cable)
  D  a motor reports a fault flag                         -> undervoltage / overcurrent / overtemp / encoder
It also says whether a camera (USB detach or frame stall) or the CAN adapter vanished first.

  .venv/bin/python tools/drop_report.py                 newest handeye log + matching monitor folder
  .venv/bin/python tools/drop_report.py logs/handeye_2026-11-22_00-10-00.log [--window 5] [--out report.md]
"""
import argparse
import glob
import json
import os
import re
import time
from pathlib import Path

HUM = Path(os.environ.get("HUMANOID_DIR", str(Path.home() / "humanoid")))
ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
# RobStride type-2 feedback frame fault bits (motorbridge fault_code, bit0..bit5)
FAULTS = {0: "undervoltage", 1: "overcurrent", 2: "overtemperature", 3: "magnetic encoder fault",
          4: "HALL encoder fault", 5: "not calibrated"}


def fmt(t):
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(round((t % 1) * 1000)) % 1000:03d}"


def jl(path):
    out = []
    try:
        with open(path) as f:
            for ln in f:
                try: out.append(json.loads(ln))
                except Exception: pass
    except FileNotFoundError:
        pass
    return out


def log_start(path):
    m = re.search(r"(\d{4}-\d\d-\d\d)_(\d\d)-(\d\d)-(\d\d)", os.path.basename(path))
    return time.mktime(time.strptime(f"{m[1]} {m[2]}:{m[3]}:{m[4]}", "%Y-%m-%d %H:%M:%S")) if m else os.path.getmtime(path)


def handeye_lines(path):
    """[handeye HH:MM:SS] lines -> (epoch, text); the date comes from the file name (midnight roll-over handled)."""
    t0 = log_start(path); day0 = time.mktime(time.strptime(time.strftime("%Y-%m-%d", time.localtime(t0)), "%Y-%m-%d"))
    out = []
    for ln in open(path, errors="replace"):
        m = re.match(r"\[handeye (\d\d):(\d\d):(\d\d)\] (.*)", ln.rstrip())
        if m:
            t = day0 + int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3])
            if t < t0 - 60: t += 86400
            out.append((float(t), m[4]))
    return out


def analyse(log, window=5.0, mon_dir=None):
    trace_p = log[:-4] + "_motors.jsonl"
    trace = jl(trace_p)
    t_start = log_start(log)
    hl = handeye_lines(log)
    # monitor folder: the newest one that started before this run
    if mon_dir is None:
        cands = sorted(glob.glob(str(HUM / "logs" / "dropmon_*")))
        cands = [c for c in cands if log_start(c) <= t_start + 5]
        mon_dir = cands[-1] if cands else None
    mon = jl(os.path.join(mon_dir, "events.jsonl")) if mon_dir else []
    R = [f"# Drop report: {os.path.basename(log)}", "",
         f"- motor trace: {os.path.basename(trace_p) if trace else 'MISSING (handeye < 3.9.4 or HANDEYE_MOTOR_TRACE unset)'}"
         f" ({sum(1 for r in trace if 'm' in r)} samples)",
         f"- monitor: {mon_dir or 'MISSING'} ({len(mon)} events)"]

    # ---- failure time: first trace error/event, else the handeye ERROR line (monitor's ms stamp if available)
    t_fail, why = None, None
    for r in trace:
        if r.get("err") or r.get("ev") in ("command_error", "frozen", "connect_failed"):
            t_fail, why = r["t"], r.get("err") or r.get("ev"); break
    he_err = [(t, x) for t, x in hl if x.startswith("ERROR")]
    mon_err = [e for e in mon if e.get("src") == "log" and e.get("file") == os.path.basename(log) and "ERROR" in e.get("text", "")]
    if t_fail is None and mon_err:
        t_fail, why = mon_err[0]["t"], mon_err[0]["text"]
    if t_fail is None and he_err:
        t_fail, why = he_err[0][0] + 0.5, he_err[0][1]
    if t_fail is None:
        R += ["", "No failure found in this run (no ERROR line, no trace error)."]
        return "\n".join(R), None
    lo, hi = t_fail - window, t_fail + window
    if mon and not (min(e.get("t", 1e18) for e in mon) <= lo and max(e.get("t", 0) for e in mon) >= t_fail):
        R += [f"- monitor {mon_dir} does not cover the failure time: ignored"]; mon = []
    R += [f"- failure at {fmt(t_fail)}: {why}"]
    pose = [x for t, x in hl if t <= t_fail + 1 and x.startswith("pose ")]
    if pose: R += [f"- during: {pose[-1]}"]

    # ---- per motor: last feedback (sequence advance) before the drop
    samples = [r for r in trace if "m" in r]
    last_adv, prev_seq, faults, last_state = {}, {}, {}, {}
    for r in samples:
        for n, d in r["m"].items():
            s = d.get("seq")
            if s is not None and s != prev_seq.get(n):
                last_adv[n] = r["t"] - float(d.get("age") or 0.0)   # driver time of the newest frame
                prev_seq[n] = s
            if d.get("fc"):
                faults.setdefault(n, (r["t"], d["fc"]))
            if "p" in d and r["t"] <= t_fail:
                last_state[n] = d
    motors = sorted(last_adv, key=lambda n: ORDER.index(n) if n in ORDER else 99)
    silent = {n: last_adv[n] for n in motors if last_adv[n] < t_fail - 0.15}
    if motors:
        R += ["", "## Last feedback per motor (driver frame time)", "", "| motor | CAN id | last frame | before failure | last pos / torque / t_mos / status |", "|---|---|---|---|---|"]
        for n in motors:
            d = last_state.get(n, {})
            R.append(f"| {n} | {d.get('id', '?')} | {fmt(last_adv[n])} | {t_fail - last_adv[n]:+.3f} s | "
                     f"{d.get('p', '?')} / {d.get('tq', '?')} Nm / {d.get('tm', '?')} C / st {d.get('st', '?')} fc {d.get('fc', '?')} |")

    # ---- timeline
    ev = []
    for r in trace:
        if "ev" in r and lo <= r["t"] <= hi:
            ev.append((r["t"], "motor", f"{r['ev']} {r.get('err', '')}".strip()))
        elif r.get("err") and lo <= r["t"] <= hi:
            ev.append((r["t"], "motor", f"error: {r['err']}"))
    for n, t in last_adv.items():
        if lo <= t <= hi and n in silent:
            ev.append((t, "motor", f"LAST frame from {n}"))
    for n, (t, fc) in faults.items():
        ev.append((t, "motor", f"FAULT {n}: " + ", ".join(v for b, v in FAULTS.items() if fc >> b & 1) + f" (0x{fc:x})"))
    usb_ev, cam_ev = [], []
    for e in mon:
        t = e.get("t", 0)
        if not (lo <= t <= hi): continue
        if e["src"] == "usb" and e["ev"] in ("detach", "attach", "reenumerate"):
            ev.append((t, "usb", f"{e['ev']} {e.get('role')} '{e.get('name')}' @{e.get('loc')}")); usb_ev.append(e)
        elif e["src"] == "cam" and e["ev"] in ("stall", "resume", "shm_error"):
            ev.append((t, "cam", f"{e['ev']} {e.get('role')} {e.get('since_s', e.get('gap_s', ''))}")); cam_ev.append(e)
        elif e["src"] == "log" and e.get("file", "").startswith("handeye"):
            ev.append((t, "log", e["text"]))
        elif e["src"] == "log":
            ev.append((t, "log", f"{e['file']}: {e['text']}"))
    if not mon:
        ev += [(t, "log", x) for t, x in hl if lo <= t <= hi]
    ev.sort(key=lambda x: x[0])
    R += ["", f"## Timeline {fmt(lo)} .. {fmt(hi)}", "", "```"] + [f"{fmt(t)} {t - t_fail:+7.3f}s  {s:5s} {x}" for t, s, x in ev] + ["```"]

    # ---- verdict
    can_usb = [e for e in usb_ev if e.get("role") == "can_adapter"]
    cam_usb = [e for e in usb_ev if str(e.get("role", "")).startswith("cam")]
    hub_usb = [e for e in usb_ev if e.get("role") == "hub"]
    V = []
    if faults:
        V.append("D  motor fault flag: " + "; ".join(f"{n}: " + ", ".join(v for b, v in FAULTS.items() if fc >> b & 1)
                                               for n, (t, fc) in faults.items()))
    if can_usb:
        V.append(f"A  CAN adapter lost on USB ({can_usb[0]['ev']} at {fmt(can_usb[0]['t'])}, {can_usb[0]['t'] - t_fail:+.2f} s)"
                 + (f"; hub event too: {hub_usb[0]['name']}" if hub_usb else ""))
    elif motors and silent and len(silent) < len(motors):
        prox = [n for n in motors if n not in silent]
        first_silent = min(silent, key=silent.get)
        V.append(f"C  only some motors went silent: {', '.join(silent)} (first: {first_silent}); still answering: "
                 f"{', '.join(prox)} -> check the cable/plug between {prox[-1] if prox else '?'} and {first_silent}")
    elif motors:
        spread = (max(last_adv.values()) - min(last_adv.values())) if last_adv else 0
        V.append(f"B  whole bus silent while the CAN adapter stayed on USB (all {len(motors)} motors stopped within "
                 f"{spread * 1000:.0f} ms) -> motor power (supply, e-stop, main connector) or CAN cable/termination "
                 f"to the first motor" + ("" if mon else " [no monitor data: USB loss not excluded]"))
    else:
        V.append("?  no motor trace: only the log is available")
    firsts = []
    if can_usb: firsts.append((can_usb[0]["t"], "CAN adapter (USB)"))
    if cam_usb: firsts.append((cam_usb[0]["t"], f"camera {cam_usb[0]['role']} (USB {cam_usb[0]['ev']})"))
    stalls = [e for e in cam_ev if e["ev"] == "stall"]
    if stalls: firsts.append((stalls[0]["t"] - stalls[0].get("since_s", 0), f"camera {stalls[0]['role']} frames stopped"))
    if silent: firsts.append((min(silent.values()), f"motor feedback ({min(silent, key=silent.get)})"))
    elif last_adv: firsts.append((max(last_adv.values()), "motor feedback (all)"))
    firsts.sort()
    if firsts:
        V.append("Order: " + " -> ".join(f"{x} {fmt(t)}" for t, x in firsts)
                 + (f".  FIRST to vanish: {firsts[0][1]}" if len(firsts) > 1 else ""))
    if mon and not usb_ev:
        V.append("No USB detach/attach in the window (all devices incl. CAN adapter and cameras stayed enumerated).")
    R += ["", "## Verdict", ""] + [f"- {v}" for v in V]
    return "\n".join(R), V


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--window", type=float, default=5.0)
    ap.add_argument("--mon")
    ap.add_argument("--out")
    a = ap.parse_args()
    log = a.log or sorted(glob.glob(str(HUM / "logs" / "handeye_*.log")))[-1]
    txt, _ = analyse(log, a.window, a.mon)
    out = a.out or log[:-4] + "_dropreport.md"
    Path(out).write_text(txt + "\n")
    print(txt); print(f"\n(saved {out})")


if __name__ == "__main__":
    main()
