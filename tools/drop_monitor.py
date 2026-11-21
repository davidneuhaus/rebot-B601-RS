#!/usr/bin/env python3
"""Passive drop diagnostics monitor (v3.9.4). Never touches the CAN bus, the motors or a camera device.

Writes JSON lines with millisecond wall-clock timestamps ("t", epoch seconds) to <out>/events.jsonl:
  usb      every 0.25 s `ioreg -p IOUSB -w0` (no sudo): attach / detach / re-enumerate (new registry id at the
           same location) of every USB device, with a role tag for the CAN adapter, the leader, cameras and hubs.
           The full tree is saved to usb_tree_<t>.txt on every change. (`log stream` needs admin -> not available.)
  cam      the GUI's shared camera frames (header read only, no device access): per role 1 Hz fps summary and
           "stall" / "resume" events when no new frame arrives for > 0.4 s.
  log      new lines of the newest logs/handeye_*.log, logs/recorder.log and /tmp/teleop_gui.log, stamped
           when they appear (gives ~0.1 s resolution for the handeye errors, which only print seconds).
Start/stop with tools/drop_monitor.sh. Analyse with tools/drop_report.py.
"""
import glob
import json
import os
import re
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

HUM = Path(os.environ.get("HUMANOID_DIR", str(Path.home() / "humanoid")))
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HUM / "logs" / time.strftime("dropmon_%Y-%m-%d_%H-%M-%S")
OUT.mkdir(parents=True, exist_ok=True)
LOCK = threading.Lock()
EV = open(OUT / "events.jsonl", "a", buffering=1)
STOP = threading.Event()


def emit(src, ev, **kw):
    with LOCK:
        EV.write(json.dumps({"t": round(time.time(), 3), "src": src, "ev": ev, **kw}, separators=(",", ":")) + "\n")


def role_of(name):
    n = name.lower()
    if "xcan" in n or "pcan" in n or "peak" in n or "can-usb" in n:
        return "can_adapter"
    if "innomaker" in n or "u20cam" in n:
        return "cam_wrist"
    if "c920" in n or "webcam" in n:
        return "cam_overview"
    if "serial" in n or "ch340" in n or "cp210" in n or "ftdi" in n:
        return "leader_serial"
    if "hub" in n or "dock" in n:
        return "hub"
    return "other"


LINE = re.compile(r"^(?P<ind>[\s|]*)\+-o (?P<name>.+?)@(?P<loc>[0-9a-fA-F]+)\s+<class (?P<cls>\w+), id (?P<id>0x[0-9a-f]+)")


def usb_tree():
    out = subprocess.run(["ioreg", "-p", "IOUSB", "-w0"], capture_output=True, text=True, timeout=5).stdout
    devs = {}
    for ln in out.splitlines():
        m = LINE.match(ln)
        if m and m["cls"] != "AppleT8103USBXHCI" and "XHCI" not in m["cls"]:
            devs[m["loc"]] = (m["name"], m["id"])
    return devs, out


def usb_loop():
    prev, txt = usb_tree()
    (OUT / "usb_tree_start.txt").write_text(txt)
    try:
        sp = subprocess.run(["system_profiler", "SPUSBDataType"], capture_output=True, text=True, timeout=30).stdout
        (OUT / "system_profiler_usb_start.txt").write_text(sp)
    except Exception as e:
        emit("usb", "system_profiler_failed", err=str(e))
    emit("usb", "baseline", devices={loc: {"name": n, "id": i, "role": role_of(n)} for loc, (n, i) in prev.items()})
    while not STOP.is_set():
        time.sleep(0.25)
        try:
            cur, txt = usb_tree()
        except Exception as e:
            emit("usb", "ioreg_failed", err=str(e)); continue
        changed = False
        for loc, (n, i) in prev.items():
            if loc not in cur:
                emit("usb", "detach", name=n, loc=loc, id=i, role=role_of(n)); changed = True
            elif cur[loc][1] != i:
                emit("usb", "reenumerate", name=cur[loc][0], loc=loc, old_id=i, id=cur[loc][1], role=role_of(n)); changed = True
        for loc, (n, i) in cur.items():
            if loc not in prev:
                emit("usb", "attach", name=n, loc=loc, id=i, role=role_of(n)); changed = True
        if changed:
            (OUT / f"usb_tree_{time.time():.3f}.txt").write_text(txt)
        prev = cur


def cam_loop():
    hdr = struct.Struct("<8sQdIII")
    try:
        sys.path.insert(0, str(HUM))
        import camera_feeds as cf
        prefix = cf.SHM_PREFIX
    except Exception:
        prefix = "humanoid_cam_"
    from multiprocessing import shared_memory, resource_tracker
    shms, last, stalled, count, t_sum = {}, {}, {}, {}, time.time()
    roles = ("gripper", "overview")
    while not STOP.is_set():
        now = time.time()
        for r in roles:
            if r not in shms:
                try:
                    shm = shared_memory.SharedMemory(name=prefix + r)          # attach only, never create
                    try: resource_tracker.unregister(shm._name, "shared_memory")
                    except Exception: pass
                    shms[r] = shm; emit("cam", "shm_attached", role=r)
                except Exception:
                    continue
            try:
                magic, seq, ts, w, h, _ = hdr.unpack_from(shms[r].buf, 0)
            except Exception as e:
                emit("cam", "shm_error", role=r, err=str(e)); shms.pop(r, None); continue
            if seq % 2 == 0 and seq != last.get(r, (None,))[0]:
                if stalled.get(r):
                    emit("cam", "resume", role=r, gap_s=round(now - last[r][2], 3), frame_ts=round(ts, 3))
                    stalled[r] = False
                last[r] = (seq, ts, now); count[r] = count.get(r, 0) + 1
            elif r in last and not stalled.get(r) and now - last[r][2] > 0.4:
                stalled[r] = True
                emit("cam", "stall", role=r, last_frame_ts=round(last[r][1], 3), since_s=round(now - last[r][2], 3))
        if now - t_sum >= 1.0:
            emit("cam", "fps", **{r: round(count.get(r, 0) / (now - t_sum), 1) for r in roles})
            count, t_sum = {}, now
        time.sleep(0.01)


def tail_loop():
    pos = {}
    fixed = [HUM / "logs" / "recorder.log", Path("/tmp/teleop_gui.log")]
    for f in fixed:
        if f.exists():
            pos[str(f)] = f.stat().st_size
    for f in glob.glob(str(HUM / "logs" / "handeye_*.log")):
        pos[f] = os.path.getsize(f)
    while not STOP.is_set():
        files = [str(f) for f in fixed] + glob.glob(str(HUM / "logs" / "handeye_*.log"))
        for f in files:
            try:
                size = os.path.getsize(f)
            except OSError:
                continue
            p = pos.setdefault(f, 0 if "handeye_" in f else size)
            if size < p:
                p = 0
            if size > p:
                with open(f, "rb") as fh:
                    fh.seek(p); data = fh.read(size - p)
                pos[f] = size
                for ln in data.decode("utf-8", "replace").splitlines():
                    if ln.strip():
                        emit("log", "line", file=os.path.basename(f), text=ln[:500])
        time.sleep(0.1)


def main():
    (OUT / "monitor.pid").write_text(str(os.getpid()))
    emit("mon", "start", pid=os.getpid(), out=str(OUT))
    import signal
    signal.signal(signal.SIGTERM, lambda *a: STOP.set())
    th = [threading.Thread(target=f, daemon=True, name=f.__name__) for f in (usb_loop, cam_loop, tail_loop)]
    for t in th:
        t.start()
    try:
        while not STOP.is_set():
            STOP.wait(1.0)
            for t in th:
                if not t.is_alive():
                    emit("mon", "thread_died", name=t.name)
                    th.remove(t)
    except KeyboardInterrupt:
        pass
    emit("mon", "stop")


if __name__ == "__main__":
    main()
