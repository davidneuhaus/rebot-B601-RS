#!/usr/bin/env python3
"""Dry-run click-to-grasp demo: detect -> locate -> reach filter -> grasp plan. NEVER talks to the robot or the cameras.

  tools/grasp_demo.py --image vision/calibration/handeye/20261121_231523/overview_01.png --prompts "cup,blue block"
  tools/grasp_demo.py --shm overview --prompts "cup"        # newest shared frame published by the GUI (read-only)
  options: --cal-dir DIR  --segmenter auto|sam|grabcut|box|none  --rest-q "a,b,c,d,e,f"  --place "x,y,z"
           --pick N (plan only object #N, default: all)  --out DIR (default tmp/grasp_demo/<time>)

Writes <out>/annotated.png and <out>/plan.json. Without a calibration (vision/calibration/handeye.json + table.json
with table_to_robot) detection still runs and plan.json says which calibration file is missing.
"""
import argparse
import json
import sys
import time
from pathlib import Path

HOME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOME / "vision"))

import cv2                      # noqa: E402
import numpy as np              # noqa: E402

from grasp import detect as D, locate as L, grasp_plan as G      # noqa: E402


def read_shared(role):
    """Newest frame from the GUI's shared memory (attach only, nothing is opened or written)."""
    sys.path.insert(0, str(HOME))
    import camera_feeds
    rd = camera_feeds.ShmReader(role)
    for _ in range(50):
        r = rd.read()
        if r:
            rd.close()
            return r[2]
        time.sleep(0.1)
    raise SystemExit(f"no shared frame for role {role!r} (is the GUI running with cameras?)")


def build(args, bgr, detector=None, cal=None):
    """Returns (annotated image, result dict). Separated from main() so tests can call it with fakes."""
    h, w = bgr.shape[:2]
    prompts = [p.strip() for p in args.prompts.split(",") if p.strip()]
    detector = detector or D.GraspDetector(segmenter=args.segmenter)
    objs = detector.detect(bgr, prompts)
    res = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "image_size": [w, h], "prompts": prompts,
           "segmenter": detector.segmenter, "dry_run": True, "objects": [], "calibration": None}
    cfg = G.load_profiles(args.profiles)
    if cal is None:
        try:
            cal = L.load_calibration(args.cal_dir, require_handeye=False)
            res["calibration"] = cal.source
        except L.CalibrationMissing as e:
            res["calibration"] = f"MISSING: {e}"
    else:
        res["calibration"] = cal.source
    reach = None
    if isinstance(cal, L.Calibration):
        signs = tuple((cal.handeye or {}).get("joint_signs") or (1, 1, 1, 1))
        reach = L.ReachChecker(table_z_mm=cal.table_z_mm, signs=signs)
    rest_q = [float(v) for v in args.rest_q.split(",")] if args.rest_q else None
    place = {"xyz_mm": [float(v) for v in args.place.split(",")]} if args.place else None
    labels = {}
    for i, o in enumerate(objs):
        e = o.to_json()
        e["index"] = i
        if args.pick is not None and i != args.pick:
            res["objects"].append(e); continue
        pname, prof = G.pick_profile(o.label, cfg)
        e["profile"] = pname
        if not isinstance(cal, L.Calibration):
            e["plan"] = {"ok": False, "reason": "no calibration, detection only"}
            res["objects"].append(e); continue
        try:
            loc = L.locate_object(o, cal, (w, h), height_mm=prof["top_height_mm"])
            e["location"] = loc
            r = reach.check(loc["xyz_mm"], loc["jaw_yaw_deg"] or 0.0)
            e["reach"] = r.to_json()
            if not r.ok:
                e["plan"] = {"ok": False, "reason": f"reach filter: {r.reason}"}
                labels[i] = "unreachable"
            else:
                plan = G.plan_grasp(loc, o.label, reach, cal.table_z_mm, rest_q=rest_q, place=place, seed_q=r.q, cfg=cfg)
                e["plan"] = plan.to_json()
                labels[i] = (f"{len(plan.waypoints)} wp" if plan.ok else "plan failed")
        except (ValueError, L.CalibrationMissing) as ex:
            e["plan"] = {"ok": False, "reason": str(ex)}
            labels[i] = "locate failed"
        res["objects"].append(e)
    res["assumed_jaw_axis"] = L.JAW_TOOL_AXIS
    return D.draw(bgr, objs, labels), res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image"); src.add_argument("--shm", metavar="ROLE")
    ap.add_argument("--prompts", required=True)
    ap.add_argument("--cal-dir"); ap.add_argument("--profiles")
    ap.add_argument("--segmenter", default="auto")
    ap.add_argument("--rest-q"); ap.add_argument("--place"); ap.add_argument("--pick", type=int)
    ap.add_argument("--out")
    a = ap.parse_args()
    bgr = cv2.imread(a.image) if a.image else read_shared(a.shm)
    if bgr is None:
        raise SystemExit(f"cannot read image {a.image}")
    img, res = build(a, bgr)
    out = Path(a.out or HOME / "tmp" / "grasp_demo" / time.strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "annotated.png"), img)
    (out / "plan.json").write_text(json.dumps(res, indent=1))
    print(f"{len(res['objects'])} object(s); calibration: {res['calibration']}")
    for e in res["objects"]:
        p = e.get("plan", {})
        print(f"  #{e['index']} {e['label']} {e['conf']:.2f} yaw_img={e['yaw_img_deg']} -> "
              f"{'plan ok, ' + str(len(p.get('waypoints', []))) + ' waypoints' if p.get('ok') else p.get('reason', 'not planned')}")
    print(f"wrote {out}/annotated.png and plan.json (dry run, nothing sent to the robot)")


if __name__ == "__main__":
    main()
