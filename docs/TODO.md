# TODO

## Grasp pipeline (after calibration)
Roadmap for a general click-to-grasp pipeline (nothing here is apple-specific). Needs the hand-eye result
(`vision/calibration/handeye.json` + `table.json` with `table_to_robot`). Code lives in `vision/grasp/`.
Status: [x] delivered as dry-run groundwork (no motor code), [ ] open.

1. [x] Free-text object detection – YOLO-World prompts (`vision/grasp/detect.py`, `GraspDetector`).
2. [x] Segmentation outline + grasp yaw – MobileSAM (weights in `vision/models/mobile_sam.pt`) with GrabCut/box fallback;
       yaw from the min-area rectangle short axis.
3. [x] Pixel -> 3D on the table plane – `vision/grasp/locate.py` (overview camera; wrist camera via handeye.json).
       Open: verify on the real arm after calibration (one test point), height of objects is taken from the profile.
4. [x] Reach filter – IK via `handeye_plan.Kin` (`ReachChecker`). Open: check on hardware; jaw axis (`y` vs `z`) is assumed.
5. [ ] Freeze -> select -> run UI – freeze detections, click an object, run the plan (GUI work, owner of teleop_gui.py).
6. [~] Grasp sequence – planned as joint waypoints (`grasp_plan.py`, dry run). Open: executor with speed limits + stop.
7. [x] Height lock – min tip z = table_z + 12 mm in the planner (`min_tip_clearance_mm`); executor must enforce it too.
8. [~] Gripper success check – spec only (`success_check` in plan.json). Open: evaluate it in the executor; calibrate
       the gripper mm -> deg map (`profiles.json` gripper.calibrated=false).
9. [x] Grasp profiles per object type – `vision/grasp/profiles.json` (width, insert depth, release width, approach-open).
10. [ ] Live camera feed during motion (preview of what the wrist/overview camera sees while the arm moves).
11. [ ] Post-calibration fine-correction offset – one test grasp, store a small XYZ offset applied in `locate.py`.
12. [ ] Manual gravity-compensation calibration fallback – push the arm by hand, press Enter at each pose.
13. [ ] Depth camera (e.g. RealSense D405 / Orbbec) – real object heights. Needs David's price approval before ordering.

Delivered with the groundwork: `tools/grasp_demo.py` (dry run on a saved image or a GUI shared frame, writes
annotated image + plan.json) and `vision/grasp/tests` (pytest, synthetic images + fake calibration).
