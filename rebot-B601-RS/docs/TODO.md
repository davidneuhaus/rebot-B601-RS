# TODO – ideas and next steps

Open items and ideas for the reBot B601-RS / humanoid project. Anything that costs money is cleared with David
first (price before ordering).

## Open items

- [ ] **Hardware emergency stop**: mushroom e-stop switch in the 48 V line (approx. €20–40). Today the power strip is
      the only e-stop.
- [ ] **Camera setup**: choose and mount camera(s) for recording (wrist camera and/or scene camera, e.g. a depth
      camera); configure LeRobot cameras.
- [ ] **Record demonstrations**: `lerobot-record` with ~50 demos of one task (leader → follower, with camera).
- [ ] **Training**: train a policy (e.g. ACT) on the demos; cloud GPU price to be approved by David before booking.
- [ ] **Mobile base for the B601**: needs 48 V and enough payload, e.g. AgileX Ranger Mini 3 or AgileX Tracer 2.0.
      Hobby mecanum kits are ruled out (voltage, payload, robustness).
- [ ] Verify guard v3.1 thresholds on the real arm (free motion, blocks on every joint) and adjust from `Guard-Stats`.

## Collision safety – beyond fixed thresholds

- [ ] **Model-based collision detection**: momentum observer / expected-torque model (gravity per joint from the
      current pose + dynamics: inertia, Coriolis, friction). Compare measured vs expected torque instead of fixed
      per-joint thresholds; residual above a band = contact. Removes the false alarms from gravity load and fast moves.
- [ ] **Payload estimation after a grasp**: estimate the object's mass from the torque change after closing the
      gripper, so lifting an object does not trigger the guard.
- [ ] **Context awareness**: intended task (approach / grasp / transport), expected contacts vs unexpected ones;
      contact location – wrist force/torque sensor (contact at the tool) vs link contact (arm body hits something).
- [ ] **Vision for pre-contact slowdown**: depth camera / lidar watches the workspace; slow down or stop before a
      person or object is touched.
- [ ] **Speed & separation monitoring / power & force limiting per ISO/TS 15066**: speed limit depending on the
      distance to people, body-region force/pressure limits.
- [ ] **Humanoid extension**: balance control, foot force sensors, IMU; whole-body collision and fall detection.

## Software / tooling

- [ ] Guard-Stats with full per-cycle traces (CSV) for better offline tuning and replay with torque.
- [ ] Optional: let the GUI show the current guard settings and live per-joint offsets.


## After v3.2 (2026-11-15)

- [ ] **Real-arm test of v3.2** (guard + `GRAVITY_FF` checkbox) following the procedure in the v3.2 report; then
      decide whether the feedforward becomes the default.
- [ ] **shoulder_pan torque freezes at 1.6–2.3 Nm** (20:48 session) while the shoulder sagged – check whether they
      disappear with `GRAVITY_FF=1`; otherwise raise `COLLISION_TORQ_SHOULDER_PAN` or add a dynamics term.
- [ ] Full dynamics (inertia/Coriolis) in the residual – today only gravity is modelled.
- [ ] Wire the iPhone cameras (stream1/stream2) into LeRobot recording.

## Laptop

- [x] PCBUSB in `~/humanoid/lib` (user space).
- [ ] Run `preflight.py` with the arm powered.
- [ ] Calibration files from the Mac mini or a fresh calibration.
- [ ] Choose cameras on the laptop (iPhone bridge or Innomaker USB) in `cameras.json`.
- [ ] Optional: SmolVLA extras (`transformers`) on the laptop.

## Auto calibration (3.9)

- [ ] First real run of 🤖 Auto-Kalibrierung with David at the arm (hand on Stop): board taped, teach, plan, run.
      Check the numbers (OPERATING_PROCEDURE.md section 8), then press 📐 Neu kalibrieren once.
- [ ] If the solver reports a different joint sign convention: fix robot_fk / calibrate.SIGN_COMBOS default.
- [ ] Validate the result: detect an object in the overview, convert to robot coordinates (pixel_to_table ->
      table_to_robot_xyz), hover the gripper 30 mm above it in a slow test move.
- [ ] Overview intrinsics are from a rushed 35 s capture (rms 0.81 px): redo with varied board poses
      (or reuse the auto-calibration idea: board held still, many views).
- [ ] Optional: let the planner refine the aim after the first 3 samples (better coverage of the image corners).
