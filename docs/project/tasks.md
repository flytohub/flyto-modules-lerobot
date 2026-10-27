# Tasks

- [ ] Fit and test the 12 V cut-off switch on the AM-ARM200 follower supply.
- [ ] Install lerobot_alohamini on the execution host; confirm `driver.lerobot_robot` builds and connects the arm.
- [ ] Cloud: add the `manipulation.mark_pose` vocabulary row (`DECISIONS.md` has the exact row).
- [ ] Measure joint limits; teach `ready`, `fixture_pick`, `tray_place` with `manipulation.mark_pose`; tune `motion`.
- [ ] Stop a few real moves and read `stop_to_still_ms` / `overshoot` (measured on every stop).
- [ ] Cloud: add `perception.locate_objects`, `manipulation.pick`, `manipulation.place` rows (another lane).
- [ ] Cloud: pass a video frame's `source` through `live_frames._shape_video` so the browser can label it.
- [ ] Confirm each joint's sign and offset between LeRobot's calibrated reading and the URDF angle on the real arm (and the gripper's jaw-angle mapping); then mark `verified_on_hardware: true`.
- [ ] Measure the real joint limits and the gripper's closed/open readings.
- [ ] Mount and calibrate the overhead USB camera (four marked table points, colour classes, camera height).
- [ ] Make the camera its own resource so the perception specs become `other_resource`.
- [ ] Gripper effort: find whether the fork exposes a load/current reading; set `gripper.effort_key`.
- [ ] On the arm: measure `stop_to_hold_ms`, `stop_to_still_ms` and `overshoot` at the 20 ms control period before relying on servo-speed trajectories; confirm the STS acceleration (register 254 at 100 steps/s^2 per unit is an estimate).
- [ ] On the arm: find the wrist camera's device index, field of view and image orientation; confirm the lens origin on `left_camera`; teach the `look` pose; set colour classes from real frames.
- [ ] Measure the joint and forward-kinematics error bounds (`perception.camera.mount.joint_error_deg`, `fk_error_m`).
- [ ] One config with both the wrist camera and a fixed overhead camera (if the owner buys one), the fixed one as an independent observer in Cloud.

- [ ] Cloud: decide the "moved" bound with the pack -- Cloud's vocabulary says the larger of the two uncertainties, the pack uses their sum (two honest looks at a still container differ by up to the sum); accept `container_moved` as `failed` (holding) from a camera on the arm, which moves to look.
- [ ] A pick descends with the block centred between the open jaws (the fixed jaw is 9 mm clear of a 30 mm block at the tool point; servo lag swung 6-7 mm in 1 of 50 eye-in-hand episodes): take the open jaws' geometry from the robot description.
- [ ] On the arm: confirm `perception.max_uncertainty_m` and the look pose's viewpoint (`capture.viewpoint`) from real frames.
