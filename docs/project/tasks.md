# Tasks

- [ ] Fit and test the 12 V cut-off switch on the AM-ARM200 follower supply.
- [ ] Install lerobot_alohamini on the execution host; confirm `driver.lerobot_robot` builds and connects the arm.
- [ ] Cloud: add the `manipulation.mark_pose` vocabulary row (`DECISIONS.md` has the exact row).
- [ ] Measure joint limits; teach `ready`, `fixture_pick`, `tray_place` with `manipulation.mark_pose`; tune `motion`.
- [ ] Stop a few real moves and read `stop_to_still_ms` / `overshoot` (measured on every stop).
- [ ] Cloud: add `perception.locate_objects`, `manipulation.pick`, `manipulation.place` rows (another lane).
- [ ] Cloud: pass a video frame's `source` through `live_frames._shape_video` so the browser can label it.
- [ ] Measure the real AM-ARM200 kinematic chain; write it into its arm config.
- [ ] Mount and calibrate the overhead USB camera (four marked table points, colour classes, camera height).
- [ ] Make the camera its own resource so the perception specs become `other_resource`.
- [ ] Gripper effort: find whether the fork exposes a load/current reading; set `gripper.effort_key`.
