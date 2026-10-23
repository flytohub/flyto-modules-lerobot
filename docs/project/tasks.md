# Tasks

- [ ] Fit and test the 12 V cut-off switch on the AM-ARM200 follower supply.
- [ ] Install lerobot_alohamini on the execution host; confirm `driver.lerobot_robot` builds and connects the arm.
- [ ] Cloud: add the `manipulation.mark_pose` vocabulary row (`DECISIONS.md` has the exact row).
- [ ] Measure joint limits; teach `ready`, `fixture_pick`, `tray_place` with `manipulation.mark_pose`; tune `motion`.
- [ ] Stop a few real moves and read `stop_to_still_ms` / `overshoot` (measured on every stop).
- [ ] Cloud/core: an independent "part in place zone" observation (outside this repo).
