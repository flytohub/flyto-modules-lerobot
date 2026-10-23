# Tasks

- [ ] Fit and test the 12 V cut-off switch on the AM-ARM200 follower supply.
- [ ] Install lerobot_alohamini on the execution host; confirm `driver.lerobot_robot` builds and connects the arm.
- [x] Commissioning tool `flyto-lerobot-arm` (check, read, jog, teach, stop-latency), tested on the fake.
- [ ] Measure joint limits (`flyto-lerobot-arm check`); teach `ready`, `fixture_pick`, `tray_place` (`jog`, `teach`); tune `motion`.
- [ ] Measure the hold-stop latency during a move (`flyto-lerobot-arm stop-latency`).
- [ ] Cloud/core: an independent "part in place zone" observation (outside this repo).
