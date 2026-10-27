# AM-ARM200 robot description

The official AM-ARM200 follower model, vendored from
[liyiteng/AM-ARM](https://github.com/liyiteng/AM-ARM) at commit
`5b03af1ef6c80595bd988e0137cf8e7c35811eb6`, under the Apache License 2.0
(`LICENSE` in this folder; attribution in the repository's `NOTICE`).

| Here | Upstream | Change |
|---|---|---|
| `urdf/am-arm200.urdf` | `am-arm200/urdf/urdf/am-arm200.urdf` | none (sha256 `a8caac56d7db400547d7c4b10e2019c8e0b8b647e2950c8adf4fa1a06aaf0913`) |
| `meshes/left_Fixed_Jaw.STL`, `left_Moving_Jaw.STL`, `left_Wrist_Pitch_Roll.STL`, `left_camera.STL` | `am-arm200/urdf/am-arm200-meshes/` | none |
| `meshes/left_Base.STL`, `left_Lower_Arm.STL`, `left_Rotation_Pitch.STL`, `left_Upper_Arm.STL`, `left_wrist_yaw.STL` | same folder | decimated |

The five upstream meshes total 35 MB (120k-252k triangles each). They were
reduced to about 12,000 triangles each, 3.4 MB for the whole folder, with
quadric edge collapse (`fast-simplification` 0.2.0, meshes read and written with `trimesh` 5.2.0):

```python
points, faces = fast_simplification.simplify(mesh.vertices, mesh.faces,
                                             target_reduction=1 - 12000 / len(mesh.faces))
```

Their bounding boxes moved by at most 5 mm (left_Rotation_Pitch; the others 3 mm or less). Meshes are drawn only; nothing
collides with them (the MuJoCo world declares collision boxes over the jaws in
the arm config's `simulation.collision`).

## What the description does not say, and what this pack assumes

- **Joint zero and direction against the real arm.** LeRobot reports joints
  in the degrees its calibration defines; how those relate to the URDF's
  angles is not published. The example configs map them one to one
  (`sign: 1`, `offset: 0`) and flag every joint `verified_on_hardware: false`;
  a real arm refuses Cartesian moves until they are confirmed.
- **Joint limits.** The URDF gives +/-3.14 rad with zero effort and velocity
  (exporter defaults). The configs' limits are placeholders.
- **Gripper.** The moving jaw is `left_gripper`; its jaw tips meet at about
  1.17 rad and stand about 7.5 cm apart at 0 rad (measured on these meshes).
  The configs map gripper reading 2 (closed) to 1.17 rad and 60 (open) to 0
  rad; unverified on hardware.
- **Masses.** The URDF's inertials come from the CAD exporter. `left_camera`
  reports 1.27 kg with its centre of mass 0.2 m away from a 36 mm camera
  mount, which cannot be right; the configs override it with an estimated
  0.03 kg (`kinematics.model.mass_overrides_kg`).

## The Pro variant

`am-arm200-pro/urdf/urdf/am-arm200pro.urdf` upstream has the same joint
origins, axes and limits as this one (its meshes differ and total 47 MB).
Upstream marks the Pro "coming soon". It is not vendored; to use it, add its
URDF and meshes as `models/am-arm200-pro/{urdf,meshes}/` (decimated the same
way) and name it in the arm config's `kinematics.model.source`
(`{"packaged": "am-arm200-pro"}`), or point `{"path": ...}` at a local copy.
Its servo classes follow lerobot_alohamini's `am-follower-6dof-hd` profile
(STS3250 on all joints but shoulder_lift and elbow_flex, which keep STS3095).
