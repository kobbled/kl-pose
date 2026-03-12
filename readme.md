# lib/pose

Kinematics, coordinate transforms, and pose arithmetic for FANUC Karel. Central to the UV→XYZWPR slicer pipeline.

---

## Overview

`pose` solves four related problems that Karel's built-ins don't cover well:

1. **Kinematics** — `pose__solveIK` / `pose__solveK` for Cartesian↔Joint conversion.
2. **Coordinate system conversions** — cylindrical (θ,z,r) and polar (θ,ρ,r) to/from Cartesian, essential for printing on curved surfaces.
3. **Pose arithmetic** — composition, inversion, interpolation, scalar ops — both in Karel and via TP-callable wrappers.
4. **Frame construction from geometry** — build frames aligned to measured surface normals using quaternion (gimbal-lock free).

You reach for this module whenever you need to work with robot positions beyond basic PR read/write.

---

## Files

| File | Purpose |
|------|---------|
| `lib/poselib/pose.kl` | Main implementation (~1400 lines): IK/FK, frame ops, coordinate conversions, circle geometry |
| `lib/poselib/pose.klh` | Function declarations — include in any module using poselib |
| `lib/poselib/posetp.kl` | TP-callable arithmetic wrappers (composition, inversion, frame ops) |
| `lib/poselib/posetp.klh` | TP wrapper declarations |
| `lib/poselib/pose.klt` | Includes pose.const.klt + pose_structs.klt |
| `lib/poselib/pose.const.klt` | Constants: `MAX_AXS`, `MAX_GRPS`, axis codes, PR type codes |
| `lib/poselib/pose_structs.klt` | Types: `T_CIRCLE`, `t_AXES_FRAME` |
| `lib/poselib/pose_type.klt` | `t_node` template: dual XYZWPR+JOINTPOS struct |
| `lib/poseclass/pose.klc` | OOP class wrapping dual Cartesian/joint representation |
| `lib/matpose/matpose.kl` | 4×4 homogeneous matrix ops: rotx/roty/rotz, transl, pose↔matrix |
| `lib/matpose/matpose.klh` | matpose declarations |
| `lib/matpose/matrot.kl` | 3×3 rotation-only matrix ops |
| `lib/matpose/quaternion.kl` | Quaternion arithmetic |
| `lib/matpose/quaternion.klh` | Quaternion declarations |
| `lib/matpose/quaternion.klt` | `T_QUAT` struct definition |
| `lib/matpose/matArr.klt` | 4×4 REAL matrix template config (`t_matarr`) |
| `lib/matpose/rotArr.klt` | 3×3 REAL rotation matrix template config (`t_rotarr`) |
| `test/test_pose.kl` | KUnit tests: IK/FK round-trip, constructors, cylindrical, circumcenter |
| `test/test_matpose.kl` | KUnit tests: matpose and quaternion |
| `build_notes.md` | Deploy order warning — read before building |

---

## API Reference

### Types

```karel
TYPE T_CIRCLE FROM pose = STRUCTURE
    radius : REAL
    center : VECTOR
ENDSTRUCTURE
-- Returned by pose__find_circumcenter

TYPE t_AXES_FRAME FROM pose = STRUCTURE
    orient   : VECTOR   -- tool orientation axis
    approach : VECTOR   -- approach direction
    normal   : VECTOR   -- surface normal
ENDSTRUCTURE

TYPE T_QUAT FROM matpose = STRUCTURE
    w : REAL   -- scalar component
    x : REAL   -- i
    y : REAL   -- j
    z : REAL   -- k
ENDSTRUCTURE
```

### Constants

```karel
-- Axis reference codes (for z_axis parameter in coordinate conversions)
X_AXES   = 1
Y_AXES   = 2
Z_AXES   = 3
VERT_AXES = 4

-- PR representation codes (returned by pose__get_posreg_rep)
CC_POSITION = 1
CC_XYZWPR   = 2
CC_EXT      = 6
CC_JOINT    = 9
```

---

### poselib — Core Routines

#### Kinematics

```karel
-- Inverse kinematics: Cartesian position → joint angles
pose__solveIK(pose_in: XYZWPR; grp_no: INTEGER) : JOINTPOS

-- Forward kinematics: joint angles → Cartesian position
pose__solveK(jpos: JOINTPOS; grp_no: INTEGER) : XYZWPR
```

Always check `pose__get_ok()` after IK — solutions may fail near singularities.

#### User Frame / Tool Frame

```karel
pose__set_userframe(frm_no, grp_no: INTEGER)
pose__get_userframe(grp_no: INTEGER) : XYZWPR
pose__set_toolframe(frm_no, grp_no: INTEGER)
pose__get_toolframe(grp_no: INTEGER) : XYZWPR
pose__update_uframe(frame: XYZWPR; frm_no, grp_no: INTEGER)  -- write + activate
pose__update_utool(frame: XYZWPR; frm_no, grp_no: INTEGER)   -- write + activate
```

#### Position Register I/O

```karel
pose__get_posreg_xyz(reg_no, grp_no: INTEGER) : XYZWPR
pose__get_posreg_joint(reg_no, grp_no: INTEGER) : JOINTPOS
pose__get_posreg_rep(reg_no, grp_no: INTEGER) : INTEGER  -- CC_POSITION, CC_JOINT, etc.
pose__set_posreg_xyz(pose_in: XYZWPR; reg_no, grp_no: INTEGER)
pose__set_posreg_joint(pose_in: JOINTPOS; reg_no, grp_no: INTEGER)

-- Selective update — leave the other components untouched:
pose__mask_posreg_xyz(x, y, z: REAL; reg_no, grp_no: INTEGER)
pose__mask_posreg_orient(w, p, r: REAL; reg_no, grp_no: INTEGER)

pose__set_vector_to_posreg(vec: VECTOR; reg_no: INTEGER)  -- xyz only, w/p/r = 0
```

#### Pose Constructors

```karel
-- From array
pose__set_xyzwpr(arr: ARRAY[*] OF REAL; conf_str: STRING) : XYZWPR
pose__set_jointpos(arr: ARRAY[*] OF REAL) : JOINTPOS

-- From comma-delimited string
pose__set_xyzwpr_str('-214.806,1335.678,330.26,115.919,-89.833,65.080', 'F U T, 0, 0, 0')
pose__set_jointpos_str('106.304,11.256,-42.688,-100.565,101.138,46.420')

-- Utility
pose__set_jointpos_axis(pose: JOINTPOS; axs: INTEGER; ang: REAL) : JOINTPOS
pose__zero_jointpos() : JOINTPOS
pose__set_config(conf_str: STRING) : CONFIG
pose__replace_config(p: XYZWPR; conf: CONFIG) : XYZWPR
```

#### Extraction & Conversion

```karel
pose__pose_to_vector(pose: XYZWPR) : VECTOR   -- extract xyz as VEC
pose__pose_to_orient(pose: XYZWPR) : VECTOR   -- extract wpr as VEC
pose__get_orientation(pose: XYZWPR) : VECTOR  -- alias for pose_to_orient
pose__vector_to_pose(v, orient: VECTOR; cnf: CONFIG) : XYZWPR

pose__replace_orient(pose: XYZWPR; orient: VECTOR) : XYZWPR  -- keep xyz, replace wpr

pose__get_lpos(grp_no: INTEGER) : XYZWPR     -- current robot position (Cartesian)
pose__get_jpos(grp_no: INTEGER) : JOINTPOS   -- current robot position (joint)
pose__get_ok() : BOOLEAN                     -- last operation success flag

pose__norm(v: VECTOR) : VECTOR               -- normalize to unit length
pose__dot(v1, v2: VECTOR) : REAL             -- dot product
```

#### Coordinate Conversions

```karel
-- Cylindrical (theta deg, z mm, r mm) → Cartesian
pose__cylindrical_to_cartesian(origin: XYZWPR; cyl_pose: XYZWPR; z_axis: INTEGER) : XYZWPR

-- Cartesian → Cylindrical
pose__cartesian_to_cylindrical(origin: XYZWPR; cart_pose: XYZWPR; z_axis: INTEGER; radius: REAL; use_mm: BOOLEAN) : XYZWPR

-- Polar → Cartesian
pose__polar_to_cartesian(origin: XYZWPR; pol_pose: XYZWPR; z_axis: INTEGER) : XYZWPR

-- Transform a cylindrical basis vector to Cartesian
pose__cylindrical_to_cartesian_vector(theta: REAL; theta_hat, rad_hat, z_hat: REAL; vecAxis: INTEGER) : VECTOR

-- Move surface point back to cylinder origin along radial direction
pose__cylinder_surf_to_origin(pos: XYZWPR; n_pol: VECTOR) : XYZWPR
```

`z_axis` values: `X_AXES=1`, `Y_AXES=2`, `Z_AXES=3`, `VERT_AXES=4`.

#### Euler / Frame from Vectors

```karel
-- Convert direction vector → WPR Euler angles
-- Use vector_to_euler2 (quaternion-based) unless you need the legacy version
pose__vector_to_euler(vi, vj, vk: REAL; vectorAxis: INTEGER) : VECTOR
pose__vector_to_euler2(v: VECTOR; vectorAxis: INTEGER) : VECTOR  -- gimbal-lock safe

-- For cylindrical UV tangent vectors
pose__vector_to_euler_cylindrical(tangent: VECTOR; theta: REAL) : VECTOR

-- Build frame from two measured surface points (p1=origin, p2=point along normal)
pose__create_frame_from_normal(p1, p2: VECTOR; parentfrm: XYZWPR) : XYZWPR

-- Align active user frame axis to measured surface (quaternion rotation)
pose__correctFrame(crrAxisNo: INTEGER; p1, p2: XYZWPR)
```

#### Geometry

```karel
-- Circle/arc geometry
pose__find_circle_center(points: ARRAY[*] OF VECTOR) : VECTOR
pose__find_circumcenter(p1, p2, p3: VECTOR) : T_CIRCLE  -- returns { center, radius }

-- Interpolation and distance
pose__line_increment(p1, p2: VECTOR; fraction: REAL) : VECTOR  -- p1 + (p2-p1)*fraction
pose__distance(pose1, pose2: XYZWPR) : REAL                    -- position distance only

-- Convert matrix-type PR to Cartesian in-place
pose__matrix_to_cart(reg_no, grp_no: INTEGER)
```

#### Joint Operations

```karel
pose__jpos_add(J1, J2: JOINTPOS) : JOINTPOS  -- element-wise add
pose__jpos_sub(J1, J2: JOINTPOS) : JOINTPOS  -- element-wise subtract
pose__jpos_to_jpos6(pose: JOINTPOS) : JOINTPOS6
pose__jpos_to_jpos3(pose: JOINTPOS) : JOINTPOS3
pose__jpos_to_jpos2(pose: JOINTPOS) : JOINTPOS2
```

---

### posetp — TP-Callable Wrappers

Each routine has a short TP program name (e.g., `pos_mult`). TP programs call these via AR[] arguments.

| Operation | TP Name | Karel Routine |
|-----------|---------|---------------|
| Pose composition p1:p2 | `pos_mult` | `posetp__matmul` |
| Pose addition | `pos_add` | `posetp__add` |
| Pose subtraction | `pos_sub` | `posetp__sub` |
| Scalar multiply | `pos_sclmult` | `posetp__scalar_mult` |
| Scalar divide | `pos_scldiv` | `posetp__scalar_divide` |
| Pose inversion | `pos_inv` | `posetp__inv` |
| Dot product | `pos_dot` | `posetp__dot` |
| Cross product | `pos_cross` | `posetp__cross` |
| 3-point frame | `pos_frame` | `posetp__frame` |
| 4-point frame | `pos_frame4` | `posetp__frame4` |
| Frame from vectors | `pos_frmvec` | `posetp__framevec` |
| Circumcenter | `pos_center` | `posetp__find_center` |
| Frame from normal | `pos_linefrm` | `posetp__create_frame_from_normal` |
| Linear interpolation | `pos_lineinc` | `posetp__line_increment` |
| Set PR (cartesian) | `pos_setxyz` | `posetp__set_pr_xyz` |
| Set PR (6 joints) | `pos_setjnt6` | `posetp__set_pr_jpos6` |
| Clear PR | `pos_clrpr` | `posetp__clrpr` |
| Vector → Euler | `pos_vec2eul` | `pose__vector_to_euler2` |
| Frame correction | `pos_crtFrm` | `pose__correctFrame` |
| Euclidean distance | `pos_dist` | `pose__distance` |

Full list of all 68 TP interfaces: see `package.json`.

---

### matpose — 4×4 Matrix Operations

```karel
-- Elementary matrices
matpose__rotx(angle: REAL) : t_matarr   -- rotation around X (degrees)
matpose__roty(angle: REAL) : t_matarr   -- rotation around Y
matpose__rotz(angle: REAL) : t_matarr   -- rotation around Z
matpose__transl(x, y, z: REAL) : t_matarr  -- pure translation

-- Conversions (FANUC ZYX / RPY convention: W=yaw, P=pitch, R=roll)
matpose__pose_to_mat(pose: XYZWPR; out_mat: t_matarr)  -- in/out param
matpose__mat_to_pose(mat: t_matarr) : XYZWPR
```

Matrix format:
```
[R11 R12 R13  tx]
[R21 R22 R23  ty]
[R31 R32 R33  tz]
[0   0   0    1 ]
```

To compose transforms, use the `matrix` class `mult` method: `result = mat_a.mult(mat_b)`.

---

### quaternion

```karel
quaternion__set(w, x, y, z: REAL) : T_QUAT
quaternion__normalize(q: T_QUAT) : T_QUAT
quaternion__conj(q: T_QUAT) : T_QUAT              -- (w, -x, -y, -z)
quaternion__mult(q1, q2: T_QUAT) : T_QUAT         -- Hamilton product
quaternion__pose_to_quat(p: XYZWPR) : T_QUAT      -- ZYX Euler → quaternion
quaternion__quat_to_pose(q: T_QUAT) : XYZWPR      -- quaternion → WPR (xyz = 0)
quaternion__mat_to_quat(mat: t_matarr) : T_QUAT   -- from 4×4 rotation block
quaternion__quat_to_mat(q: T_QUAT; out_mat: t_matarr)  -- fill rotation block
quaternion__test(expected, actual: T_QUAT) : BOOLEAN    -- KUnit fuzzy compare
```

Note: `quaternion__quat_to_pose` only fills W, P, R. Combine with `pose__vector_to_pose` to get a full XYZWPR.

---

## Common Patterns

### Pattern 1: IK for Path Planning

When you have a planned Cartesian waypoint and need to verify or execute it as joint motion:

```karel
VAR
    target  : XYZWPR
    jpos    : JOINTPOS

BEGIN
    pose__set_userframe(1, 1)               -- activate user frame 1, group 1
    target = pose__get_posreg_xyz(10, 1)    -- read target from PR[10]
    jpos = pose__solveIK(target, 1)         -- compute IK

    IF NOT pose__get_ok THEN
        karelError(INVALID_INDEX, 'IK failed', ER_ABORT)
    ENDIF

    pose__set_posreg_joint(jpos, 11, 1)     -- store result in PR[11]
END
```

### Pattern 2: Cylindrical Coordinate Mapping

For printing on a cylindrical surface. The slicer outputs (θ, z, r) in `cyl_pose`, and you need robot (x, y, z, w, p, r):

```karel
VAR
    origin    : XYZWPR   -- cylinder axis frame from calibration
    cyl_pose  : XYZWPR   -- (theta_deg=x, z_mm=y, r_mm=z, w, p, r)
    cart_pose : XYZWPR

BEGIN
    -- Load cylinder origin frame from PR[1]
    origin = pose__get_posreg_xyz(1, 1)

    -- cyl_pose.x = angle in degrees, .y = height, .z = radius
    cart_pose = pose__cylindrical_to_cartesian(origin, cyl_pose, Z_AXES)
    -- Z_AXES: cylinder rotates around Z axis
    -- Result is in robot world frame, ready for motion
END
```

### Pattern 3: Surface Normal Alignment (pose__correctFrame)

Used during printer calibration. Operator touches tool tip to two points on the print surface. This aligns the active user frame so the specified axis is perpendicular to the surface:

```karel
-- p1, p2 come from operator teach points
VAR
    p1, p2 : XYZWPR

BEGIN
    p1 = pose__get_posreg_xyz(1, 1)
    p2 = pose__get_posreg_xyz(2, 1)
    pose__correctFrame(Z_AXES, p1, p2)
    -- Active user frame now updated; tool Z-axis is aligned to surface normal
END
```

Internally, `correctFrame` builds a quaternion from the cross product of the current tool axis and the measured normal vector — no Euler angle singularities.

### Pattern 4: Building a Pose from a Path Tangent Vector

Common in `pathlib` when converting 2D path tangent vectors to full 3D robot orientations:

```karel
VAR
    tangent : VECTOR
    wpr     : VECTOR
    pos     : VECTOR
    pose    : XYZWPR
    cfg     : CONFIG

BEGIN
    tangent = VEC(0.707, 0.707, 0.0)
    wpr = pose__vector_to_euler2(tangent, Z_AXES)   -- quaternion-based, no gimbal lock
    pos = VEC(100.0, 200.0, 300.0)
    cfg = pose__set_config('F U T, 0, 0, 0')
    pose = pose__vector_to_pose(pos, wpr, cfg)
END
```

### Pattern 5: Selective PR Update

When you want to update only position or only orientation without disturbing the other:

```karel
BEGIN
    -- Move robot height without changing orientation or XY position
    pose__mask_posreg_xyz(pos.x, pos.y, new_z, 5, 1)

    -- Rotate tool without changing tip position
    pose__mask_posreg_orient(new_w, new_p, new_r, 5, 1)
END
```

### Pattern 6: Frame Composition with 4×4 Matrices

When building multi-frame transforms (e.g., tool offset relative to a tilted surface frame):

```karel
VAR
    tool_rot : t_matarr
    offset   : t_matarr
    combined : t_matarr
    result   : XYZWPR

BEGIN
    tool_rot = matpose__rotz(90.0)
    offset   = matpose__transl(0.0, 50.0, 0.0)
    combined = offset.mult(tool_rot)         -- compose: translate after rotate
    result   = matpose__mat_to_pose(combined)
END
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Using `pose__vector_to_euler` near ±90° pitch | WPR jumps, path discontinuities | Use `pose__vector_to_euler2` — quaternion-based, no gimbal lock |
| Wrong `z_axis` in cylindrical conversion | Positions at wrong angles/heights | `Z_AXES=3` = local Z is rotation axis; `VERT_AXES=4` = world vertical |
| Not checking `pose__get_ok()` after IK | Silently wrong joint positions | Always check; abort if FALSE |
| Reading a joint-mode PR with `pose__get_posreg_xyz` | Error or garbage data | Check `pose__get_posreg_rep(reg_no, grp_no)` first |
| `quaternion__quat_to_pose` expecting full XYZWPR | Gets (0,0,0) position | It returns WPR only; use `pose__vector_to_pose(pos, orient, cfg)` to combine |
| Deploying without removing `shapes.pc` / `draw.pc` | `MEMO-128 parameters are different` | Run `master_del.bat` + `master_test_del.bat` first |
| FANUC Euler convention mismatch with 4×4 matrix | Rotation appears transposed | FANUC uses ZYX/RPY: W=yaw(Z), P=pitch(Y), R=roll(X) |

---

## Build Flow

```
ktransw-macros → errors → Strings → math → shapes → matrix → pose
```

```shell
cd lib/pose
del /f robot.ini && setrobot
mkdir build && cd build
rossum .. -w -o           # resolve deps, write build.ninja
ninja                     # compile pose.kl, posetp.kl, matpose.kl, quaternion.kl → .pc
kpush                     # deploy to controller
```

**Before deploying:** delete `shapes.pc` and `draw.pc` from the controller. See `build_notes.md`.

To include tests: `rossum .. -w -o -t`

See the [Ka-Boost readme](../../readme.md) for full build instructions.
