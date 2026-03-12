# lib/pose — CLAUDE.md

## Purpose

`pose` is the kinematics and coordinate-transform backbone of Ka-Boost. It provides:
- **Inverse / Forward kinematics** — `pose__solveIK` / `pose__solveK`
- **XYZWPR pose arithmetic** — composition, inversion, interpolation, scalar ops
- **Coordinate system conversions** — Cartesian ↔ cylindrical ↔ polar (critical for printing on curved/rotary surfaces)
- **Euler angle / frame construction from surface normals** — using quaternion internally to avoid gimbal lock
- **4×4 homogeneous transformation matrices** — `matpose` sub-module
- **Quaternion arithmetic** — `quaternion` sub-module
- **Position register (PR) I/O** — read/write/mask FANUC PR[] registers
- **TP-callable arithmetic wrappers** — `posetp` sub-module

Layer 6 in the Ka-Boost dependency stack. Consumed by `paths`, `draw`, `sensors`, `registers`, and `shapes`.

---

## Repository Layout

```
lib/pose/
├── package.json                  rossum manifest — 68 TP-interfaces declared
├── readme.md                     developer overview
├── CLAUDE.md                     this file
├── build_notes.md                IMPORTANT: shapes/draw must be removed before deploy
│
├── lib/poselib/                  core implementation
│   ├── pose.kl                   ~1400-line main implementation
│   ├── pose.klh                  function declarations (include in dependents)
│   ├── posetp.kl                 TP-callable wrappers (Karel routines callable from TP via AR[])
│   ├── posetp.klh                TP declarations
│   ├── pose.klt                  includes: pose.const.klt + pose_structs.klt
│   ├── pose.const.klt            axis codes, position type codes, constants
│   ├── pose_structs.klt          T_CIRCLE, t_AXES_FRAME type definitions
│   └── pose_type.klt             t_node dual-rep struct template (XYZWPR + JOINTPOS)
│
├── lib/poseclass/                OOP class wrapper
│   ├── pose.klc                  class template (pose_get_xyz, pose_get_joint, etc.)
│   ├── pose_class.klh            class declarations
│   └── pose_type.klt             class config (t_node template)
│
├── lib/matpose/                  4×4 matrix operations
│   ├── matpose.kl                rotx/roty/rotz/transl + pose↔matrix conversions
│   ├── matpose.klh               declarations
│   ├── matrot.kl                 3×3 rotation-only matrix operations
│   ├── matrot.klh                declarations
│   ├── quaternion.kl             quaternion arithmetic
│   ├── quaternion.klh            declarations
│   ├── quaternion.klt            T_QUAT struct definition
│   ├── matArr.klt                4×4 REAL matrix template config (t_matarr)
│   └── rotArr.klt                3×3 REAL rotation matrix template config (t_rotarr)
│
└── test/
    ├── test_pose.kl              KUnit test suite (~400 lines)
    └── test_matpose.kl           matpose/quaternion tests
```

---

## Types

### `T_CIRCLE` (pose_structs.klt)
```
TYPE T_CIRCLE FROM pose = STRUCTURE
    radius : REAL
    center : VECTOR
ENDSTRUCTURE
```
Returned by `pose__find_circumcenter`.

### `t_AXES_FRAME` (pose_structs.klt)
```
TYPE t_AXES_FRAME FROM pose = STRUCTURE
    orient   : VECTOR    -- tool orientation axis
    approach : VECTOR    -- approach axis
    normal   : VECTOR    -- surface normal axis
ENDSTRUCTURE
```

### `T_QUAT` (quaternion.klt)
```
TYPE T_QUAT FROM matpose = STRUCTURE
    w : REAL    -- scalar
    x : REAL    -- i component
    y : REAL    -- j component
    z : REAL    -- k component
ENDSTRUCTURE
```

### `t_node` (pose_type.klt — template)
```
TYPE t_node FROM <object_name> = STRUCTURE
    t_xyz   : XYZWPR
    t_joint : JOINTPOS
ENDSTRUCTURE
```
Dual representation: both Cartesian and joint-space simultaneously. Used by `poseclass`.

### Matrix types
- `t_matarr` — 4×4 REAL (from matArr.klt). Row-major homogeneous transform: [R|t; 0 1].
- `t_rotarr` — 3×3 REAL (from rotArr.klt). Rotation only.

### Constants (pose.const.klt)
```
MAX_AXS   = 9     -- max JOINTPOS axes (up to J9)
MAX_GRPS  = 5     -- max robot groups

-- PR representation codes (from pose__get_posreg_rep)
CC_POSITION = 1
CC_XYZWPR   = 2
CC_EXT      = 6
CC_JOINT    = 9

-- Axis reference codes (for z_axis parameter in coordinate conversions)
X_AXES   = 1
Y_AXES   = 2
Z_AXES   = 3
VERT_AXES = 4
```

---

## Full API Reference

### poselib — Core Kinematics

#### IK / FK
```
pose__solveIK(pose_in: XYZWPR; grp_no: INTEGER) : JOINTPOS
    -- Inverse kinematics: Cartesian position → joint angles
    -- Uses FANUC CALC_JPOS_DATA. Sets global ok flag.

pose__solveK(jpos: JOINTPOS; grp_no: INTEGER) : XYZWPR
    -- Forward kinematics: joint angles → Cartesian
    -- Uses FANUC CALC_KINE_DATA.
```

#### User Frame / Tool Frame
```
pose__set_userframe(frm_no, grp_no: INTEGER)
    -- SET_UFRAME: activate user frame number frm_no for group grp_no

pose__get_userframe(grp_no: INTEGER) : XYZWPR
    -- Return current UFRAME[frm_no] for group grp_no

pose__set_toolframe(frm_no, grp_no: INTEGER)
    -- SET_UTOOL: activate tool frame number frm_no

pose__get_toolframe(grp_no: INTEGER) : XYZWPR
    -- Return current UTOOL[frm_no]

pose__update_uframe(frame: XYZWPR; frm_no, grp_no: INTEGER)
    -- Write frame data to UFRAME[frm_no] and activate it

pose__update_utool(frame: XYZWPR; frm_no, grp_no: INTEGER)
    -- Write frame data to UTOOL[frm_no] and activate it
```

#### Position Register I/O
```
pose__get_posreg_xyz(reg_no, grp_no: INTEGER) : XYZWPR
pose__get_posreg_joint(reg_no, grp_no: INTEGER) : JOINTPOS
pose__get_posreg_rep(reg_no, grp_no: INTEGER) : INTEGER
    -- Returns CC_POSITION, CC_JOINT, CC_EXT, etc.

pose__set_posreg_xyz(pose_in: XYZWPR; reg_no, grp_no: INTEGER)
pose__set_posreg_joint(pose_in: JOINTPOS; reg_no, grp_no: INTEGER)

pose__mask_posreg_xyz(x, y, z: REAL; reg_no, grp_no: INTEGER)
    -- Update x,y,z only — leaves w,p,r and config unchanged

pose__mask_posreg_orient(w, p, r: REAL; reg_no, grp_no: INTEGER)
    -- Update w,p,r only — leaves x,y,z and config unchanged

pose__set_vector_to_posreg(vec: VECTOR; reg_no: INTEGER)
    -- Write (x,y,z) to PR; w,p,r set to 0
```

#### Pose Constructors
```
pose__set_xyzwpr(arr: ARRAY[*] OF REAL; conf_str: STRING) : XYZWPR
    -- Construct from 6-element REAL array [x,y,z,w,p,r] + config string

pose__set_xyzwpr_str(pose_in: STRING; conf_str: STRING) : XYZWPR
    -- Construct from comma-delimited string '214.806,1335.678,...'

pose__set_jointpos(arr: ARRAY[*] OF REAL) : JOINTPOS
    -- Construct JOINTPOS from REAL array

pose__set_jointpos_str(pose_in: STRING) : JOINTPOS
    -- Construct JOINTPOS from comma-delimited string '106.304,11.256,...'

pose__set_jointpos_axis(pose: JOINTPOS; axs: INTEGER; ang: REAL) : JOINTPOS
    -- Return copy of pose with axis axs set to ang

pose__zero_jointpos() : JOINTPOS
    -- Return JOINTPOS with all axes = 0

pose__set_config(conf_str: STRING) : CONFIG
    -- Parse 'F U T, 0, 0, 0' format → CONFIG

pose__replace_config(p: XYZWPR; conf: CONFIG) : XYZWPR
    -- Return pose with config replaced (position/orientation unchanged)
```

#### Joint Operations
```
pose__jpos_add(J1, J2: JOINTPOS) : JOINTPOS    -- element-wise J1 + J2
pose__jpos_sub(J1, J2: JOINTPOS) : JOINTPOS    -- element-wise J1 - J2
pose__add_jpos(J1, J2: JOINTPOS) : JOINTPOS    -- alias for jpos_add
pose__jpos_to_jpos2(pose: JOINTPOS) : JOINTPOS2
pose__jpos_to_jpos3(pose: JOINTPOS) : JOINTPOS3
pose__jpos_to_jpos6(pose: JOINTPOS) : JOINTPOS6
```

#### Extraction & Conversion
```
pose__pose_to_vector(pose: XYZWPR) : VECTOR
    -- Extract position as VEC(pose.x, pose.y, pose.z)

pose__pose_to_orient(pose: XYZWPR) : VECTOR
    -- Extract orientation as VEC(pose.w, pose.p, pose.r)

pose__get_orientation(pose: XYZWPR) : VECTOR
    -- Alias for pose_to_orient

pose__vector_to_pose(v: VECTOR; orient: VECTOR; cnf: CONFIG) : XYZWPR
    -- Combine position vector + orientation vector → XYZWPR

pose__replace_orient(pose: XYZWPR; orient: VECTOR) : XYZWPR
    -- Return new pose with orientation replaced; position unchanged

pose__get_lpos(grp_no: INTEGER) : XYZWPR     -- current robot Cartesian position
pose__get_jpos(grp_no: INTEGER) : JOINTPOS   -- current robot joint position
pose__get_ok() : BOOLEAN                     -- last operation success flag

pose__norm(v: VECTOR) : VECTOR               -- normalize to unit length
pose__dot(v1, v2: VECTOR) : REAL             -- v1 · v2 dot product
```

#### Coordinate System Conversions
```
pose__cylindrical_to_cartesian(origin: XYZWPR; cyl_pose: XYZWPR; z_axis: INTEGER) : XYZWPR
    -- (θ in deg, z, r) → (x, y, z) in the origin frame
    -- z_axis: Z_AXES(3)=vertical, X_AXES(1)/Y_AXES(2) for tilt
    -- orientation is preserved from cyl_pose

pose__cylindrical_to_cartesian_vector(theta: REAL; theta_hat, rad_hat, z_hat: REAL; vecAxis: INTEGER) : VECTOR
    -- Transform a basis vector from cylindrical → Cartesian
    -- theta_hat/rad_hat/z_hat: components along each cylindrical basis axis

pose__cartesian_to_cylindrical(origin: XYZWPR; cart_pose: XYZWPR; z_axis: INTEGER; radius: REAL; use_mm: BOOLEAN) : XYZWPR
    -- (x, y, z) → (θ in deg, z, r)
    -- use_mm=TRUE: return r in mm; FALSE: normalized 0–1

pose__polar_to_cartesian(origin: XYZWPR; pol_pose: XYZWPR; z_axis: INTEGER) : XYZWPR
    -- (θ, ρ, r) → (x, y, z) using spherical-style projection

pose__cylinder_surf_to_origin(pos: XYZWPR; n_pol: VECTOR) : XYZWPR
    -- Move a surface point back along the radial direction to the cylinder origin
```

#### Euler / Frame from Vectors
```
pose__vector_to_euler(vi, vj, vk: REAL; vectorAxis: INTEGER) : VECTOR
    -- Convert 3 components to Euler (W,P,R) angles
    -- vectorAxis: which robot axis this vector represents (X_AXES=1, Y_AXES=2, Z_AXES=3)

pose__vector_to_euler2(v: VECTOR; vectorAxis: INTEGER) : VECTOR
    -- Same but uses quaternion internally → no gimbal lock
    -- PREFERRED over vector_to_euler for near-singularity orientations

pose__vector_to_euler_cylindrical(tangent: VECTOR; theta: REAL) : VECTOR
    -- Convert cylindrical UV tangent vector → WPR Euler angles

pose__create_frame_from_normal(p1, p2: VECTOR; parentfrm: XYZWPR) : XYZWPR
    -- Build coordinate frame from two measured surface points
    -- p1=origin point, p2=point along surface normal axis
    -- Used in 5-axis slicer to align tool perpendicular to print surface
```

#### Frame Correction
```
pose__correctFrame(crrAxisNo: INTEGER; p1, p2: XYZWPR)
    -- Rotates the active user frame so axis crrAxisNo aligns with the line p1→p2
    -- Uses quaternion to find shortest rotation (gimbal-lock free)
    -- Writes result back to user frame register
    -- p1, p2: two measured points on the print surface
```

#### Circle / Arc Geometry
```
pose__find_circle_center(points: ARRAY[*] OF VECTOR) : VECTOR
    -- Circular arc center from N measured points (current impl: 3-point)

pose__find_circumcenter(p1, p2, p3: VECTOR) : T_CIRCLE
    -- Circumcenter + radius from 3 non-collinear points
    -- Uses determinant method (numerically stable)
    -- Returns T_CIRCLE { center: VECTOR; radius: REAL }
```

#### Line & Distance
```
pose__line_increment(p1, p2: VECTOR; fraction: REAL) : VECTOR
    -- p1 + (p2 - p1) * fraction
    -- fraction=0 → p1, fraction=1 → p2

pose__distance(pose1, pose2: XYZWPR) : REAL
    -- Euclidean distance between positions only (ignores orientation)
    -- ||pose2.xyz - pose1.xyz||

pose__matrix_to_cart(reg_no, grp_no: INTEGER)
    -- Convert matrix-format PR[reg_no] to XYZWPR representation in-place
```

---

### posetp — TP-Callable Wrappers

All take AR[] register arguments from TP. Each has a short TP program name in package.json.

#### Register / Group Info
```
posetp__groups_length() : INTEGER           -- pos: pos_grplen
posetp__posreg_type(reg_no, grp_no) : INT  -- pos: pos_prtype  (CC_POSITION etc.)
posetp__num_of_axes(reg_no, grp_no) : INT  -- pos: pos_axescnt
posetp__get_group_xyz(pr_num, grp_no) : XYZWPR
posetp__get_group_joint(pr_num, grp_no) : JOINTPOS
posetp__get_jpos_group(grp_no, axs) : REAL  -- pos: pos_getjpos
```

#### PR Write Operations
```
posetp__clrpr(pr_num)                                            -- pos: pos_clrpr
posetp__set_pr_xyz(x,y,z,w,p,r: REAL; pr_num, grp_no: INTEGER)  -- pos: pos_setxyz
posetp__set_pr_config(conf_str: STRING; pr_num, grp_no: INTEGER) -- pos: pos_setcfg
posetp__set_pr_jpos6(j1..j6: REAL; pr_num, grp_no: INTEGER)     -- pos: pos_setjnt6
posetp__set_pr_jpos3(j1,j2,j3: REAL; pr_num, grp_no: INTEGER)   -- pos: pos_setjnt3
posetp__set_pr_jpos2(j1,j2: REAL; pr_num, grp_no: INTEGER)      -- pos: pos_setjnt2
posetp__set_pr_jpos(j1: REAL; pr_num, grp_no: INTEGER)          -- pos: pos_setjnt
```

#### Arithmetic
```
posetp__matmul(p1, p2: XYZWPR) : XYZWPR     -- pos_mult  — p1:p2 composition
posetp__add(p1, p2: XYZWPR) : XYZWPR        -- pos_add   — element-wise + normalize
posetp__sub(p1, p2: XYZWPR) : XYZWPR        -- pos_sub   — element-wise - normalize
posetp__scalar_mult(p1: XYZWPR; v: REAL)    -- pos_sclmult
posetp__scalar_divide(p1: XYZWPR; v: REAL)  -- pos_scldiv
posetp__scalar_add(p1: XYZWPR; v: REAL)     -- pos_scladd
posetp__scalar_subtract(p1: XYZWPR; v: REAL)-- pos_sclsub
posetp__inv(p1: XYZWPR) : XYZWPR            -- pos_inv  — INV(p1)
posetp__dot(p1, p2: XYZWPR) : REAL          -- pos_dot  — position vectors only
posetp__cross(p1, p2: XYZWPR) : XYZWPR      -- pos_cross
```

#### Geometric / Frame
```
posetp__frame(p1, p2, p3: XYZWPR) : XYZWPR             -- pos_frame  — FRAME(p1,p2,p3)
posetp__frame4(p1, p2, p3, p4: XYZWPR) : XYZWPR        -- pos_frame4
posetp__framevec(v1, v2, v3: VECTOR) : XYZWPR           -- pos_frmvec
posetp__frame4vec(v1, v2, v3, v4: VECTOR) : XYZWPR      -- pos_frmvec4
posetp__find_center(p1, p2, p3: XYZWPR) : XYZWPR        -- pos_center
posetp__create_frame_from_normal(start, end, parent: XYZWPR) : XYZWPR  -- pos_linefrm
posetp__line_increment(start, end: XYZWPR; fraction: REAL) : XYZWPR    -- pos_lineinc
```

---

### matpose — 4×4 Matrix Operations

The 4×4 homogeneous matrix format (row-major):
```
[R11 R12 R13  tx]
[R21 R22 R23  ty]
[R31 R32 R33  tz]
[0   0   0    1 ]
```

```
matpose__rotx(angle: REAL) : t_matarr    -- pure X-axis rotation (angle in degrees)
matpose__roty(angle: REAL) : t_matarr    -- pure Y-axis rotation
matpose__rotz(angle: REAL) : t_matarr    -- pure Z-axis rotation

matpose__transl(x, y, z: REAL) : t_matarr
    -- Pure translation: identity rotation, (x,y,z) in position column

matpose__pose_to_mat(pose: XYZWPR; out_mat: t_matarr)
    -- Convert XYZWPR (Euler ZYX convention) → 4×4 matrix
    -- Writes into caller-provided out_mat (no return value — in/out param)

matpose__mat_to_pose(mat: t_matarr) : XYZWPR
    -- Convert 4×4 matrix → XYZWPR (ZYX Euler extraction)
    -- Handles atan2 safely for pose extraction
```

---

### quaternion — Gimbal-Lock-Free Rotations

```
quaternion__set(w, x, y, z: REAL) : T_QUAT
    -- Construct quaternion directly

quaternion__normalize(q: T_QUAT) : T_QUAT
    -- Scale to unit quaternion: q / ||q||

quaternion__conj(q: T_QUAT) : T_QUAT
    -- Conjugate: (w, -x, -y, -z)

quaternion__mult(q1, q2: T_QUAT) : T_QUAT
    -- Hamilton product q1 * q2

quaternion__pose_to_quat(p: XYZWPR) : T_QUAT
    -- Convert ZYX Euler angles (W,P,R in degrees) → unit quaternion

quaternion__quat_to_pose(q: T_QUAT) : XYZWPR
    -- Convert unit quaternion → WPR Euler angles (degrees)
    -- Position components are 0; caller must add position separately

quaternion__mat_to_quat(mat: t_matarr) : T_QUAT
    -- Extract rotation quaternion from 4×4 matrix (top-left 3×3)

quaternion__quat_to_mat(q: T_QUAT; out_mat: t_matarr)
    -- Fill 4×4 matrix rotation block from quaternion (in/out param)

quaternion__test(expected, actual: T_QUAT) : BOOLEAN
    -- KUnit helper: fuzzy comparison of all 4 components
```

---

## Core Patterns

### 1. Inverse Kinematics for Path Planning

Used in `lib/paths/pathlib` to convert planned Cartesian waypoints to joint motion:

```karel
VAR
    cart_pose : XYZWPR
    joint_pose : JOINTPOS
    grp_no : INTEGER

BEGIN
    grp_no = 1
    cart_pose = pose__get_posreg_xyz(5, grp_no)   -- read target from PR[5]
    joint_pose = pose__solveIK(cart_pose, grp_no) -- IK: Cartesian → Joint
    pose__set_posreg_joint(joint_pose, 6, grp_no) -- write result to PR[6]
END
```

### 2. Cylindrical Coordinate Mapping (5-Axis Slicer)

The slicer operates in (θ, z, r) space and converts to robot (x, y, z, w, p, r):

```karel
VAR
    origin      : XYZWPR    -- cylinder axis frame (from robot calibration)
    cyl_pose    : XYZWPR    -- (theta_deg, z_mm, r_mm, w, p, r) in cylindrical space
    cart_pose   : XYZWPR    -- output in robot world frame

BEGIN
    -- cyl_pose.x = theta (degrees), cyl_pose.y = z height, cyl_pose.z = radius
    cart_pose = pose__cylindrical_to_cartesian(origin, cyl_pose, Z_AXES)
    -- Z_AXES=3 means the cylinder's rotation axis is the Z axis
END
```

### 3. Frame Alignment from Surface Normals (pose__correctFrame)

Used during printer calibration to align tool perpendicular to print surface:

```karel
-- Operator touches tool to two points on the print surface
-- p1 and p2 are measured touch-off positions
-- crrAxisNo = which robot axis to align (e.g., Z_AXES = 3)
pose__correctFrame(Z_AXES, p1, p2)
-- Internally: computes quaternion rotation from current axis to measured normal
-- Writes result back to active user frame
```

### 4. Vector-to-Euler (Gimbal-Lock Safe)

Converting surface normal / path tangent vectors to robot WPR orientations:

```karel
VAR
    surface_normal : VECTOR
    wpr_orient     : VECTOR
    target_pose    : XYZWPR

BEGIN
    surface_normal = VEC(0.0, 0.707, 0.707)

    -- Use vector_to_euler2 (quaternion-based) for numerically stable result
    wpr_orient = pose__vector_to_euler2(surface_normal, Z_AXES)

    -- Combine with position to build full pose
    target_pose = pose__vector_to_pose(pos_vec, wpr_orient, config)
END
```

### 5. 4×4 Matrix Composition

When composing complex multi-frame transforms (e.g., tool offset in rotated frame):

```karel
VAR
    rot   : t_matarr
    trans : t_matarr
    mat   : t_matarr
    pose  : XYZWPR

BEGIN
    rot   = matpose__rotz(45.0)        -- 45-degree Z rotation
    trans = matpose__transl(100, 0, 0) -- 100mm X offset

    -- Compose: first rotate, then translate
    mat = rot.mult(trans)   -- using matrix class mult method
    pose = matpose__mat_to_pose(mat)
END
```

### 6. Circumcenter for Arc Interpolation

Finding arc center from 3 waypoints (used in `posetp` for arc motion planning):

```karel
VAR
    p1, p2, p3 : XYZWPR
    v1, v2, v3 : VECTOR
    circle     : T_CIRCLE

BEGIN
    v1 = pose__pose_to_vector(p1)
    v2 = pose__pose_to_vector(p2)
    v3 = pose__pose_to_vector(p3)
    circle = pose__find_circumcenter(v1, v2, v3)
    -- circle.center : VECTOR  (arc center)
    -- circle.radius : REAL    (arc radius)
END
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Passing `Z_AXES` (3) vs `VERT_AXES` (4) for cylindrical conversion | Positions rotated wrong, theta mapping incorrect | Check coordinate convention: `VERT_AXES` = robot vertical, `Z_AXES` = local frame Z |
| Using `pose__vector_to_euler` instead of `pose__vector_to_euler2` near 90° pitch | Gimbal lock → discontinuous WPR jumps, path jitter | Always use `pose__vector_to_euler2`; it uses quaternion internally |
| Not calling `pose__set_userframe` / `pose__set_toolframe` before IK | IK result is in wrong frame; robot goes to wrong position | Set active frame first; IK uses current active frame |
| Deploying `pose.pc` without removing `shapes.pc` and `draw.pc` | MEMO-128 "parameters are different" compile error | Run `master_del.bat` + `master_test_del.bat` before deploying |
| Reading PR with `pose__get_posreg_xyz` when PR is in joint mode | Status error, returns garbage XYZWPR | Check `pose__get_posreg_rep(reg_no, grp_no)` first; if `CC_JOINT`, use `pose__get_posreg_joint` |
| `matpose__pose_to_mat` result used without checking axis convention | Wrong rotation: FANUC uses ZYX (RPY) Euler convention | Verify: XYZWPR W=yaw (Z), P=pitch (Y), R=roll (X) in FANUC — same as ZYX RPY |
| `quaternion__quat_to_pose` position components | Caller expects xyz too, gets (0,0,0) position | `quat_to_pose` returns only WPR; use `pose__vector_to_pose(pos, orient, cfg)` to add position |

---

## Dependencies

### This module depends on
- `errors` — `CHK_STAT`, `karelError`, named error codes
- `Strings` — `s_to_xyzwpr`, `s_to_joint`, string-to-type parsing
- `math` — `math__norm`, `math__atan2`, trig, `VEC()`
- `shapes` — `shapes__create_plane`, `shapes__project_point_on_plane` (in `correctFrame`)
- `matrix` — 2D array matrix class (via `carr4` template for 4×4)
- `systemlib` — `VEC()`, `ZEROPOS()`, system variables
- `ktransw-macros` — `declare_function`, `funcname`, namespace macros

### Depended on by
- `lib/paths` — pathlib, pathmake, pathmotion, pathlayer all depend on pose
- `lib/draw` — frame ops, cylindrical transforms
- `lib/sensors` — ToF spatial scanning uses IK and cylindrical conversion
- `lib/registers` — TP bridging for PR operations
- `lib/shapes` — `pose__vector_to_euler2` for surface normal → Euler

---

## Build / Integration Notes

1. **Deploy order:** `shapes.pc` and `draw.pc` must be deleted from controller before deploying `pose.pc`. Cross-dependency causes `MEMO-128 parameters are different` errors.

2. **TP Interface:** 68 TP programs are declared in `package.json`. Each has the form `pos_<shortname>`. These TP programs call Karel via AR[] parameters. Arguments are read via `tpe__get_int_arg()`, `tpe__get_real_arg()` etc. Results are written to R[] or PR[].

3. **Matrix type instantiation:** `t_matarr` (4×4) and `t_rotarr` (3×3) are GPP-expanded templates. They depend on the `matrix` `carr4` / `carr3` class. Do not instantiate these manually — they come pre-configured via `matArr.klt` / `rotArr.klt`.

4. **`poseclass` sub-module:** OOP wrapper that stores both `t_xyz` (XYZWPR) and `t_joint` (JOINTPOS) simultaneously. Useful when you need fast switching between representations without redundant IK calls.

5. **Test suite:** `test/test_pose.kl` covers IK/FK round-trip, string constructors, mask operations, cylindrical conversion, and circumcenter. Run via KUnit HTTP interface: `http://robot.ip/KAREL/kunit?filenames=test_pose`.
