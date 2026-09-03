# pose-3d: `fitra_fusion_pose_v1` producer contract

## Background

`fitra_pose_gate_v1` is the compatibility surface used by the first
fitra-fusion prototype.  It correctly taps `tri.skeleton` before Kalman, IK,
floor contact and tracker extraction, but its `content_mono_ns` is the host
observation time immediately after `VIDIOC_DQBUF`.  That is sufficient for its
original freshness boundary, but not for D50's cross-host uncertainty gate:
the consumer needs the V4L2 kernel timestamp, its start/end-of-exposure
semantics, the complete contributing capture interval and per-joint ray
geometry.

This change therefore adds a second contract.  `fitra_pose_gate_v1`,
`/api/pose-gate` and `/ws/pose-gate` remain byte/schema compatible and retain
their existing lifecycle.  The new contract is exposed independently as
`GET /api/fusion-pose` and `WS /ws/fusion-pose`.

The FusionPose record carries two explicitly different position paths for
each joint: raw triangulation evidence from the contributing capture, and an
additive same-capture `post_kalman_ik` position taken after Kalman/IK but before
floor-contact or root-output corrections.  The latter may exist for a
predict-only or IK-filled joint, but it never creates raw observation evidence
or a `Fresh` state.

## Goals and non-goals

Goals:

- preserve the kernel-provided V4L2 monotonic timestamp and SOE/EOF meaning
  from capture through decoded frames and synchronized multi-camera samples;
- expose the ten position-only fusion joints and the quality evidence needed
  by the fixed consumer gates;
- expose same-capture post-Kalman/IK positions without overwriting raw
  position or quality lineage;
- make ordinary pose delivery latest-only while preserving ordered lifecycle
  boundaries;
- make boundary loss explicit through a monotonic `continuity_epoch`;
- support monotonic clock mapping without logging pose, identity or quality.

Non-goals:

- changing camera synchronization policy or widening `sync_window_ms`;
- changing Camera root, Kalman, IK, floor-contact, tracker quaternion or VMT;
- changing `fitra_pose_gate_v1` fields or lifecycle semantics;
- accepting DQBUF return time as a substitute for a missing/unknown kernel
  timestamp.

## Wire contract

All positions are metres in `fitra_world_z_up_m`; errors are pixels and ray
angles are degrees.  `sample_seq`, epochs and every monotonic nanosecond field
are non-negative JSON integer numbers.  The Jetson monotonic values are within
the signed 64-bit range and the fitra-fusion adapter contract intentionally
does not accept decimal strings.

```json
{
  "protocol_version": "fitra_fusion_pose_v1",
  "sample_seq": 42,
  "event_type": "pose",
  "stream_id": "stream-opaque",
  "subject_track_id": "subject-opaque",
  "coordinate_epoch": 123,
  "continuity_epoch": 1,
  "source_state": "Fresh",
  "source_reason": "Fresh",
  "source_publish_mono_ns": 1234567890123,
  "capture": {
    "oldest_mono_ns": 1234567879000,
    "newest_mono_ns": 1234567885000,
    "span_ms": 0.006,
    "timestamp_semantics": "monotonic_soe"
  },
  "filtered_position_provenance": {
    "stage": "post_kalman_ik",
    "position_source": "skeleton3d_snapshot",
    "floor_contact": false,
    "root_transform": false
  },
  "position_space": "fitra_world_z_up_m",
  "joints": {
    "hips": {
      "position_m": [0.0, 0.0, 0.9],
      "filtered_position_m": [0.01, 0.0, 0.91],
      "availability": "Fresh",
      "observed_this_frame": true,
      "keypoint_score": 0.93,
      "inlier_view_count": 3,
      "mean_reproj_error_px": 0.8,
      "max_ray_angle_deg": 12.4
    }
  }
}
```

The ten joint keys are `hips`, `neck`, `left_hip`, `right_hip`, `left_knee`,
`right_knee`, `left_ankle`, `right_ankle`, `left_shoulder`, and
`right_shoulder`.  The shoulder entries map directly to the corresponding
HALPE26 joints and carry the same score, final-inlier view count, mean
reprojection error, and maximum acute ray angle as the original eight joints.
`position_m` and all four quality fields are always the raw triangulation
tuple.  `filtered_position_m` is the same HALPE26 joint after the producer's
Kalman/IK stage; it is `null` when that postprocessed skeleton has no finite
valid joint.  `observed_this_frame` is true only when the raw joint is valid,
has score at least 0.3, has an inlier view, and has finite non-negative
reprojection/ray-angle evidence.  A predict-only or IK-filled filtered value
therefore remains `observed_this_frame:false`, `availability:"Unavailable"`,
and has null raw position/quality fields.

HALPE26 has no independent camera observation for the formal `hips` role.  The
producer derives it only from the same-capture raw left/right hip pair.  Its
raw position is their midpoint; score, inlier views and ray angle use the
weaker side, while mean reprojection error uses the worse side.  If either raw
hip fails the observation conditions, Hips remains unavailable even when a
filtered midpoint can be formed from postprocessed positions.  A missing
non-Hips joint is local to that joint and does not invalidate the other nine.

The D50 hardware review added the shoulder pair without changing the protocol
identifier because this producer contract has not merged.  The consumer uses
the left/right hip lateral axis as the waist world-yaw observation and the
left/right shoulder lateral axis as the chest world-yaw observation.  This
producer exports raw position evidence plus the explicitly labelled
post-Kalman/IK position sidecar; BoneLocal axis comparison, post-wear I-pose
shoulder-width normalization, and tracker orientation remain consumer
responsibilities.  `fitra_pose_gate_v1` deliberately stays at its original
exact eight keys.

`capture.timestamp_semantics` is one of:

- `monotonic_soe`: V4L2 reports `TIMESTAMP_MONOTONIC | TSTAMP_SRC_SOE`;
- `monotonic_eof`: V4L2 reports `TIMESTAMP_MONOTONIC | TSTAMP_SRC_EOF`;
- `unavailable`: any contributing timestamp is absent/invalid, uses another
  clock, or the cameras disagree on SOE versus EOF.

For `unavailable`, both capture endpoints and `capture.span_ms` are `null`.
The pose may still be Fresh, but the consumer's capture-time gate must reject
it.  DQBUF observation time is retained only for the legacy v1 contract and
internal latency diagnostics; it is never copied into these v1 fusion capture
fields.

Boundary events use `event_type:"boundary"`, keep the same complete top-level
shape, and set all joint position/quality fields to `null`,
`observed_this_frame` to false, and capture interval fields to `null` /
`Unavailable`.  `source_state` is exactly one of `Fresh`, `Reacquired`,
`PersonSwitched`, `Unavailable`, `EpochChanged`, `UnsupportedTopology`,
`UnsupportedMultiPerson`, or `ContinuityReset`.  PoseGate loss reasons such as
`no_triangulated_gate_joint`, `sync_miss`, and `idle` map to an `Unavailable`
boundary; `PersonLost` is not a wire state.  `ContinuityReset` is also a
PersonLost-equivalent invalidation boundary for the adapter.  A consumer must
invalidate accumulated evidence whenever `event_type` is `boundary`, any of
`stream_id`, `subject_track_id`, or `coordinate_epoch` changes, or
`continuity_epoch` changes.

`Reacquired` and `PersonSwitched` are boundary-only on this producer: their
ten joints and capture evidence are unavailable, and the next ordinary
`Fresh` observation is published separately as a pose.  This avoids making a
lifecycle state carry an ambiguous partially usable pose.

Before a non-`Fresh` lifecycle frame reaches the post-Kalman/IK seam, the
producer resets the Kalman joint and direction history.  The boundary frame's
measurement therefore seeds the new lifecycle, and the following `Fresh`
`filtered_position_m` cannot interpolate with the previous subject or
coordinate epoch.

The WebSocket accepts:

```json
{"type":"clock_sync_ping","nonce":7,"client_send_mono_ns":123456789}
```

and responds only to that connection with:

```json
{"type":"clock_sync_pong","nonce":7,"client_send_mono_ns":123456789,
 "server_receive_mono_ns":223456789,"server_send_mono_ns":223456999}
```

Both server values come directly from `CLOCK_MONOTONIC`.  Malformed or unknown
messages are ignored without logging their body.

## Decisions and rejected alternatives

### Additive bus instead of extending PoseGate v1

Adopted: a separate `FusionPoseBus` consumes the same raw triangulation and the
already-decided PoseGate lifecycle result, then receives an explicitly bounded
post-Kalman/IK skeleton sidecar.  This shares opaque stream/subject identity
without making the new consumer depend on post-floor `/ws3d` output or
overwriting raw quality lineage.

Rejected: append the fields to `fitra_pose_gate_v1`.  Even additive fields can
break strict consumers and would change the meaning of the existing content
timestamp.

### Kernel timestamp as evidence, DQBUF time as legacy observation

Adopted: store both.  `v4l2_buffer.timestamp` is decoded only when its clock is
monotonic and its source is SOE/EOF.  The existing DQBUF timestamp remains in
the existing member and old wire.

Rejected: replace the legacy timestamp or synthesize a kernel timestamp from
DQBUF return.  The first violates v1 compatibility; the second hides capture
uncertainty and can pass the consumer's 10 ms gate incorrectly.

### Latest pose slot plus bounded boundary queue

Adopted: Fresh pose frames replace one latest slot.  Lifecycle frames enter a
FIFO boundary queue.  Publication chooses the oldest sequence between the FIFO
head and the current latest pose, preserving causal order while collapsing only
ordinary poses.

On FIFO overflow, the queue and pending pose slot are cut, `continuity_epoch`
is incremented, and one `ContinuityReset` boundary with
`source_reason:"boundary_queue_overflow"` is inserted before the incoming
boundary.  The consumer therefore receives an explicit invalidation even
though the exact lost boundary sequence is unknowable.

The normal stream-ID change is process/runtime reconstruction: `make_threed`
constructs both buses from the new PoseGate stream ID and calibration epoch.
For defensive in-process lifecycle-owner replacement, `FusionPoseBus::observe`
also detects a different lifecycle stream ID, cuts the pending pose/boundaries,
increments continuity, and emits `ContinuityReset(stream_id_changed)` before
the first new-stream pose.

Rejected: an unbounded event queue (capture stalls can exhaust memory) and a
single latest slot for all events (can erase loss/epoch evidence).

## Invariants

1. PoseGate observes `TriangulatedSkeleton` before Kalman/IK.  FusionPose
   retains that raw `TriangulatedSkeleton` and capture interval, while its
   additive filtered position is copied after Kalman/IK and before
   floor-contact/root-output mutation.
2. `fitra_pose_gate_v1` serialization and tests remain unchanged.
3. Every document contains exactly ten fusion joints.  A raw-observed joint
   contains `position_m`, all four quality fields and
   `observed_this_frame:true`; a joint with only predicted/filled filtered
   position remains unavailable with null raw position/quality.
4. Hips is a same-capture left/right raw midpoint with conservative quality
   aggregation; one missing hip cannot make Hips Fresh.  Other joint loss is
   local.
5. The triangulator's `max_ray_angle_deg` is computed only across the final
   inlier views after reprojection rejection.
6. Capture interval endpoints exist only when every participating camera has a
   valid kernel monotonic timestamp with identical SOE/EOF semantics.
7. Boundary order is never intentionally collapsed.  If capacity prevents
   preservation, `continuity_epoch` changes before further evidence is used.
8. Clock-sync responses and normal logs never contain pose, quality or opaque
   subject identity.

## Milestones

1. Decode and propagate V4L2 timestamp value/semantics.
2. Add maximum inlier-ray crossing angle to triangulation output.
3. Implement `FusionPoseBus`, schema and boundary/latest queue.
4. Add HTTP/WS routes and clock-sync response.
5. Wire raw capture/quality and post-Kalman/IK positions into one additive
   FusionPose record before floor-contact/root-output mutation.
6. Add regression tests and update track/migration records.

## Validation

- unit: V4L2 monotonic SOE/EOF, unknown/copy/invalid values;
- unit: ray angle for known camera geometry and final-inlier selection;
- unit: complete exact-ten Fresh/unavailable wire shape and exact JSON
  number/null types, direct HALPE26 shoulder mapping,
  old PoseGate serialization, latest-only replacement, stream/subject/
  coordinate invalidation, boundary ordering and overflow continuity;
- unit/integration: exact-ten GET/WS documents plus clock-sync echo, numeric
  fields and monotonic receive/send ordering through the actual Crow routes;
- unit: deliberately different raw/filtered positions, predict-only and
  low-quality raw joints, conservative Hips midpoint quality, one-hip loss,
  and local single-joint loss;
- focused: `test_pose_gate`, `test_fusion_pose`, `test_triangulator`;
- full: configure, build, `ctest --test-dir cpp/build --output-on-failure`, and
  `./cpp/build/main --help`;
- hardware: inspect live V4L2 semantics, verify mixed/unknown cameras produce
  `unavailable`, record `/ws/fusion-pose`, and compare capture cadence before
  and after enabling a fusion WebSocket consumer.

## Jetson merge validation (2026-09-03)

- Release configure/build completed on Jetson Orin Nano Super in MAXN_SUPER.
  The focused eight tests and the full 37-test CTest suite passed; the Crow
  test required the host context because the sandbox denies loopback sockets.
- The live GET endpoint returned the exact ten joint keys and the live WS
  delivered exact-ten boundary documents.  Numeric clock-sync pong and
  receive-before-send ordering passed.  No pose coordinates, raw camera data,
  or opaque identity were written to the smoke output.
- With PoseGate, FusionPose, and TrackerAxis WebSockets connected together,
  the three cameras remained at 59.3--59.9 processed frames/s.  3D processed
  cadence was 58.803 Hz before and 58.816 Hz during the connections; the
  Skeleton3D publish cadence was 30.088 Hz and 30.095 Hz respectively.
- No person was present during this window.  The live document was therefore
  an `Unavailable` boundary with unavailable capture evidence.  Fresh
  exact-ten data, non-null `filtered_position_m`, and live SOE/EOF timestamp
  semantics remain unverified; their unit/Crow coverage is not hardware
  acceptance.
