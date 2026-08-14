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

## Goals and non-goals

Goals:

- preserve the kernel-provided V4L2 monotonic timestamp and SOE/EOF meaning
  from capture through decoded frames and synchronized multi-camera samples;
- expose the eight position-only fusion joints and the quality evidence needed
  by the fixed consumer gates;
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
  "position_space": "fitra_world_z_up_m",
  "joints": {
    "hips": {
      "position_m": [0.0, 0.0, 0.9],
      "availability": "Fresh",
      "keypoint_score": 0.93,
      "inlier_view_count": 3,
      "mean_reproj_error_px": 0.8,
      "max_ray_angle_deg": 12.4
    }
  }
}
```

The eight joint keys are `hips`, `neck`, `left_hip`, `right_hip`,
`left_knee`, `right_knee`, `left_ankle`, and `right_ankle`.  A joint with no
current triangulated observation has `availability:"Unavailable"` and every
position/quality field is `null`; values are never held or predicted.

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
shape, and set all joint values and capture interval fields to `null` /
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
eight joints and capture evidence are unavailable, and the next ordinary
`Fresh` observation is published separately as a pose.  This avoids making a
lifecycle state carry an ambiguous partially usable pose.

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
already-decided PoseGate lifecycle result.  This shares opaque stream/subject
identity without making the new consumer depend on postprocessed `/ws3d`.

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

1. Both wire producers observe `TriangulatedSkeleton` before Kalman, IK and
   floor-contact mutation.
2. `fitra_pose_gate_v1` serialization and tests remain unchanged.
3. A Fresh fusion joint contains position plus all four quality fields; an
   unavailable joint contains none of them.
4. The triangulator's `max_ray_angle_deg` is computed only across the final
   inlier views after reprojection rejection.
5. Capture interval endpoints exist only when every participating camera has a
   valid kernel monotonic timestamp with identical SOE/EOF semantics.
6. Boundary order is never intentionally collapsed.  If capacity prevents
   preservation, `continuity_epoch` changes before further evidence is used.
7. Clock-sync responses and normal logs never contain pose, quality or opaque
   subject identity.

## Milestones

1. Decode and propagate V4L2 timestamp value/semantics.
2. Add maximum inlier-ray crossing angle to triangulation output.
3. Implement `FusionPoseBus`, schema and boundary/latest queue.
4. Add HTTP/WS routes and clock-sync response.
5. Wire the bus at the existing pre-postprocess observation point.
6. Add regression tests and update track/migration records.

## Validation

- unit: V4L2 monotonic SOE/EOF, unknown/copy/invalid values;
- unit: ray angle for known camera geometry and final-inlier selection;
- unit: complete Fresh/unavailable wire shape and exact JSON number/null types,
  old PoseGate serialization, latest-only replacement, stream/subject/
  coordinate invalidation, boundary ordering and overflow continuity;
- unit/integration: clock-sync echo, numeric fields and monotonic receive/send
  ordering through the actual Crow WebSocket route;
- focused: `test_pose_gate`, `test_fusion_pose`, `test_triangulator`;
- full: configure, build, `ctest --test-dir cpp/build --output-on-failure`, and
  `./cpp/build/main --help`;
- hardware: inspect live V4L2 semantics, verify mixed/unknown cameras produce
  `unavailable`, record `/ws/fusion-pose`, and compare capture cadence before
  and after enabling a fusion WebSocket consumer.
