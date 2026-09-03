# pose-3d: `fitra_tracker_axis_v1` producer contract

(Started 2026-08-16; extends the unmerged D50 FusionPose producer)

## Background / motivation

The D50 camera calibration needs anatomical world-axis observations at the
same lifecycle and capture time as `fitra_fusion_pose_v1`, but the useful
orientation is produced later than raw triangulation.  Chest, waist and leg
tracker quaternions become stable only after Kalman, IK, occlusion handling,
`TrackerExtractor`, and One-Euro filtering.  Publishing those quaternions
would expose more state than the consumer needs and would allow predicted or
held geometry to masquerade as a current camera observation.

`fitra_tracker_axis_v1` is therefore an additive, axis-only surface.  It
reuses FusionPose lineage and V4L2 capture evidence, combines that evidence
with the existing post-One-Euro `TrackerPose`, and publishes only six unit
axes.  `fitra_pose_gate_v1` remains exact eight joints and
`fitra_fusion_pose_v1` remains exact ten joints.

## Considered alternatives

### Add axes to FusionPose

Rejected.  FusionPose is explicitly a raw pre-Kalman/pre-IK position
contract.  Adding postprocessed orientation to it would mix stages and make
its provenance false.

### Derive all axes from raw joint pairs

Rejected.  This duplicates the already validated tracker reconstruction,
occlusion handling and One-Euro history, and would not match the orientation
used by the live VR output.

### Publish tracker quaternions and let the consumer choose axes

Rejected.  Quaternions expose twist/root state that D50 does not need and do
not prove that the anatomical landmarks were measured in the same capture.

### Carry lineage only in the latest Skeleton3D snapshot

Rejected.  A short lifecycle boundary can be overwritten by a following
Fresh snapshot before the fixed-rate TrackerExtractor reads it.  Lifecycle
records require their own bounded ordered sidecar.

## Adopted design

### Data flow and ownership

```text
triangulate() -> PoseGate lifecycle -> FusionPoseFrame
                                      |
                                      v
                         compact TrackerAxisLineage
                         (capture/lifecycle + 8 booleans)
                                      |
                  +-------------------+-------------------+
                  |                                       |
       Skeleton3DSnapshot latest                 boundary-only FIFO
                  |                                       |
        Kalman -> IK -> floor -> TrackerExtractor <-------+
                  |
             One-Euro TrackerPose
                  |
             TrackerAxisBus
                  |
       GET /api/tracker-axis, WS /ws/tracker-axis
```

`MultiCameraDriver` converts the returned `FusionPoseFrame` into a compact
`pipeline::TrackerAxisLineage`.  It contains no raw coordinates or quality
values: only source sequence, opaque identity/epochs, publish time, capture
interval, source lifecycle, and eight per-joint observed booleans.  Fresh
lineage stays paired with the postprocessed skeleton in
`Skeleton3DSnapshot`.  Boundary lineage also enters
`TrackerAxisLineageBus`, whose bounded FIFO crosses the latest-only snapshot
handoff without losing ordering.

`TrackerExtractor` remains the single owner of smoothing history.  After
One-Euro it drains ordered lineage boundaries, then calls `TrackerAxisBus`
with the current post-filter tracker array and the snapshot's same-capture
lineage.  Repeated fixed-rate reads of one source sample are deduplicated by
`stream_id + source_sample_seq`.

### Axis definitions and provenance gates

The wire order is exact:

1. `chest`
2. `hips`
3. `left_upper_leg`
4. `right_upper_leg`
5. `left_lower_leg`
6. `right_lower_leg`

Roles map to `Chest`, `Waist`, left/right `UpperLeg`, and left/right
`LowerLeg`.  Chest and hips are anatomical body-right:
`-rotate(post_one_euro_quaternion, local +X)`.  Leg axes are
proximal-to-distal: `rotate(quaternion, local +Z)`.  The producer normalizes
the result and rejects non-finite or degenerate quaternions/axes.

An axis is Fresh only when the post-One-Euro tracker is valid and every raw
source landmark below was triangulated in the same FusionPose capture:

| Axis | Required raw landmarks |
|---|---|
| chest | left shoulder + right shoulder |
| hips | left hip + right hip |
| left/right upper leg | same-side hip + knee |
| left/right lower leg | same-side knee + ankle |

IK/FK fill, Kalman prediction, held quaternions, and other postprocess-only
validity cannot satisfy this gate.  Failure produces
`availability:"unavailable"`, `observed_this_frame:false`, and `axis:null`.

### Wire and lifecycle

Every document uses `protocol_version:"fitra_tracker_axis_v1"` and numeric
non-negative sequence, epoch and monotonic-nanosecond fields.  A Fresh
document has `source_state:"fresh"`, the nested capture interval with only
`monotonic_soe` or `monotonic_eof`, and the exact ordered six-axis array.
Capture absence, mixed SOE/EOF, a non-monotonic kernel clock, or invalid
endpoints fails closed as an `unsupported_timestamp` boundary; DQBUF
observation time is never substituted.

A boundary document has `source_state:"boundary"`, one of
`person_lost`, `stream_changed`, `subject_changed`, `coordinate_changed`,
`continuity_reset`, `source_ended`, or `unsupported_timestamp`, and omits
`capture` and `axes`.  Stream, subject, coordinate and source-continuity
changes are compared defensively even when the upstream FusionPose frame is
Fresh, because FusionPose can retain an intermediate boundary in its own
queue while returning the subsequent latest pose.

Both lineage handoff and public delivery use bounded boundary FIFOs.  On
overflow the queue and any pending Fresh document are cut, the effective
`continuity_epoch` advances, and the unknown history collapses to one
`continuity_reset`.  Ordinary frames use one replaceable latest slot.

The tracker-axis WebSocket reuses FusionPose's numeric
`clock_sync_ping`/`clock_sync_pong` parser and serializer.  Message bodies,
axes, identity and capture values are never written to normal logs.

## Invariants

1. Existing PoseGate and FusionPose serializers and endpoint behavior do not
   change.
2. A Fresh axis is derived from the same post-One-Euro tracker snapshot used
   by VR output and from raw landmarks measured in the same source capture.
3. No predicted, held, filled, invalid or degenerate axis is labelled Fresh.
4. Fresh documents contain exactly six ordered axis items; boundary documents
   contain neither capture nor axes.
5. Unknown or mixed timestamp semantics fail closed.
6. Normal frames are latest-only; lifecycle evidence is ordered or replaced
   by an explicit continuity reset.
7. Run and calib-subject construct the same producer path.
8. TrackerExtractor consumes lifecycle boundaries before extraction and
   smoothing, resets quaternion/position/One-Euro/FK history, and does not let
   a pre-boundary latest snapshot reseed that history.
9. The upstream floor-contact latch, anchor, and release correction are reset
   before a non-Fresh boundary frame reaches the floor stage, so the next
   Fresh ankle and lower-leg axis start from the new lifecycle only.

## Milestones

1. Add compact lineage and a boundary-preserving handoff at the FusionPose
   seam.
2. Add post-One-Euro axis generation, provenance gates, schema and delivery
   queues.
3. Wire run/calib-subject composition roots and Crow GET/WS routes.
4. Add pure and real-route tests, documentation, full build, and Jetson
   hardware acceptance.

## Validation

- pure: SOE/EOF acceptance, unavailable/mixed timestamp fail-closed behavior;
- pure: torso sign, leg direction, unit/finite output and invalid quaternion;
- pure: predict/hold/FK-only rejection through per-axis raw landmark gates;
- pure: exact-six ordered schema, nullable unavailable shape, boundary order,
  two queue-overflow continuity resets, and numeric clock pong;
- integration: real Crow GET and WebSocket documents plus clock ping/pong;
- regression: `pose_gate`, `fusion_pose`, `tracker_axis`, `tracker_extract`,
  and `kalman` focused tests followed by all CTest targets and `main --help`;
- hardware: restart the normal three-camera runtime, verify Fresh exact-six
  GET/WS and clock sync, and reconfirm legacy exact-eight/exact-ten routes.

## Remaining work

The 2026-09-03 Jetson merge smoke completed the Release build, focused/full
CTest, boundary GET/WS shape, and numeric clock-sync checks.  Three concurrent
fusion-facing WebSockets did not reduce the fixed-rate TrackerBus tick proxy:
60.052 Hz before versus 60.065 Hz during the connections.  Camera processed
and 3D cadence were also stable.  No pose coordinates, raw camera data, or
opaque identity were saved.

Fresh exact-six hardware acceptance remains required because no person was
present during the smoke; the live endpoint correctly returned a `person_lost`
boundary without `capture` or `axes`.  Live SOE/EOF semantics and the rejection
of a real predict/hold-only role therefore remain unverified.  Camera
disconnect/person-switch destructive tests are not part of this deployment;
the pure lifecycle and overflow tests fix their producer behavior.
