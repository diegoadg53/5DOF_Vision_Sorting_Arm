# Custom-model ID diagnostic

AUTO logs showed three different tracks all assigned `part ID=8 bin=3`.
The ESP32 currently derives the transmitted custom-model ID using `atoi` on
the text after the last underscore in the camera class name. It does not use
the camera's numeric ID for the transmitted custom-model ID.

`ENABLE_CLASSIFICATION_DIAGNOSTIC` near the top of `ESP32_V3.ino` enables a
sampled diagnostic while a custom-model stream is running. It changes no
classification or April-tag conversion policy. Example:

```
[CLASS TX] model=131 box=1/1 cameraID=2 name="Pieza_08" sentID=8
```

Here `cameraID` is the numeric ID captured from the camera result, `name` is
the cached camera class name (the existing cache retains up to 23 characters),
and `sentID` is the byte placed in the UDP detection packet. The example
demonstrates a disagreement; it is not a measurement from the user's camera.

At most one bounding box is sampled per 500 ms. With multiple boxes, the
sample rotates among array positions; those positions are not tracking IDs.
The diagnostic is formatted from the same detection data as the UDP packet.
It queues text after releasing the camera-data mutex. `DiagnosticQueue.h`
retains pending bytes when the serial buffer is busy and drains at most 32
bytes per main-loop iteration, capped by available transmit space. A full
768-byte queue drops a new message and increments `logDrops`; it never waits
for an entire line to fit. Controller packet timing and vision packet layout
are unchanged.

For a run without arm motion, choose AUTO's ONLY VISION option and press A to
start streaming. Present each physical part separately and record several
`[CLASS TX]` lines from the ESP32 monitor. Use Y to stop streaming.

Selecting ONLY VISION with X enters AUTO in the stopped state; A is still
required. Portenta now prints `START_STREAM requested`, then
`START_STREAM acknowledged`. While running ONLY VISION, it prints a sampled
`[VISION]` report every 500 ms containing received IDs, a rotating box's pixel
and corrected world coordinates, and the number of tracker candidates.
`updates` counts processed detection snapshots, not every UDP datagram. A
tracker candidate is not necessarily eligible for pickup (ID mapping,
workspace and deadline checks still apply when MOVE ARM is enabled).
The stage label now says `VISION STARTING`, `VISION TRACKING` or
`VISION STOPPED` rather than the arm's `IDLE` stage. Stop also reports
`visionUpdates`; sorted/missed/rejected totals remain zero in a vision-only
run because no part is assigned to the arm.

ESP32 also reports `[VISION ESP]` status every two seconds while connected:

- `stream=0`: camera polling is disabled. Check the A press and START exchange.
- `stream=1`, increasing `polls` and `empty`, without increasing `objectFrames`:
  camera calls complete but report no objects for `model`.
- `polls` stops increasing and `pollAge` grows: camera polling is not completing;
  inspect camera-task and algorithm-switch messages.
- `objectFrames` and `dataTx` increase: the ESP32 is obtaining objects and its
  local UDP send calls succeed. This does not prove delivery to Portenta.
- `txFailures` increases: a local detection UDP send failed.
- Portenta reports no DATA_OBJECTS while ESP32 `dataTx` increases: investigate
  the network/receive path. Portenta deliberately stops displaying old boxes
  as current once the last detection packet is more than one second old.

Poll/send counters are cumulative since ESP32 boot. The existing camera code
does not send empty custom-model frames, so a quiet belt can legitimately
produce Portenta's `No DATA_OBJECTS` message. ESP32's `empty` counter
distinguishes that case from a stream that never started. These diagnostics
do not change the camera algorithm, suffix conversion, or tracker rules.

- If distinct numeric IDs all have a name ending in `_08`, the current suffix
  conversion explains the identical transmitted ID.
- If camera ID and name both identify the same class for different physical
  parts, inspect the selected model/labels and camera recognition results.
- If transmitted IDs vary between 2, 5 and 8 but Portenta assigns ID 8 to
  every pickup, inspect the tracker label history and reservation point next.

The expected active Portenta mapping is 2 -> bin 1, 5 -> bin 2, 8 -> bin 3.
Do not switch to numeric camera IDs until the logged values establish that
they identify the desired classes in this model. Set the diagnostic macro
to 0 after completing the investigation, then rebuild/upload ESP32.
