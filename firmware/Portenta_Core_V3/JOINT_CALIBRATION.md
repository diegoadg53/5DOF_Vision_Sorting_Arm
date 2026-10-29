# Guided joint-reference calibration

This feature requires the updated **Portenta V3 M7** and **Nucleo V2** firmware.
ESP32 firmware is unaffected. Install firmware once; later offset changes are
saved by Nucleo without rebuilding or reflashing either board.

The wizard teaches shoulder **pitch**, shoulder **roll**, and elbow. The two
bevel gear offsets are computed together. Wrist q4/q5 offsets are preserved.
Use an independently checked physical reference, such as the same level placed
on the same link faces every time. The firmware cannot identify physical HOME
from the encoders whose offsets it is teaching. It cannot correct deformed
link lengths or pose-dependent magnetic/mechanical errors with a constant offset.

## Controls and sequence

1. Enter MANUAL_0. With joysticks/triggers/buttons neutral, hold **L3+R3 for one
   second**. This enters the dedicated wizard and stops ordinary motor control.
   There is no automatic HOME movement.
2. Left stick Y selects PITCH, ROLL or ELBOW. Left stick X chooses the sweep
   direction (+/-). The serial monitor prints the selection. Select while
   stopped; movement controls have separate meanings inside the wizard.
3. **A** starts a lead-in **relative to the current position**, opposite the
   selected approach direction. Default distance: 10 degrees, speed: 3 deg/s.
   Choose a direction with physical clearance. The old offsets are not used
   to find an absolute HOME target.
4. At READY TO SWEEP, **A** starts a continuous approach in the selected
   direction (default 1 deg/s). The other virtual coordinates are supervised
   and corrected using joint feedback. Nucleo generates both bevel commands
   locally; equal servo speed commands are not assumed to produce pure roll.
5. **Y** stops the movement immediately. Check the level AFTER the arm settles.
   If it overshot, repeat the lead-in and sweep; prefer the same approach
   direction each time. Stopping does not accept a calibration.
6. When physically aligned and stationary, **X** starts capture. The default
   requires 800 ms of stability before capture, then at least 800 ms and 40
   valid samples during capture. The permitted raw-angle deviation from the
   stability anchor is 0.20 degrees, using wrap-aware angular differences.
   An already aligned axis may be captured directly from READY without moving.
7. Repeat for all three virtual coordinates. Keep the previously aligned
   coordinates physically correct and recheck them; recapture an axis if
   needed. All captures use the original offsets. Individual accepted axes
   do not change live calibration.
8. REVIEW prints proposed offsets and their differences from the entry
   calibration. **X** applies the complete candidate temporarily to RAM.
   Position targets and controller history are invalidated; Apply commands
   zero speed rather than moving the arm into a new coordinate target.
9. Recheck the physical references. After settling, **X** acknowledges your
   verification. This acknowledgement is an operator judgement, not an
   automatic measurement by the level.
10. **R1** saves from VERIFIED. The arm must remain stationary without active
    motor hold, as confirmed for this mechanism. Wait for **SAVED and read
    back**, then **L1** returns to MANUAL_0 without an automatic HOME move.

**L1** cancels at other stages and restores the calibration active at entry.
After SAVED it exits while preserving the newly saved calibration. A Flash
commit already underway is allowed to finish before ordinary cancellation.
**B** retains the existing emergency action. Controller loss stops the wizard.
Unsaved RAM changes are rolled back on cancellation/fault; a reboot loads the
last complete saved record. A committed record remains saved even if an
emergency is pressed just as the commit finishes.

The initial reference values are pitch=0, roll=0 and elbow=0 degrees. They can
be changed to another independently measurable configuration in Nucleo's
`Core/Inc/JointCalibrationConfig.h`.

## Adjustment points

Nucleo `Core/Inc/JointCalibrationConfig.h` is the main tuning file:

| Setting | Default | Meaning |
|---|---:|---|
| `JC_DEFAULT_OFFSETS` | -201.1,-24.8,-276.2,-207.1,-95.4 | Fallback L,R,elbow,q4,q5 |
| `JC_LEAD_DISTANCE_DEG` | 10 | Lead-in distance from the current coordinate |
| `JC_LEAD_SPEED_DPS` | 3 | Lead-in joint speed |
| `JC_SWEEP_SPEED_DPS` | 1 | Operator-observed approach joint speed |
| `JC_MAX_SWEEP_DEG` | 25 | Maximum travel from the sweep starting point |
| `JC_MOTION_TIMEOUT_MS` | 45000 | Maximum duration of each movement |
| `JC_LEASE_MS` | 500 | Local movement permission expiry |
| `JC_STALL_MS` | 2500 | Maximum time without 0.15-degree progress |
| `JC_STILL_MS` | 800 | Stationary and capture observation duration |
| `JC_STILL_RANGE_DEG` | 0.20 | Allowed drift from the stability anchor |
| `JC_CROSS_AXIS_LIMIT_DEG` | 2 | Maximum movement of another virtual coordinate |
| `JC_VELOCITY_GAIN` | 18 | Velocity-error correction in servo speed units |
| `JC_MAX_OUTPUT` | 350 | Calibration-only motor command cap |

The calibration velocity controller uses existing `u_fric_val_L`,
`u_fric_val_R`, `u_fric_val_3` from Nucleo `main.c` as feed-forward. These are
motor command units, not degrees/sec. Requested speed is not proof of achieved
speed. Actual behavior under load, breakaway motion and stopping distance
still require low-clearance-risk hardware tests. A slower setting may stall
instead of improving precision. Normal position-control tuning is unchanged.

Current estimated joint limits remain enforced. If incorrect old offsets put
the arm outside those limits, use MANUAL_0 to position it approximately before
entering the wizard. Calibration does not bypass feedback validity checks.

## Persistence and recovery

Nucleo reserves Flash sectors 6 and 7 (`0x08040000..0x0807FFFF`). Both linker
scripts limit firmware Flash to the first 256 KiB. No program sections are
placed in the journal sectors. Firmware is currently well below that limit.

Records have magic, schema/model version, generation, all five offsets,
the wrap-aware mean raw joint readings at each accepted reference, reference
angles, and a firmware build identifier. Software CRC32 covers the payload;
a completion marker is programmed last. Entries append in fixed 96-byte
slots. A partially programmed slot is never reused. Sector
recycling preserves the sector containing the newest valid record until a
replacement is committed. An identical record reuses the current generation.

Startup reports source 100 (blank journal: compiled defaults), 101 (stored
record), or 102 (nonblank journal without a valid record). The existing
compiled offsets remain usable on the first installation with blank storage.
Export source 103 identifies a temporary RAM Apply that is not yet saved.
Corrupt storage is explicitly reported; automatic/position commands are
blocked, while MANUAL_0 and the wizard remain available for recovery. Load
happens before motor control is enabled, including when sensors were powered
before the MCU.

Flash erase/program can stall Nucleo instruction fetching and software
emergency processing. Therefore saving first inhibits motion, waits for
confirmed zero-speed UART transmissions, and checks stationary, fresh
feedback. Portenta uses SAVE/COMMIT acknowledgements and keeps the Nucleo UART
quiet during COMMIT while continuing controller/ESP32 monitoring. SAVING is
not treated as a completed Save acknowledgement. A bounded completion timeout
is separate from ordinary command retries; replayed COMMIT never repeats a
completed write. Fresh sensor samples must be acquired after the operation.
No movement resumes automatically.
The stopped SAVE ARMED phase has a separate 30-second expiry so a lost COMMIT
can be retried without losing the pending calibration. The 500 ms motion lease
still applies to all moving phases. Saved sessions can be closed or replaced
without restoring the earlier calibration.

A debugger/programmer **mass erase** deletes calibration. Firmware updates
must preserve reserved sectors if calibration is to be retained. Keep an
external text backup before changing firmware or programming settings.

## Text backup, import and previous calibration

Use the Portenta serial monitor with newline termination:

```
JC:EXPORT
```

Available in MANUAL_0 or the wizard. The `[JOINT CAL] ACTIVE ... offsets`
line lists L,R,elbow,q4,q5. Export reflects current RAM, including temporary
Apply; the stage tells whether it has been saved. Record the five values.
`Capture PITCH/ROLL/ELBOW` lines additionally show the raw reference readings,
reference angles and build identifier retained with a taught calibration.
Import/restore records are marked as having no new physical captures; older
teaching evidence remains with its historical record until sector recycling.

Inside the wizard, while READY/STOPPED/REVIEW:

```
JC:IMPORT,-201.100,-24.800,-276.200,-207.100,-95.400
JC:RESTORE
```

IMPORT stages those five values for review. RESTORE stages the preceding
valid committed record if one exists. Neither applies or saves automatically:
use X to Apply, verify physically, X to acknowledge, then R1 to Save.
Apply also rejects a candidate that would put the current measured pose
outside the configured joint limits (including wrist/PWM joint coordinates).

After changing joint zeros, recheck the TCP and the empirically fitted pickup
height before AUTO. Camera homography and belt velocity are separate
calibrations; this wizard does not alter their data or your pickup tuning.

## Implementation and tests

UART receive errors stop motion and abort the calibration session, rolling back
unsaved offsets. The Nucleo reports `FAULT` with reason 13 (`JC_LINK_ERROR`),
and Portenta reports `Nucleo Portenta UART receive fault` immediately. An
unexpected `IDLE` is also treated as an ended session, including with older
Nucleo firmware. A normal acknowledged L1 cancellation still returns to MANUAL_0.

Receiver restart happens in the Nucleo main loop after the HAL interrupt has
returned. It discards the interrupted command through the next newline and
leaves the emergency stop latched; recovery never resumes movement. Flash
calibration records are unaffected by a receive fault.

`[RX COUNTS]` separates cumulative noise, framing, overrun, parity and DMA
error callbacks, successful receiver restarts and failed start/abort attempts.
These are counts since Nucleo boot, not necessarily separate physical events.
For STM32F4 HAL, `uartError=0x2` means receive noise; `flags=0x08` means a
UART error, not queue overflow. A `queuePeak` below 32 does not establish a
FIFO overflow, and a healthy `validMask=0x3F` does not establish a sensor fault.
If noise persists, inspect the Portenta D14 TX to Nucleo PC7 RX connection
and its ground return, connectors and routing near motor/servo wiring. A
waveform measurement at PC7 is needed to distinguish signal disturbances
from an MCU-side issue; the error flag alone does not establish the source.

- Nucleo `JointCalibration.h`: portable sweep/capture/correction state machine.
- Nucleo `CalibrationStorage.h`: portable, fault-testable NOR journal.
- Nucleo `JointCalibrationRuntime.h`: HAL/motor/UART adapter, included once by
  `main.c` so CubeIDE's generated source lists need no edits.
- Portenta `JointCalibrationWizard.h`: acknowledged operator workflow.
- Protocol headers in both projects are identical; the tests enforce this.

Run `firmware/Nucleo_CodeV2/Tests/run_tests.py` for real command-adapter checks,
fault-injected journal tests, engine tests and Portenta wizard tests. The test
hardware/Flash is emulated; these checks cannot validate mechanical precision.
