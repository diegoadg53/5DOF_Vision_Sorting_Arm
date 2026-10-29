# Repeated trajectory and HOME drift measurement

This is a separate fourth option, **JOINT MOTION TEST**, in CALIBRATION MENU.
The original TRAJECTORY TEST and camera AUTO remain available. This test reuses
their validated pickup, classification waypoint, classification pose and HOME
logic. It uses an imaginary part; camera/homography/belt calibration are not
required. Joint limits, fresh feedback and the controller connection are required.

No automatic zero correction, PID tuning or Flash calibration writes occur.
The Nucleo position-control equations are unchanged. HOME is the same direct
five-joint zero target, independent of the pickup height fit and IK solution.

## Run the test

1. Build/upload Portenta V3 and the currently used Nucleo V2 project. Preserve
   the Nucleo calibration journal when programming; mass erase removes it.
2. Close the **Nucleo** serial monitor so Python can open its ST-LINK virtual
   COM port. The Portenta serial monitor can remain open for operator messages.
3. On the PC, install the capture dependency once. Matplotlib is optional:

   ```powershell
   python -m pip install pyserial
   python -m pip install matplotlib
   python -m serial.tools.list_ports
   ```

4. From the Code directory, start capture. Replace `COM7` with the Nucleo port
   from the list; it is an example, not an automatically detected port:

   ```powershell
   python .\tools\joint_motion_report.py record --port COM7 --output .\joint_motion_runs\run01 --plots
   ```

   Use a new output directory for each capture. The program refuses to overwrite
   an existing capture.bin. Without `--plots`, only standard-library analysis and
   pyserial capture are needed. Python reads the Nucleo; it sends no motion commands.

5. Select CALIBRATION MENU -> JOINT MOTION TEST with X, then press A.
   There is no movement until the Nucleo acknowledges the recorder protocol.
6. The arm prepares HOME and holds it for the initial baseline. It then repeats
   the existing trajectory and holds HOME after every completed cycle. The
   pickup X and destination are fixed throughout the run.
7. Observe the level at the baseline and each HOME dwell. Record the run/cycle
   shown on Portenta, numerical pitch/roll if available, and qualitative notes.
   Use the same level placement and viewing/sign convention each time. Y pauses
   between actions; during a HOME dwell it freezes the dwell timer. A resumes.
8. L1 cancels and returns HOME before exiting. B retains the normal emergency
   stop. A pause is not an emergency stop. Python stopping or Ctrl+C does **not**
   stop the robot: motion is controlled by the Portenta and controller.
9. After the final HOME dwell, the recorder sends END and Python automatically
   writes the reports. Portenta stays at HOME. A starts another run; L1 exits.
   Restart Python before starting another run because capture normally ends at END.

Start with an empty gripper and document the payload condition. This test opens
the claw as the normal trajectory does. An empty test does not rule out a fault
that depends on the mass of a real part.

## Editable settings

`JointMotionTestConfig.h` contains defaults of 10 cycles, pickup X=332 mm, bin 3,
a 5000 ms baseline, a 5000 ms HOME dwell, and a 3 degree maximum measured
joint deviation during HOME holds. This last bound stops a large reported drift;
it is not a recalibration or the settling band used by the report. The normal
ArmTrajectory arrival/feedback/timeout checks continue during movements.

Pickup geometry and arrival tolerances remain in `PickMotionConfig.h`;
classification poses/routes remain in `ClassificationConfig.h`. Editing these
changes the test's motion, so use settings whose physical paths have already
been validated. Fixed X/bin isolate repeatability from changes in load path.
The next cycle starts directly from the preceding HOME dwell, as AUTO does;
it does not change the control mode or re-zero the arm between cycles.

## Files produced

| File | Meaning |
|---|---|
| capture.bin | Original byte stream, retained for re-analysis |
| samples.csv | Timestamped targets, slew references, measurements, signed/absolute errors, raw encoders, offsets, motor requests and freshness |
| targets.csv | Metrics per accepted Nucleo P command and coordinate |
| home_drift.csv | Final stationary-window means, spread and change from the initial HOME baseline |
| summary.json | Firmware/controller metadata and data-quality flags |
| run_*_angles.png | Optional requested/slewed/measured pitch, roll and elbow plots |
| run_*_home_drift.png | Optional HOME coordinate drift per completed cycle |

`l` and `r` are the derived left/right bevel coordinates:

```
pitch = (right - left) / 2
roll  = (right + left) / 2
left  = roll - pitch
right = roll + pitch
```

The program reports pitch, roll, elbow and these two derived coordinates. It
does not claim independently measured tracking accuracy for PWM joint 5 or
the gripper. The wrist feedback age is retained to expose supervision failures.
Motor request values are servo speed units, not measured torque/current.
UART submission counters confirm MCU submissions, not actuator acknowledgements.

Raw encoder angles are unwrapped only within continuous, fresh observation
segments. `unwrap_segment_*` changes after invalid samples or excessive gaps;
the program never guesses full turns through a missing interval. Keep the raw
columns when comparing motor motion against joint motion. No motor-to-joint
gear ratio is assumed by the report.

## Metric definitions and limits

The default report uses a 1 degree band, a 500 ms settling hold, a maximum
60 ms observation gap, and a final 1000 ms statistics window. These are analysis
settings; they do not change Nucleo control or Portenta arrival tolerances.

```
error = measured - accepted target
absolute error = abs(error)
MAE = mean(abs(error))
angular RMSE = sqrt(mean(error * error))
```

Metrics are sample-wise in degrees. Full-move RMSE includes the deliberate
movement/slew duration; compare `slew_tracking_rmse_deg` to assess following
the internal slew reference separately. `terminal_*` describes the end of the
observation segment; `steady_*` is supplied only after confirmed settling.
Overshoot direction is inferred from the first observed position toward the
target, so very fast motion before the first sample cannot be reconstructed.

Settling time starts at the Nucleo's acceptance of the P command. It is the
entry into the final contiguous in-band interval, provided at least the hold
duration is observed through the end of that command segment. An out-of-band,
invalid, inactive, paused or missing interval prevents a confident settling
result. Short intermediate moves may legitimately show INSUFFICIENT_HOLD:
the normal trajectory can advance before the offline 500 ms criterion has
been demonstrated. Missing/unsettled results are blank/null, never zero.

HOME means require a contiguous valid stationary window. A missing or inadequate
initial baseline yields no drift-from-baseline value. `normal_completion` is
separate from acquisition quality; inspect both it and `quality_warnings`.
Always retain reports with gaps/faults rather than treating them as successful tests.

Re-analyse without connecting to either MCU:

```powershell
python .\tools\joint_motion_report.py analyse .\joint_motion_runs\run01\capture.bin --output .\joint_motion_runs\run01_reanalysis --tolerance 0.5 --hold-ms 500 --plots
```

## What the experiment can establish

The error is relative to the robot's sensor coordinates, not independent physical
absolute accuracy. At HOME, compare the encoder trend with the shoulder-mounted
level. A consistent sign convention and a stable level mounting are essential.
The shoulder's differential coordinates are not general Euler readings from
the level throughout arbitrary poses; make the external comparison at HOME.

Check whether the level contains magnets. The TLE5012B measures magnetic-field
orientation ([Infineon product description](https://www.infineon.com/part/TLE5012B-E5000)),
so a nearby level magnet is a possible measurement bias, not an established cause
of this robot's tilt. To investigate it, disable position control and keep the
physical joint pose fixed while comparing raw readings with and without the
level nearby. Do not move a suspected interfering magnet with position control
active: the controller could move the arm in response to the changed reading.

| Observation | Next investigation |
|---|---|
| HOME encoder errors grow alongside physical tilt | Holding error, stiction, backlash, load dependence; compare requests and motor/joint encoder changes |
| Encoder HOME repeats but the level moves | Mechanical reference changes outside the measured rotation, sensor/magnet geometry or reference shift, or level mounting/reference |
| Drift depends on approach direction without accumulating | Backlash/hysteresis or asymmetric friction/control behaviour |
| Targets or offsets/generation change unexpectedly | Command/configuration/calibration path |
| Submission counters stop while requests remain nonzero | MCU/UART drive path; submissions alone cannot prove the servo executed them |

For small HOME coordinate changes, left-only change gives approximately equal
and opposite pitch/roll changes; right-only change gives equal same-sign
pitch/roll changes. This identifies the changed reported coordinate, not the
physical cause. An external level is needed to catch movement invisible to
the encoders. Keep a note such as:

```csv
run,cycle,pitch_level_deg,roll_level_deg,note
12345,0,0.0,0.0,initial baseline
12345,1,,,negative pitch visible
```

No automatic level import or calibration correction is performed. Leave numeric
fields empty if the level only gives a qualitative indication.

## Acquisition implementation

Nucleo USART2 remains at 115200 8N1. Fixed 152-byte little-endian packets at
50 Hz use about 7600 bytes/s of its 11520 bytes/s payload capacity, plus one
configuration packet. CRC32, sequence numbers, sample timestamps, per-channel
ages and queue-drop counters identify corruption/loss. The three-slot usable
queue and interrupt transmission never wait for USB progress. A full queue
drops measurements instead of delaying the motor loop. The schema is in the
identical `JointDiagnosticProtocol.h` files; D: commands configure observation
only and cannot command motors or write calibration.

Normal Nucleo text output is suppressed while the binary queue owns USART2.
Use Portenta's monitor for fault messages during recording. Control gains,
offsets, feedback configuration and the existing servo-bus polling decisions
are preserved. Logging has some CPU/interrupt overhead, and suppression removes
the normal blocking USB text transmission; this is not a zero-observer-effect
experiment. Sampled `control_dt_ms` statistics help reveal timing changes but
do not capture every 100 Hz iteration.

The recorder closes after two seconds without a Portenta keepalive. A controller
emergency can flush the queued D:STOP along with other ordinary commands, in
which case END arrives on that recorder timeout. The emergency still stops the
arm through the existing path first; logger expiry itself never commands motion.

## Verification

Run Nucleo `Tests/run_tests.py`, Portenta `tests/trajectory/run_tests.py`, then
`python tools/tests/test_joint_motion_report.py`. These test the real control
adapters/protocol, recorder CRC/ring lifetime, cancellation/handshake/feedback
guards and Python metrics. The Python decoder test consumes a binary packet
produced by the Nucleo C code in emulation. Hardware timing, physical clearance,
level accuracy and actual mechanical drift still require the operator's trial.
