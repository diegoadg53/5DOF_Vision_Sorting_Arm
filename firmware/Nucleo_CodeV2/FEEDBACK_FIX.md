# Nucleo V2 feedback repair

This is the firmware project currently flashed on the F446RE. Changes are in
`Core/Src/main.c` and the header-only `Core/Inc/FeedbackPath.h`; no CubeIDE source
registration or peripheral configuration changes are required.

## Startup and motion admission

All five SPI encoders are sampled in ESTOP and MODES as well as MANUAL/AUTO.
Cached angles start as `NAN`. Each sensor needs two successful reads before it
is considered ready; a gap past its age limit requires two new valid reads.
The position command checks actual feedback before changing actuator targets
and initializes slew/derivative history from that validated snapshot.

SPI transactions use bounded 2 ms HAL timeouts and verify the SSC CRC and
active-low reset/system/interface/angle-validity bits. A valid-CRC error status
causes a STAT read to clear latched indications, but that angle remains invalid
and subsequent valid reads are still required. CRC covers command and data in
MSB-first order, excluding the safety word, following the
[Infineon reference implementation](https://raw.githubusercontent.com/Infineon/TLE5012-Magnetic-Angle-Sensor/master/src/corelib/TLE5012b.cpp).
The TLE5012B datasheet, pages 31–33, describes the safety
word and status clearing.

Sensor setup is verified individually while stopped, rather than blindly
rewriting registers at MCU boot. This supports sensors that stayed powered
during an MCU restart and sensors whose power returns later. The desired
configuration remains `MOD_1=0xC001`, `MOD_2=0x0804`.

When needed, AUTOCAL is disabled first with the checksum for that intermediate
image installed before the mode write. Prediction/range are then changed with
autocal disabled. Configuration CRC_PAR is recalculated from the first 15 bytes
of registers 0x08..0x0F, preserving coefficient bytes, and verified by readback.
The checksum is repaired even if the mode values already match. This follows
[Infineon's configuration checksum implementation](https://raw.githubusercontent.com/Infineon/TLE5012-Magnetic-Angle-Sensor/master/src/corelib/TLE5012b.cpp)
and [autocalibration sequencing guidance](https://community.infineon.com/t5/%E5%8D%9A%E5%AE%A2/TLE5012B-%E8%AF%A6%E8%A7%A3%E4%B9%8B%E5%9C%A8%E7%BA%BF%E8%87%AA%E6%A0%A1%E5%87%86%E5%8A%9F%E8%83%BD%E5%92%8C%E9%A2%84%E6%B5%8B%E5%8A%9F%E8%83%BD/ba-p/1036065).
Genuine sensor status errors remain invalid; configuration repair does not mask
voltage, magnetic-field or other system failures.

SPI raw angles are calibrated across the 0/360-degree boundary using the sectors
implied by the configured joint limits. For example, right raw 359.99 becomes
-38.01 degrees rather than +321.99. The wrist's 0..240-degree sector is not
wrapped. Position admission allows 2 degrees of measurement margin around the
configured limits; this does not widen command limits. MANUAL_0 still bypasses
joint-position limits but requires valid feedback before enabling motion.

## Wrist polling

Polling starts at ID 1. Before any ID has been confirmed, three failed attempts
advance discovery to the next ID while in ESTOP/MODES. Discovery includes IDs
0..253. Once a matching valid position reply confirms an ID, failures never
change it. A changed physical ID requires a reboot and rediscovery.

The parser checks header, requested ID, reply length, command, checksum and
position range. Reads finish immediately after a complete valid reply or time
out after 8 ms. UART availability/transmission waits are bounded. Receiver
cleanup happens before RX is enabled so a fast reply's first byte is retained.
Position writes go to the confirmed ID and propagate transmission failures.
The HiWonder bus servo communication protocol document, pages
11–13, specifies the position reply structure and ID behavior.

The wrist's last successful value can bridge a brief failed read for up to
150 ms. Failed reads never renew its timestamp. A rejected SPI transport/CRC
transaction is retried at most three times. If all attempts fail, the current
sample is unavailable and is excluded from telemetry/control. The last good
sample is retained only to measure the 50 ms recovery deadline.

During a brief SPI loss, control updates are withheld and the fast motors are
commanded to zero; an existing wrist position command is kept in position mode.
No slew/PID update uses a rejected or cached failed sample. Valid feedback
restores the current motion after derivative history is reset. If the good
sample expires, or a CRC-valid response reports a real sensor error, a fault
is latched. This bounded retry hold is distinct from an emergency latch.

## Faults, telemetry and reset

Position or manual motion is rejected when required feedback is unavailable.
Persistent loss of feedback during motion halts all four bus motors, cancels position and
velocity demands, and latches a local feedback fault. Position control also
checks that measured joints remain within their configured sectors/limits.
Restoring a connection alone cannot restart motion. `S:MODES` clears the local
latch; the normal Portenta menu/emergency reset sends that command. It enables
no motion, and a later command must pass feedback validation again.

The existing `A:pitch,roll,j3,j4,j5,gripper` protocol is preserved. If any required
sensor is invalid/stale, or a local fault is latched, the five arm angles are
`nan`. Current Portenta V3 parsing rejects the entire invalid packet without
renewing feedback freshness. AUTO/TRAJECTORY TEST therefore also latch their
existing stale-feedback emergency. Raw `R:` packets likewise use `nan` for
unavailable sensor readings. Wrist 2 and gripper feedback remain PWM-derived.

USART2/ST-Link serial diagnostics:

- `[FB] Wrist ID confirmed: ...`
- `[FB] validMask=0x3F ... fault=0` means all six measured sensors are fresh.
- Mask bits 0..5 are left motor, left joint, right motor, right joint, elbow,
  wrist. `0x1F` indicates unavailable wrist feedback.
- `[FB SPI] data=... safety=...` records the last five raw SSC responses while
  SPI feedback is unavailable, in that same order excluding wrist.
- `[FB CFG] sensor=... verified; priorSTAT=... clearReadSTAT=...` confirms mode
  and parameter checksum readback. `clearReadSTAT` is the status returned by
  the read that clears latched flags; its value can still include those flags.
- `[FB STAT] ...` reports status register values. Bit `0x0008` is S_FUSE
  (configuration checksum); `0x0004` is S_VR (supply/voltage monitor). Higher
  status bits contain other fields; test the bit masks, not whole-word equality.
- `hold=1` indicates a temporary retry hold. Mask chatter is rate-limited to
  avoid consuming control time in serial printing.
- `[FEEDBACK FAULT] ...` identifies the rejected command or feedback loss.

### Diagnosing a stationary shoulder drive

`[DRIVE L]` and `[DRIVE R]` print once per second during position/manual motion.
`pos` distinguishes position control from MANUAL_0 velocity control; `hold`
indicates a feedback retry hold. `goal`/`slew` are calibrated gear targets and
apply only to position control. `joint` is the calibrated measured gear angle;
`motorRaw` is the raw motor-axle encoder angle. The differential is:
`L=roll-pitch`, `R=roll+pitch`, `pitch=(R-L)/2`, `roll=(R+L)/2`.

`req` is the signed requested motor speed. `tx+` counts HAL-accepted UART
transmissions and `done+` counts UART TX-complete callbacks since the previous
diagnostic line. They do not acknowledge receipt or execution by the motor.
`busy+` counts dropped attempts because the UART/HAL was busy; `fail+` counts
other HAL transmit failures. The first line includes counts since boot.
`g` is the HAL UART transmit state, `status` the last submission result
(0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT). `uartErr` retains the last motor UART error
flags and total callback count as `flags/count`, so a brief error remains visible.
A busy state sampled during a successful transmission is normal; persistent
busy drops with no submitted/completed frames identify a different condition.

For the supplied APPROACH target pitch=20, roll=0, gear targets are L=-20,
R=20. The reported pitch=10.16, roll=9.81 corresponds to L=-0.35, R=19.97.
The left gear is still near HOME while the right reached its target. This is
inconsistent with joint tracking and must be resolved before trajectory tests.

With the diagnostic build, hold the left-drive input briefly in MANUAL_0 and
retain both DRIVE lines. If `req` stays nonzero and `tx+`/`done+` advance with
no HAL failures, inspect the independent motor supply, USART1/PA9 signal path,
level shifter, connector and motor state. If the motor encoder changes while
the joint encoder remains stationary, inspect the mechanical transmission and
sensor attachment. If UART submissions fail or stay busy, retain those fields
for a targeted firmware repair. No gains, motor directions, watchdog thresholds
or kinematics were changed in this diagnostic revision. Set
`ENABLE_DRIVE_DIAGNOSTICS` to 0 after diagnosis; serial printing adds control-loop
latency and is intended for this investigation.

### Servo-side readback without a debugging adapter

While in MODES or ESTOP, `[SERVO L]`, `[SERVO R]` and `[SERVO J3]` read the
three HTS-35H devices through the existing UARTs. No Portenta update or extra
hardware is required. Each bus has one servo: broadcast ID_READ first discovers
its ID, then addressed reads collect voltage, temperature, torqueEnable, mode,
storedSpeed, voltage limits, temperature limit and internalPosRaw. One bounded
query is issued every 100 ms, rotating among the three buses; a full cycle takes
about 2.4 seconds. Packet ID, command, length and checksum must match. A failed
register read is reported and the next register is still attempted.

Reads are disabled in MANUAL/AUTO so they do not change motion-loop timing.
The existing stop commands continue, so storedSpeed=0 and mode=1 are expected
while stopped. `torqueEnable` is the servo's enabled/unloaded flag; it is not a
measured torque/current. Internal position is a signed raw servo reading,
separate from the external TLE5012 motor-axle sensor. These status reads do not
refresh motion feedback, re-enable torque, clear protection or write limits/IDs.
There is no documented current measurement or definitive live fault register
in the supplied protocol; LED_ERROR_READ describes alarm configuration, so it
is deliberately not represented as a current fault code.

Physical tests also showed torqueEnable=0 on healthy J3 and on the recovered
left servo while stopped. That flag alone therefore does not identify this
failure. L and J3 both reporting ID 3 is not an address conflict when they are
on separate UART buses. Do not rewrite IDs or force-enable torque on that basis.

Failed reads include `[SERVO RX ...] tx=... bytes=... flags=... raw=...`:
`tx=0` means the query did not complete transmission; `tx=1 bytes=0` means no
bytes were captured in the bounded window. Nonzero bytes with no accepted
reply indicate a framing/ID/command/checksum mismatch or UART error. `flags`
contains F4 PE=0x01, FE=0x02, NE=0x04, ORE=0x08. The raw hex dump preserves
the captured bytes for diagnosis instead of equating every failed read with
a disabled motor. Polling does not require changing the GPIO's alternate
function; all four motor UARTs use explicit TE/RE turnaround and release their
pins through alternate-function open-drain while receiving. TX retains the
existing push-pull drive for the custom BS170/1 kOhm level-shifter circuits.

The old code left TE enabled and GPIO push-pull during RX. This conflicted with
the line-release requirement in [STM32F446 RM0390, section 25.4.10](https://www.st.com/resource/en/reference_manual/dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf).
TE and RE are now mutually exclusive, and echo cleanup precedes RX enabling
in both blocking writes and TX-complete callbacks. Pin-type updates protect
their shared GPIO register against interrupts. Tests model replies becoming
available only with TE off, RE on and the GPIO released. They establish the
firmware sequence, not physical voltage levels or BS170 behaviour.

A Nucleo-only reset starts the spine in ESTOP. Portenta sends S:MANUAL only
on entry into a manual state, so a Portenta that was already in MANUAL_0 keeps
sending M: commands which the rebooted spine ignores. Returning to MODE_SELECTOR
and entering MANUAL_0 sends S:MODES then S:MANUAL and restores the intended
mode. This explains the initial all-motor refusal after reboot; it does not
explain L staying immobile until the motor supply was cycled. Automatic
re-arming after reboot has not been added.

For a failure that clears only after motor power cycling, keep the arm supported
and keep the 11.5 V motor rail powered. Flash/reset only the Nucleo if needed,
then remain in MODE_SELECTOR or ESTOP and collect at least five seconds of
SERVO output. Repeat the snapshot after recovery for comparison. This preserves
motor power but cannot guarantee that stop/mode commands leave every internal
servo condition unchanged. A voltage reading cannot exclude a brief earlier
supply transient, and normal bulk supply voltage does not establish the
voltage at the individual motor connector under load.

The local HTS-35H manual specifies stall/over-temperature protection and
temperature/voltage/position feedback. The local bus protocol defines reads
14, 23, 25, 26, 27, 28, 30 and 32 and says voltage/temperature protection can
disable torque. Compare actual readings with the stored thresholds rather than
assuming which protection fired. Until that evidence is available, a servo-side
inhibit/protection is a hypothesis and the MCU control sequence can still be
its trigger.

The operator also reports the same failure with a brand-new motor, with only
the motor changed: the left-side BS170 circuit, cable, supply connection and
mechanical load were identical. This makes an isolated damaged motor less
likely, but does not distinguish a shared firmware/bus/power problem from a
repeatable protection event under the same load. The half-duplex correction remains
a diagnostic step; it has not been demonstrated to cure the power-cycle-only
recovery behaviour.

## Command reception

USART6 circular DMA events feed a FIFO with 31 usable complete-line slots. The
USER initialization overrides the CubeMX DMA mode to circular; HT and TC remain
enabled. Event Size is treated as a buffer position: only new bytes are copied,
including wrap, and duplicate IDLE/HT/TC positions are skipped. Reception is not
aborted/rearmed on each IDLE event. Lines split
across DMA idle events are assembled; each line is at most 127 characters.
Incoming data cannot overwrite the command currently being parsed. ESTOP has
priority over queued motion and discards the pending queue at the next loop.
Queue overflow, malformed framing, or UART receive errors halt and latch a
fault. These measures do not provide a hardware emergency-stop guarantee or
an independent master-link watchdog.

Command draining has an eight-line/two-ms budget per loop; a single blocking
command can finish after that elapsed-time budget. Receive faults now print
`[RX FAULT] flags=... uartError=... rxErrors=... queuePeak=...`. Flag bits are:
`0x01` FIFO full, `0x02` line too long, `0x04` embedded NUL, `0x08` UART error,
`0x10` receiver restart failure, `0x20` invalid DMA position. UART faults retain
HAL's error code so electrical/framing errors can be distinguished from backlog.

## Build and validation

Build/flash the existing project in STM32CubeIDE. The current Debug build's
artifact retains the existing name `Debug/NucleoTest_AngularSensor.elf`.
Tests are outside the Core/Drivers source entries and are excluded from firmware.

Run `python Tests/run_tests.py` with `unicorn` and `pyelftools` installed. The
runner also accepts the temporary dependency location used by the Portenta
tests and `--compiler` / `--portenta-compiler` overrides. It compiles the actual
Nucleo command/feedback functions and exercises startup refusal, valid startup,
SPI errors/CRC/status, encoder wrapping, wrist reply validation/ID retention,
stale-feedback halting and explicit reset, split DMA events, overflow and ESTOP
priority. A second executable extracts the actual Portenta A-validation body
and checks that invalid Nucleo packets cannot refresh it. Outputs go to the
system temporary directory; the runner does not contact or flash a board.

The current tests additionally model live-sensor MCU restart with bad CRC_PAR,
late sensor power-up, preservation of coefficient bytes, ordered autocal
disable, brief MANUAL_0/MANUAL_1 SPI failures versus persistent/hardware faults,
circular DMA duplicate/wrap events, receive bursts and UART error diagnostics.

For the first physical verification, keep the arm supported near HOME, flash
this Nucleo build, and check the USART2 diagnostics before starting motion. Wait
for `validMask=0x3F` and `fault=0`. In MANUAL_0, confirm that the wrist reading
changes when its physical angle changes. Then verify MANUAL_0 and MANUAL_1 HOME
before returning to trajectory tests;
retain both Nucleo `[FB]` and Portenta `[ARM]` output if it faults. A simulator
cannot establish electrical reliability, servo response time, or physical path
safety. Gains, slew-rate settings, calibration offsets and capture tolerances
have not been retuned.
