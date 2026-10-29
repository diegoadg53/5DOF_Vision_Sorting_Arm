# Elbow SSC receive repair

The logged `0x2F` mask identifies missing sensor 4 (DOF3 joint encoder). Several
received angle/safety pairs fail CRC but become CRC-valid at a one-bit-earlier
alignment. That is evidence of communication corruption, not permission to
accept reconstructed angles. SPI3 prescaler 32 did not eliminate the failure.

## Transport

Only the elbow bus now uses GPIO-controlled, mode-1, MSB-first SSC on its
existing wires: PC10 clock, PB0 bidirectional DATA, PB4 chip select. Hardware
SPI3 stays disabled during these transactions. Its Cube initialization and
prescaler remain in the project, but no longer set the elbow's clock rate.
SPI1/SPI2 and all servo/UART paths retain their existing transports.

`Core/Inc/TleSscGpio.h` generates exactly 16 full clock periods per word.
DATA is an output only during command/write words; it is released before the
reply. The cycle counter enforces at least 1 us per half-period (maximum
nominal clock 500 kHz; function overhead and interrupts make it slower).
CS setup, turnaround and inactive gaps are each at least that long.
The transaction always finishes with CS inactive, SCK low and DATA released,
including timeout, emergency cancellation and a stopped cycle counter. A new
attempt explicitly restores that idle state before selecting the sensor.
Clocked loops leave interrupts enabled. Only the short pin-mode register
updates save/mask/restore interrupts to avoid races with servo UART callbacks
on other pins of the same GPIO port. Emergency/receive-fault and 2 ms transaction-deadline
checks run at word boundaries. Delayed execution causes rejection; it never
turns a partial response into accepted angle feedback.

Reads and configuration writes use the same transport, with the existing CRC,
parameter readback, safety flags and warm-start validation. The three read
attempts, 50 ms SPI freshness limit, 150 ms wrist freshness limit and latched
feedback fault behavior are retained. No shifted frame is accepted.

## Hardware check after building/flashing Nucleo

The startup banner must contain:

```
[SSC] Elbow uses GPIO clock on PC10/PB0; nominal maximum 500kHz; CRC required
```

After a Nucleo reset, re-enter MODE SELECTOR on Portenta to synchronize the
Nucleo state, then select the desired mode. Confirm feedback while stationary
before movement. Healthy feedback has `validMask=0x3F`, `fault=0`, `hold=0`.

`[SSC] sensor=4 transport=GPIO ...` appears every five seconds when healthy,
and at most once per second when feedback is invalid. It reports cumulative
transaction counts (`transfers`), accepted angles (`good`), wire CRC failures
(`crc`), incomplete/cancelled transfers (`io`), CRC-valid but unsafe responses
(`safety`), and suspected one-bit shifts (`shift`, diagnostic only).
`retries` counts additional angle-read attempts. `goodAge` is time since the
last accepted elbow sample; it is distinct from the wrist age on `[FB]`.
`workUs=last/max` measures GPIO transaction duration; other HAL channels are
not timed and show zero for these fields. `[FB STAT cached]` labels the saved
status-register values, rather than implying a simultaneous fresh snapshot.

Once power and wiring are stable, `good` should keep increasing and the error
counters should stop increasing. Startup with externally powered sensors off
can legitimately increment errors. A return to `0x3F` does not automatically
clear a previously latched fault. Persistent errors still require checking
the elbow's DATA/SCK/CS signals, connections and local sensor supply. Keep the
complete `[FB]`, `[SSC]` and fault lines from such a run.

## Software verification limits

`Tests/run_tests.py` exercises the real firmware with a GPIO-edge sensor model,
including read/write word counts, configuration CRCs, bit-shift rejection,
stuck DATA, abort cleanup, timer failure/wrap and latched stop behavior.
Emulation verifies control flow and edge ordering. It does not measure the
physical waveform, electrical voltage levels or actual interrupt latency.
