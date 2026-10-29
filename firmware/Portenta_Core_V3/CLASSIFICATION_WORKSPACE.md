# Classification with fixed shoulder angles

The current policy fixes shoulder pitch at 0 degrees and shoulder roll at the
selected bin's radial angle: 50, 70 or 90 degrees. Only q3, q4 and q5 are used to
solve the final TCP pose. The shoulder still moves into its bin angle during the
final movement; this is an IK constraint on the target, not a freeze of the
shoulder throughout the entire trajectory.

`ClassificationConfig.h` sets the reference TCP to:

```cpp
constexpr float referencePose[6] = {-76, 300, 170, 0, 14.545f, 90};
```

This gives a 300 mm horizontal radius and Z=170 mm. The model's exact TCP pitch
for that radius/height is approximately +14.545292 degrees; rounding it to
+14.545 degrees changes the compatible model height by about 0.003 mm, within
the existing 0.5 mm planning tolerance. Both position AND full orientation are checked by IK.
Unreachable targets are rejected before approach; the planner does not silently
relax orientation, alter the requested XYZ or release a shoulder lock.

## Current bin targets

The position and orientation rotate together about the DH shoulder-roll axis.
For the current vertical ring, Euler roll/pitch stay constant and the yaw changes
with the bin angle. The bin spacing and part-ID mapping remain unchanged.

| Bin | Part ID | TCP X (mm) | TCP Y (mm) | TCP Z (mm) | Roll / pitch / yaw (degrees) |
| --- | --- | ---: | ---: | ---: | --- |
| 1 | 2 | 116.836 | 229.813 | 170 | 0 / 14.545 / 50 |
| 2 | 5 | 26.606 | 281.908 | 170 | 0 / 14.545 / 70 |
| 3 | 8 | -76.000 | 300.000 | 170 | 0 / 14.545 / 90 |

The corresponding final joint solution is close to q3=-16.88, q4=-2.33,
q5=0 degrees, with the bin's fixed shoulder angle. Previously q5 was held at
pickup; it is now allowed to change in the final classification movement.
The classification waypoint retains the measured pickup q3/q4 and the pickup
q5 command, so wrist reorientation begins only after that waypoint settles.

## Reachable ranges for this particular orientation

Let r be the horizontal distance from the shoulder-roll axis through {-76,0}:

```text
r = sqrt((X + 76)^2 + Y^2)
```

With reference roll=0, pitch=+14.545, yaw=90 (rotated for the other bins), the
current DH model and local TCP offset {0,0,-49} give:

```text
q5 = 0 degrees
q4 = q3 + 14.545 degrees
r  = 6.227830 + 307*cos(q3)
Z  = 259.142573 + 307*sin(q3)
```

Angles inside sin/cos are converted to radians. The constants include the
115.14 mm wrist translation plus the 49 mm tool extension. With the current
Nucleo/Portenta limits q3=-70..90, q4=-200..30, q5=-95..84 degrees, the
orientation-constrained q3 range is **-70..15.455 degrees**.

Therefore the reference bin (X=-76) has these overall bounds:

- **Y/r: 111.228..313.228 mm.**
- **Z: -29.343..340.952 mm.**

These bounds describe an **arc**, not a rectangle. Y and Z cannot be chosen
independently:

```text
(r - 6.227830)^2 + (Z - 259.142573)^2 = 307^2
```

| Coordinate held | Compatible model coordinate(s) |
| --- | --- |
| Reference Y/r=300 mm | Z approximately 169.997 mm (the upper branch exceeds the q4 limit) |
| Z=170 mm | Reference Y/r approximately 300.001 mm |
| Reference Y/r=323 mm | Unreachable at this orientation and fixed shoulder angles |

For other bins, convert radial distance to their X/Y using the bin angle; the Z
curve is the same. These equations apply to this DH geometry, tool offset and
orientation. Recompute if those change. Workspace bounds are kinematic only:
they are not collision clearance or physical path validation. The selected new
bin points and final q5 movement require physical checking.

## Planner and diagnostics

`AutoConfig::classificationFixedShoulders` selects this policy. The solver uses
`IkTask::FullPose` with fixed-joint mask 0x03, so shoulder pitch/roll remain fixed
inside IK. The same planner serves AUTO, AUTO PICKUP TUNING, TRAJECTORY TEST and
JOINT MOTION TEST. Pickup settings and stored camera/belt calibrations are
unchanged; classification poses/policy remain code settings.

At descent, `[CLASS PLAN]` reports the selected, already validated classification
plan: requested and planned TCP XYZ/RPY, planned joints, position error and full
rotation error. Planning retains 0.5 mm position and 0.035 rad (about 2 degrees)
rotation tolerance and the received joint limits. Feedback arrival continues to
use the existing joint/TCP tolerances and settling timers.

Desktop checks compare samples of the analytic arc against the firmware FK,
verify both box profiles in all bins, preserve the waypoint wrist angles and
reject the original incompatible target. Actual sensor/actuator error is not
included in these model bounds.
