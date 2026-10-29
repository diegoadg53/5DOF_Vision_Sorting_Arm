#pragma once
/* All angles are degrees; rates are measured joint degrees/second, NOT servo speed units. */
#define JC_DEFAULT_OFFSETS {-201.1f, -24.8f, -276.2f, -207.1f, -95.4f}
#define JC_REFERENCE_ANGLES {0.0f, 0.0f, 0.0f} /* shoulder pitch, roll, elbow */
#define JC_LEAD_DISTANCE_DEG 10.0f
#define JC_LEAD_SPEED_DPS 3.0f
#define JC_SWEEP_SPEED_DPS 1.0f
#define JC_MAX_SWEEP_DEG 25.0f
#define JC_MOTION_TIMEOUT_MS 45000U
#define JC_LEASE_MS 500U
#define JC_STALL_MS 2500U
#define JC_STILL_MS 800U
#define JC_STILL_RANGE_DEG 0.20f
#define JC_CROSS_AXIS_LIMIT_DEG 2.0f
#define JC_HOLD_GAIN 2.0f
#define JC_MAX_HOLD_DPS 1.0f
#define JC_VELOCITY_GAIN 18.0f
#define JC_MAX_OUTPUT 350.0f
#define JC_SAVE_TIMEOUT_MS 10000U
#define JC_SAVE_ARM_TIMEOUT_MS 30000U /* no motion; permits a lost COMMIT to be retried */
