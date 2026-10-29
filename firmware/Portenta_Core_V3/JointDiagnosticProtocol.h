#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Keep identical to Portenta's copy. PC decoder uses little-endian schema 1. */
enum {
    JD_VERSION = 1, JD_SYNC = 0x5AA5, JD_SAMPLE = 1, JD_CONFIG = 2,
    JD_INTERVAL_MS = 20, JD_LEASE_MS = 2000,
    JD_POSITION_ACTIVE = 1, JD_FEEDBACK_FAULT = 2, JD_FEEDBACK_HOLD = 4,
    JD_CALIBRATION_OPEN = 8, JD_END = 16, JD_LEASE_ENDED = 32
};
enum JointDiagnosticStage {
    JD_READY, JD_INITIAL_HOME, JD_BASELINE_HOME, JD_PLAN, JD_APPROACH,
    JD_DESCEND, JD_WAIT, JD_CLOSE, JD_WAYPOINT, JD_DROP, JD_OPEN,
    JD_FINAL_HOME, JD_HOME_DWELL, JD_PAUSED, JD_CANCEL_HOME, JD_COMPLETE, JD_FAULT
};
#pragma pack(push, 1)
typedef struct {
    uint32_t cycle, command, command_ms, control_dt_ms, dropped, calibration_generation;
    float target[3], slew[3], actual[3]; /* pitch, roll, elbow: degrees */
    float raw[5], offsets[3]; /* motor L, joint L, motor R, joint R, elbow */
    int16_t requested[3];
    uint16_t age_ms[6];
    uint32_t submitted[3];
    uint8_t state, valid_mask;
    uint16_t stage, reserved;
} JointDiagnosticSample;
typedef struct {
    uint32_t build, flash_schema, calibration_source, calibration_generation;
    /* Kp[3], Ki[3], Kd[3], friction[3], gravity[KgLR,Kg3,Pweight],
       deadband, slewLR, slew3, offsets[L,R,elbow,q4,q5]. */
    float values[23];
    uint32_t sample_interval_ms;
    uint8_t reserved[16];
} JointDiagnosticConfig;
typedef struct {
    uint16_t sync;
    uint8_t version, type;
    uint16_t size, flags;
    uint32_t sequence, run, timestamp_ms;
    union { JointDiagnosticSample sample; JointDiagnosticConfig config; uint8_t payload[128]; } data;
    uint32_t crc32;
} JointDiagnosticPacket;
#pragma pack(pop)
typedef char JD_AssertSize[(sizeof(JointDiagnosticPacket) == 152 &&
                           sizeof(JointDiagnosticSample) == 128 &&
                           sizeof(JointDiagnosticConfig) == 128) ? 1 : -1];
static inline uint32_t JD_Crc32(const void *memory, size_t size) {
    const uint8_t *p = (const uint8_t *)memory; uint32_t crc = ~0U;
    while (size--) {
        crc ^= *p++;
        for (unsigned bit=0; bit<8; ++bit) crc = (crc>>1) ^ (0xEDB88320U & (0U-(crc&1U)));
    }
    return ~crc;
}
