#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "JointCalibrationProtocol.h"
#include "JointCalibrationConfig.h"

typedef struct { float offset[5]; } JointOffsets;
typedef struct {
    uint32_t mask, build;
    float raw[3][3], reference[3]; /* sensor means at pitch/roll/elbow captures */
} JointCalibrationEvidence;
typedef struct { float raw[3], q[3]; int valid; } JointCalibrationSample;
typedef struct {
    JointOffsets active, baseline, candidate;
    JointCalibrationEvidence evidence, baseline_evidence;
    uint32_t session, lease_ms, started_ms, last_ms, progress_ms, stable_ms, capture_ms;
    unsigned phase, reason, mask, axis, samples;
    int direction, open, dirty, have_stable, have_previous;
    float correction[3], held[3], start, progress, stable_raw[3], capture_raw[3];
    float previous_q[3], velocity[3], rates[3];
    double sum[3], sum_raw[3];
} JointCalibration;

static inline float JC_Delta(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
static inline float JC_Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline int JC_OffsetsValid(const JointOffsets *o) {
    for (int i = 0; i < 5; ++i)
        if (!isfinite(o->offset[i]) || fabsf(o->offset[i]) > 720.0f) return 0;
    return 1;
}
static inline int JC_Moving(const JointCalibration *c) {
    return c->phase == JC_LEAD_IN || c->phase == JC_SWEEP;
}
static inline void JC_Init(JointCalibration *c, const JointOffsets *o) {
    memset(c, 0, sizeof(*c)); c->active = *o; c->direction = 1;
}
static inline void JC_Fault(JointCalibration *c, unsigned reason) {
    c->phase = JC_FAULT; c->reason = reason; memset(c->rates, 0, sizeof(c->rates));
}
static inline void JC_Cancel(JointCalibration *c) {
    if (c->dirty) c->active = c->baseline;
    c->evidence = c->baseline_evidence;
    c->open = c->dirty = 0; c->phase = JC_IDLE; c->reason = JC_OK;
    memset(c->rates, 0, sizeof(c->rates));
}
static inline int JC_Begin(JointCalibration *c, uint32_t session, uint32_t now,
                           const JointCalibrationSample *s) {
    if (!session || c->open || !s->valid) return 0;
    JointOffsets o = c->active; JointCalibrationEvidence evidence = c->evidence;
    JC_Init(c, &o); c->baseline_evidence = evidence;
    c->baseline = o; c->candidate = o; c->open = 1; c->session = session;
    c->phase = JC_READY; c->lease_ms = c->last_ms = now;
    return 1;
}
static inline int JC_Select(JointCalibration *c, unsigned axis, int direction) {
    if (!c->open || axis > 2 || (direction != -1 && direction != 1) ||
        (c->phase != JC_READY && c->phase != JC_STOPPED && c->phase != JC_LEAD_READY &&
         c->phase != JC_REVIEW)) return 0;
    c->axis = axis; c->direction = direction; c->phase = JC_READY; c->reason = JC_OK;
    return 1;
}
static inline int JC_Start(JointCalibration *c, int sweep, uint32_t now,
                           const JointCalibrationSample *s) {
    if (!c->open || !s->valid || (sweep ? c->phase != JC_LEAD_READY :
        (c->phase != JC_READY && c->phase != JC_STOPPED && c->phase != JC_REVIEW))) return 0;
    c->phase = sweep ? JC_SWEEP : JC_LEAD_IN;
    c->mask &= ~(1U << c->axis);
    c->evidence.mask &= ~(1U << c->axis);
    c->start = c->progress = s->q[c->axis];
    memcpy(c->held, s->q, sizeof(c->held));
    c->started_ms = c->progress_ms = c->last_ms = now;
    c->have_previous = c->have_stable = 0; c->reason = JC_OK;
    return 1;
}
static inline void JC_Stop(JointCalibration *c) {
    if (JC_Moving(c) || c->phase == JC_CAPTURING || c->phase == JC_LEAD_READY) {
        c->phase = JC_STOPPED; c->have_stable = 0; c->reason = JC_OK;
    }
    memset(c->rates, 0, sizeof(c->rates));
}
static inline int JC_Still(const JointCalibration *c, uint32_t now) {
    return c->have_stable && (uint32_t)(now - c->stable_ms) >= JC_STILL_MS;
}
static inline int JC_Capture(JointCalibration *c, uint32_t now) {
    if ((c->phase != JC_STOPPED && c->phase != JC_READY && c->phase != JC_REVIEW) ||
        !JC_Still(c, now)) return 0;
    c->phase = JC_CAPTURING; c->capture_ms = now; c->samples = 0;
    c->mask &= ~(1U << c->axis); c->evidence.mask &= ~(1U << c->axis);
    memcpy(c->capture_raw, c->stable_raw, sizeof(c->capture_raw));
    memset(c->sum, 0, sizeof(c->sum)); c->reason = JC_OK;
    memset(c->sum_raw, 0, sizeof(c->sum_raw));
    return 1;
}
static inline void JC_Tick(JointCalibration *c, uint32_t now,
                           const JointCalibrationSample *s) {
    memset(c->rates, 0, sizeof(c->rates));
    if (!c->open) return;
    if (c->phase != JC_SAVE_ARMED && c->phase != JC_SAVING && c->phase != JC_SAVED &&
        (uint32_t)(now - c->lease_ms) > JC_LEASE_MS) { JC_Fault(c, JC_LEASE_EXPIRED); return; }
    if (!s->valid) {
        c->have_stable = 0;
        if (JC_Moving(c) || c->phase == JC_CAPTURING) JC_Fault(c, JC_BAD_FEEDBACK);
        return;
    }
    float dt = (uint32_t)(now - c->last_ms) * .001f;
    c->last_ms = now;
    if (JC_Moving(c) && (dt > .100f || dt <= 0)) { JC_Fault(c, JC_TIME_LIMIT); return; }
    if (!c->have_stable) {
        memcpy(c->stable_raw, s->raw, sizeof(c->stable_raw));
        c->stable_ms = now; c->have_stable = 1;
    }
    for (int i = 0; i < 3; ++i) {
        if (fabsf(JC_Delta(s->raw[i], c->stable_raw[i])) > JC_STILL_RANGE_DEG) {
            memcpy(c->stable_raw, s->raw, sizeof(c->stable_raw)); c->stable_ms = now;
            break;
        }
    }
    if (JC_Moving(c)) {
        float distance = s->q[c->axis] - c->start;
        int intended = c->phase == JC_LEAD_IN ? -c->direction : c->direction;
        if (intended * distance < -1.0f) { JC_Fault(c, JC_TRAVEL_LIMIT); return; }
        if ((uint32_t)(now - c->started_ms) > JC_MOTION_TIMEOUT_MS) {
            JC_Fault(c, JC_TIME_LIMIT); return;
        }
        if (fabsf(distance) > (c->phase == JC_LEAD_IN ? JC_LEAD_DISTANCE_DEG + 3 : JC_MAX_SWEEP_DEG)) {
            JC_Fault(c, JC_TRAVEL_LIMIT); return;
        }
        if (fabsf(s->q[c->axis] - c->progress) >= .15f) {
            c->progress = s->q[c->axis]; c->progress_ms = now;
        }
        if ((uint32_t)(now - c->progress_ms) > JC_STALL_MS) { JC_Fault(c, JC_STALLED); return; }
        if (c->phase == JC_LEAD_IN && -c->direction * distance >= JC_LEAD_DISTANCE_DEG) {
            c->phase = JC_LEAD_READY; c->have_stable = 0; return;
        }
        for (int i = 0; i < 3; ++i) {
            if (i != (int)c->axis && fabsf(s->q[i] - c->held[i]) > JC_CROSS_AXIS_LIMIT_DEG) {
                JC_Fault(c, JC_CROSS_AXIS); return;
            }
            c->rates[i] = i == (int)c->axis ? c->direction *
                (c->phase == JC_LEAD_IN ? -JC_LEAD_SPEED_DPS : JC_SWEEP_SPEED_DPS) :
                JC_Clamp(JC_HOLD_GAIN * (c->held[i] - s->q[i]), -JC_MAX_HOLD_DPS, JC_MAX_HOLD_DPS);
        }
    }
    if (c->phase == JC_CAPTURING) {
        for (int i = 0; i < 3; ++i)
            if (fabsf(JC_Delta(s->raw[i], c->capture_raw[i])) > JC_STILL_RANGE_DEG) {
                c->phase = JC_STOPPED; c->reason = JC_CAPTURE_MOVED; return;
            }
        for (int i = 0; i < 3; ++i) {
            c->sum[i] += s->q[i];
            c->sum_raw[i] += c->capture_raw[i] + JC_Delta(s->raw[i], c->capture_raw[i]);
        }
        ++c->samples;
        if ((uint32_t)(now - c->capture_ms) >= JC_STILL_MS && c->samples >= 40) {
            const float references[3] = JC_REFERENCE_ANGLES;
            float correction = references[c->axis] - (float)(c->sum[c->axis] / c->samples);
            if (!isfinite(correction) || fabsf(correction) > 90) { JC_Fault(c, JC_BAD_VALUE); return; }
            c->correction[c->axis] = correction; c->mask |= 1U << c->axis;
            for (int i = 0; i < 3; ++i) {
                float mean = (float)(c->sum_raw[i] / c->samples);
                while (mean < 0) mean += 360;
                while (mean >= 360) mean -= 360;
                c->evidence.raw[c->axis][i] = mean;
            }
            c->evidence.mask |= 1U << c->axis;
            memcpy(c->evidence.reference, references, sizeof(references));
            c->candidate = c->baseline;
            c->candidate.offset[0] += c->correction[1] - c->correction[0];
            c->candidate.offset[1] += c->correction[1] + c->correction[0];
            c->candidate.offset[2] += c->correction[2];
            c->phase = c->mask == 7 ? JC_REVIEW : JC_READY;
        }
    }
    if (c->have_previous && dt > 0 && dt <= .100f)
        for (int i = 0; i < 3; ++i)
            c->velocity[i] += .25f * ((s->q[i] - c->previous_q[i]) / dt - c->velocity[i]);
    else memset(c->velocity, 0, sizeof(c->velocity));
    memcpy(c->previous_q, s->q, sizeof(c->previous_q)); c->have_previous = 1;
}
static inline int JC_Apply(JointCalibration *c, uint32_t now) {
    if (c->phase != JC_REVIEW || c->mask != 7 || !JC_Still(c, now) || !JC_OffsetsValid(&c->candidate)) return 0;
    c->active = c->candidate; c->dirty = 1; c->phase = JC_APPLIED;
    c->have_previous = c->have_stable = 0; return 1;
}
