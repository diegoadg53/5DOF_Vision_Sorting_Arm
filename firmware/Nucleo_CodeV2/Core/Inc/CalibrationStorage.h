#pragma once
#include "JointCalibration.h"

#define CAL_STORE_MAGIC 0x4A43414CUL
#define CAL_STORE_SCHEMA 0x00020001UL /* layout 2 with evidence, angle convention 1 */
#define CAL_STORE_COMMIT 0x434F4D54UL
#define CAL_STORE_WORDS 24U
typedef struct {
    void *context;
    uint32_t words_per_sector;
    uint32_t (*read)(void *, unsigned, uint32_t);
    int (*program)(void *, unsigned, uint32_t, uint32_t);
    int (*erase)(void *, unsigned);
} CalibrationStorage;
typedef struct {
    int valid, any_data;
    unsigned sector;
    uint32_t index, generation, free_index[2];
    JointOffsets offsets;
    JointCalibrationEvidence evidence;
} CalibrationRecord;

static inline uint32_t CalStore_Crc(const uint32_t *words) {
    uint32_t crc = 0xFFFFFFFFU;
    for (unsigned i = 0; i < CAL_STORE_WORDS - 2; ++i)
        for (unsigned byte = 0; byte < 4; ++byte) {
            crc ^= (words[i] >> (8 * byte)) & 255;
            for (unsigned bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    return ~crc;
}
static inline void CalStore_Evidence(const uint32_t *w, JointCalibrationEvidence *e) {
    e->mask = w[8]; e->build = w[21];
    memcpy(e->raw, w + 9, sizeof(e->raw)); memcpy(e->reference, w + 18, sizeof(e->reference));
}
static inline int CalStore_EvidenceValid(const JointCalibrationEvidence *e) {
    if (e->mask > 7) return 0;
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(e->reference[i]) || fabsf(e->reference[i]) > 180) return 0;
        for (int j = 0; j < 3; ++j)
            if (!isfinite(e->raw[i][j]) || e->raw[i][j] < 0 || e->raw[i][j] >= 360) return 0;
    }
    return 1;
}
static inline int CalStore_Valid(const uint32_t *w, JointOffsets *out) {
    if (w[0] != CAL_STORE_MAGIC || w[1] != CAL_STORE_SCHEMA || !w[2] ||
        w[23] != CAL_STORE_COMMIT || w[22] != CalStore_Crc(w) || w[8] > 7) return 0;
    memcpy(out->offset, w + 3, sizeof(out->offset));
    JointCalibrationEvidence e; CalStore_Evidence(w, &e);
    return JC_OffsetsValid(out) && CalStore_EvidenceValid(&e);
}
static inline CalibrationRecord CalStore_Scan(const CalibrationStorage *s, uint32_t exclude) {
    CalibrationRecord r; memset(&r, 0, sizeof(r));
    r.free_index[0] = r.free_index[1] = s->words_per_sector;
    for (unsigned sector = 0; sector < 2; ++sector)
        for (uint32_t index = 0; index + CAL_STORE_WORDS <= s->words_per_sector; index += CAL_STORE_WORDS) {
            uint32_t w[CAL_STORE_WORDS]; int empty = 1;
            for (unsigned i = 0; i < CAL_STORE_WORDS; ++i) {
                w[i] = s->read(s->context, sector, index + i);
                empty &= w[i] == 0xFFFFFFFFU;
            }
            if (empty) {
                if (r.free_index[sector] == s->words_per_sector) r.free_index[sector] = index;
                continue;
            }
            r.any_data = 1;
            JointOffsets o;
            if (CalStore_Valid(w, &o) && w[2] != exclude &&
                (!r.valid || (int32_t)(w[2] - r.generation) > 0)) {
                r.valid = 1; r.offsets = o; r.generation = w[2]; r.sector = sector; r.index = index;
                CalStore_Evidence(w, &r.evidence);
            }
        }
    return r;
}
static inline int CalStore_SaveWithEvidence(const CalibrationStorage *s, const JointOffsets *o,
        const JointCalibrationEvidence *e, uint32_t *generation) {
    if (!JC_OffsetsValid(o) || !CalStore_EvidenceValid(e)) return 0;
    CalibrationRecord prior = CalStore_Scan(s, 0);
    /* Identical data needs no extra write (also makes a repeated Save harmless). */
    if (prior.valid && !memcmp(&prior.offsets, o, sizeof(*o)) && !memcmp(&prior.evidence, e, sizeof(*e))) {
        *generation = prior.generation; return 1;
    }
    unsigned sector = prior.valid ? prior.sector : 0;
    uint32_t index = prior.free_index[sector];
    if (index + CAL_STORE_WORDS > s->words_per_sector) {
        sector ^= 1U; index = prior.free_index[sector];
        if (index + CAL_STORE_WORDS > s->words_per_sector) {
            if (!s->erase(s->context, sector)) return 0;
            index = 0;
        }
    }
    uint32_t w[CAL_STORE_WORDS] = {CAL_STORE_MAGIC, CAL_STORE_SCHEMA, prior.generation + 1};
    if (!w[2]) w[2] = 1;
    memcpy(w + 3, o->offset, sizeof(o->offset)); w[8] = e->mask;
    memcpy(w + 9, e->raw, sizeof(e->raw)); memcpy(w + 18, e->reference, sizeof(e->reference));
    w[21] = e->build; w[22] = CalStore_Crc(w); w[23] = CAL_STORE_COMMIT;
    for (unsigned i = 0; i < CAL_STORE_WORDS - 1; ++i) {
        if (!s->program(s->context, sector, index + i, w[i]) ||
            s->read(s->context, sector, index + i) != w[i]) return 0;
    }
    /* Only a fully written/verified payload receives its completion marker. */
    s->program(s->context, sector, index + 23, w[23]);
    uint32_t verify[CAL_STORE_WORDS]; JointOffsets decoded;
    for (unsigned i = 0; i < CAL_STORE_WORDS; ++i)
        verify[i] = s->read(s->context, sector, index + i);
    if (!CalStore_Valid(verify, &decoded) || memcmp(&decoded, o, sizeof(*o))) return 0;
    *generation = w[2]; return 1;
}
static inline int CalStore_Save(const CalibrationStorage *s, const JointOffsets *o, uint32_t *generation) {
    const JointCalibrationEvidence empty = {0};
    return CalStore_SaveWithEvidence(s, o, &empty, generation);
}
