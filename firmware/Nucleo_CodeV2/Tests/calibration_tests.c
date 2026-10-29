#include "CalibrationStorage.h"
volatile uint32_t test_result[3];
static void check(int ok, uint32_t id) {
    ++test_result[0]; if (!ok) { ++test_result[1]; if (!test_result[2]) test_result[2] = id; }
}
static uint32_t memory[2][CAL_STORE_WORDS * 3];
static int budget = -1, erase_fail;
static unsigned writes, erases;
static uint32_t read_word(void *ctx, unsigned sector, uint32_t word) { (void)ctx; return memory[sector][word]; }
static int program_word(void *ctx, unsigned sector, uint32_t word, uint32_t value) {
    (void)ctx;
    if (!budget) return 0;
    if (budget > 0) --budget;
    if ((memory[sector][word] & value) != value) return 0;
    memory[sector][word] &= value; ++writes; return 1;
}
static int erase_sector(void *ctx, unsigned sector) {
    (void)ctx; if (erase_fail) return 0;
    memset(memory[sector], 255, sizeof(memory[sector])); ++erases; return 1;
}
static const CalibrationStorage store = {0, CAL_STORE_WORDS * 3, read_word, program_word, erase_sector};
static const JointOffsets defaults = {JC_DEFAULT_OFFSETS};
static void reset_store(void) {
    memset(memory, 255, sizeof(memory)); writes = erases = 0; budget = -1; erase_fail = 0;
}
static void storage_tests(void) {
    reset_store(); uint32_t gen;
    CalibrationRecord r = CalStore_Scan(&store, 0);
    check(!r.valid && !r.any_data, 2000);
    check(CalStore_Save(&store, &defaults, &gen) && gen == 1 && writes == CAL_STORE_WORDS, 2001);
    check(CalStore_Save(&store, &defaults, &gen) && gen == 1 && writes == CAL_STORE_WORDS, 2002);
    JointOffsets next = defaults; next.offset[0] -= 1;
    uint32_t original[2][CAL_STORE_WORDS * 3]; memcpy(original, memory, sizeof(memory));
    for (int cut = 0; cut <= (int)CAL_STORE_WORDS; ++cut) {
        memcpy(memory, original, sizeof(memory)); budget = cut;
        int saved = CalStore_Save(&store, &next, &gen);
        r = CalStore_Scan(&store, 0);
        check(r.valid && r.generation == (cut == (int)CAL_STORE_WORDS ? 2U : 1U) && saved == (cut == (int)CAL_STORE_WORDS), 2010 + cut);
        check(!memcmp(&r.offsets, cut == (int)CAL_STORE_WORDS ? &next : &defaults, sizeof(defaults)), 2030 + cut);
    }
    // Rollover must never erase the sector containing the only latest valid record.
    reset_store();
    for (int i = 0; i < 15; ++i) {
        next = defaults; next.offset[2] -= i * .1f;
        check(CalStore_Save(&store, &next, &gen) && gen == (uint32_t)i + 1, 2060 + i);
        r = CalStore_Scan(&store, 0);
        check(r.valid && !memcmp(&r.offsets, &next, sizeof(next)), 2080 + i);
    }
    check(erases > 0, 2100);
    CalibrationRecord previous = CalStore_Scan(&store, r.generation);
    check(previous.valid && previous.generation == r.generation - 1, 2101);
    memory[r.sector][r.index + 3] ^= 1; // Corrupt payload -> older valid record wins.
    CalibrationRecord fallback = CalStore_Scan(&store, 0);
    check(fallback.valid && fallback.generation == previous.generation, 2102);
    reset_store(); memory[0][0] = CAL_STORE_MAGIC;
    r = CalStore_Scan(&store, 0); check(!r.valid && r.any_data, 2103);
    next = defaults; next.offset[0] = NAN;
    check(!CalStore_Save(&store, &next, &gen), 2104);
    reset_store();
    for (int i = 0; i < 6; ++i) { next = defaults; next.offset[0] += i; CalStore_Save(&store, &next, &gen); }
    erase_fail = 1; next.offset[0] += 1;
    check(!CalStore_Save(&store, &next, &gen) && CalStore_Scan(&store, 0).generation == 6, 2105);
    reset_store(); CalStore_Save(&store, &defaults, &gen); memory[0][1] ^= 1;
    check(!CalStore_Scan(&store, 0).valid, 2106);
    // Interrupted writes during a sector rollover must retain generation 6.
    reset_store();
    for (int i = 0; i < 6; ++i) { next = defaults; next.offset[0] += i; CalStore_Save(&store, &next, &gen); }
    memcpy(original, memory, sizeof(memory)); next.offset[0] += 1;
    for (int cut = 0; cut <= (int)CAL_STORE_WORDS; ++cut) {
        memcpy(memory, original, sizeof(memory)); budget = cut;
        CalStore_Save(&store, &next, &gen); r = CalStore_Scan(&store, 0);
        check(r.valid && r.generation == (cut == (int)CAL_STORE_WORDS ? 7U : 6U), 2110 + cut);
    }
    reset_store();
    JointCalibrationEvidence evidence = {0};
    evidence.mask = 1; evidence.build = 0x12345678; evidence.raw[0][0] = 359.95f;
    check(CalStore_SaveWithEvidence(&store, &defaults, &evidence, &gen), 2140);
    r = CalStore_Scan(&store, 0);
    check(r.valid && r.evidence.mask == 1 && r.evidence.build == 0x12345678 &&
          r.evidence.raw[0][0] == 359.95f, 2141);
    evidence.raw[0][0] = NAN;
    check(!CalStore_SaveWithEvidence(&store, &defaults, &evidence, &gen), 2142);
}
static JointCalibrationSample sample = {{201.1f,24.8f,276.2f},{0,0,0},1};
static uint32_t now;
static void tick(JointCalibration *c, unsigned duration) {
    for (unsigned i = 0; i < duration; i += 10) {
        now += 10; c->lease_ms = now; JC_Tick(c, now, &sample);
    }
}
static void engine_tests(void) {
    JointCalibration c; JC_Init(&c, &defaults); now = 1000;
    check(JC_Begin(&c, 42, now, &sample), 2200);
    check(JC_Select(&c, 0, 1) && JC_Start(&c, 0, now, &sample), 2201);
    tick(&c, 10); check(c.phase == JC_LEAD_IN && c.rates[0] == -JC_LEAD_SPEED_DPS, 2202);
    sample.q[0] = -10; tick(&c, 10);
    check(c.phase == JC_LEAD_READY && c.rates[0] == 0, 2203);
    check(JC_Start(&c, 1, now, &sample), 2204);
    tick(&c, 10); check(c.rates[0] == JC_SWEEP_SPEED_DPS, 2205);
    JC_Stop(&c); check(c.phase == JC_STOPPED && c.rates[0] == 0, 2206);
    // Separate pitch/roll/elbow observations all use the original calibration.
    const float measured[3] = {2, -3, 1};
    for (unsigned axis = 0; axis < 3; ++axis) {
        check(JC_Select(&c, axis, 1), 2210 + axis);
        sample.q[0] = measured[0]; sample.q[1] = measured[1]; sample.q[2] = measured[2];
        tick(&c, 900); check(JC_Capture(&c, now), 2220 + axis);
        tick(&c, 900);
        check(c.mask & (1U << axis), 2230 + axis);
        check(!memcmp(&c.active, &defaults, sizeof(defaults)), 2240 + axis);
    }
    check(c.phase == JC_REVIEW && c.mask == 7, 2250);
    check(fabsf(c.candidate.offset[0] - (defaults.offset[0] + 5)) < .001f &&
          fabsf(c.candidate.offset[1] - (defaults.offset[1] + 1)) < .001f &&
          c.candidate.offset[2] == defaults.offset[2] - 1, 2251);
    check(c.candidate.offset[3] == defaults.offset[3] && c.candidate.offset[4] == defaults.offset[4], 2252);
    check(JC_Apply(&c, now) && c.dirty && c.phase == JC_APPLIED, 2253);
    JC_Cancel(&c); check(!c.open && !memcmp(&c.active, &defaults, sizeof(defaults)), 2254);
    sample.q[0] = sample.q[1] = sample.q[2] = 0;
    JC_Begin(&c, 43, now, &sample); JC_Start(&c, 0, now, &sample);
    JC_Tick(&c, now + JC_LEASE_MS + 1, &sample);
    check(c.phase == JC_FAULT && c.reason == JC_LEASE_EXPIRED && c.rates[0] == 0, 2260);
    JC_Cancel(&c); JC_Begin(&c, 44, now, &sample); JC_Start(&c, 0, now, &sample);
    tick(&c, JC_STALL_MS + 100); check(c.phase == JC_FAULT && c.reason == JC_STALLED, 2261);
    JC_Cancel(&c); JC_Begin(&c, 45, now, &sample); JC_Start(&c, 0, now, &sample);
    sample.q[1] = 3; tick(&c, 10); check(c.phase == JC_FAULT && c.reason == JC_CROSS_AXIS, 2262);
    sample.q[1] = 0; JC_Cancel(&c); JC_Begin(&c, 46, now, &sample); JC_Start(&c, 0, now, &sample);
    sample.q[0] = 2; tick(&c, 10); check(c.phase == JC_FAULT && c.reason == JC_TRAVEL_LIMIT, 2263);
    sample.q[0] = 0; JC_Cancel(&c); JC_Begin(&c, 47, now, &sample); JC_Start(&c, 0, now, &sample);
    sample.valid = 0; tick(&c, 10); check(c.phase == JC_FAULT && c.reason == JC_BAD_FEEDBACK, 2264);
    sample.valid = 1; JC_Cancel(&c); JC_Begin(&c, 48, now, &sample);
    tick(&c, 900); JC_Capture(&c, now); sample.raw[0] += .5f; tick(&c, 10);
    check(c.phase == JC_STOPPED && c.reason == JC_CAPTURE_MOVED && !c.mask, 2265);
    sample.raw[0] = 359.95f; tick(&c, 900); sample.raw[0] = .05f; tick(&c, 10);
    check(JC_Still(&c, now), 2266);
    JC_Cancel(&c); now = 0xFFFFFF00U; JC_Begin(&c, 49, now, &sample);
    tick(&c, 900); check(JC_Still(&c, now), 2267);
}
void * _sbrk(int n) { (void)n; return (void *)-1; }
__attribute__((used,noinline)) void test_done(void) { for (;;) __asm__("nop"); }
void test_main(void) { storage_tests(); engine_tests(); test_done(); }
