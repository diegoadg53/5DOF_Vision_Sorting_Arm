#ifndef FEEDBACK_PATH_H
#define FEEDBACK_PATH_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// Ages describe successful sensor transactions, not UART telemetry packets.
#define FEEDBACK_SPI_MAX_AGE_MS 50U
#define FEEDBACK_WRIST_MAX_AGE_MS 150U
#define FEEDBACK_STARTUP_SAMPLES 2U
#define FEEDBACK_LIMIT_MARGIN_DEG 2.0f
#define WRIST_FIRST_ID 1U
#define WRIST_DISCOVERY_RETRIES 3U
#define WRIST_RESPONSE_TIMEOUT_MS 8U
#define SENSOR_SPI_TIMEOUT_MS 2U
#define SENSOR_SPI_ATTEMPTS 3U
#define SENSOR_CONFIG_RETRY_MS 100U
#define COMMAND_LINE_SIZE 128U
#define COMMAND_QUEUE_SIZE 32U
#define COMMAND_PROCESS_BUDGET_MS 2U
#define COMMAND_PROCESS_MAX_LINES 8U

enum {
    RX_FAULT_QUEUE_FULL = 1U, RX_FAULT_LINE_LENGTH = 2U, RX_FAULT_NUL = 4U,
    RX_FAULT_UART = 8U, RX_FAULT_RESTART = 16U, RX_FAULT_DMA_POSITION = 32U
};

typedef struct {
    float value;
    uint32_t timestamp;
    uint8_t successes;
} FeedbackSample;

static inline void Feedback_Record(FeedbackSample *s, float value, uint32_t now,
                                   int retain_on_failure) {
    uint32_t max_age = retain_on_failure == 1 ? FEEDBACK_WRIST_MAX_AGE_MS : FEEDBACK_SPI_MAX_AGE_MS;
    if (s->successes && (uint32_t)(now - s->timestamp) > max_age)
        s->successes = 0;
    if (!isfinite(value)) {
        if (!retain_on_failure)
            s->successes = 0;
        return;
    }
    s->value = value;
    s->timestamp = now;
    if (s->successes < FEEDBACK_STARTUP_SAMPLES)
        ++s->successes;
}

static inline int Feedback_Fresh(const FeedbackSample *s, uint32_t now, uint32_t max_age) {
    return s->successes >= FEEDBACK_STARTUP_SAMPLES && isfinite(s->value) &&
           (uint32_t)(now - s->timestamp) <= max_age;
}

// SPI encoders wrap at 360 degrees; all three gear calibration sectors fit
// inside +/-180 degrees relative to HOME. Wrist 4 uses a non-wrapping sector.
static inline float Feedback_CalibrateSpi(float raw, float offset) {
    if (!isfinite(raw))
        return NAN;
    float value = raw + offset;
    while (value > 180.0f)
        value -= 360.0f;
    while (value < -180.0f)
        value += 360.0f;
    return value;
}

static inline float Feedback_CalibrateSpiSector(float raw, float offset, float minimum,
                                                float maximum) {
    float value = Feedback_CalibrateSpi(raw, offset);
    if (value < minimum - FEEDBACK_LIMIT_MARGIN_DEG &&
        value + 360.0f <= maximum + FEEDBACK_LIMIT_MARGIN_DEG)
        value += 360.0f;
    else if (value > maximum + FEEDBACK_LIMIT_MARGIN_DEG &&
             value - 360.0f >= minimum - FEEDBACK_LIMIT_MARGIN_DEG)
        value -= 360.0f;
    return value;
}

static inline uint8_t Feedback_Crc8(const uint8_t *bytes, size_t count) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < count; ++i) {
        crc ^= bytes[i];
        for (int b = 0; b < 8; ++b)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x1D : 0));
    }
    return (uint8_t)~crc;
}

// Infineon SSC CRC covers command + data, MSB first, excluding the safety word.
static inline int Feedback_TleCrcValid(uint16_t command, uint16_t data, uint16_t safety) {
    uint8_t bytes[4] = {(uint8_t)(command >> 8), (uint8_t)command, (uint8_t)(data >> 8),
                        (uint8_t)data};
    return Feedback_Crc8(bytes, sizeof(bytes)) == (uint8_t)safety;
}

static inline int Feedback_TleBlockCrcValid(uint16_t command, const uint16_t *words,
                                          size_t count, uint16_t safety) {
    if (!count || count > 8U) return 0;
    uint8_t bytes[18] = {(uint8_t)(command >> 8), (uint8_t)command};
    for (size_t i = 0; i < count; ++i) {
        bytes[2 + 2*i] = (uint8_t)(words[i] >> 8);
        bytes[3 + 2*i] = (uint8_t)words[i];
    }
    return Feedback_Crc8(bytes, 2 + 2*count) == (uint8_t)safety;
}

// CRC_PAR at 0x0F covers the first 15 bytes of registers 0x08..0x0F.
static inline uint16_t Feedback_TleParameterCrc(const uint16_t registers[8]) {
    uint8_t bytes[16];
    for (size_t i = 0; i < 8; ++i) {
        bytes[2*i] = (uint8_t)(registers[i] >> 8);
        bytes[2*i + 1] = (uint8_t)registers[i];
    }
    return (registers[7] & 0xFF00U) | Feedback_Crc8(bytes, 15);
}

static inline float Feedback_TleAngle(uint16_t command, uint16_t data, uint16_t safety) {
    // Reset, system error, interface error, and invalid-angle bits are active-low.
    if ((safety & 0xF000U) != 0xF000U || !Feedback_TleCrcValid(command, data, safety))
        return NAN;
    return (float)(data & 0x7FFFU) * (360.0f / 32768.0f);
}

typedef struct {
    uint8_t id, locked, misses;
} WristAddress;

static inline void Wrist_RecordAddress(WristAddress *w, int success, int allow_discovery) {
    if (success) {
        w->locked = 1;
        w->misses = 0;
    } else if (!w->locked && allow_discovery) {
        if (++w->misses >= WRIST_DISCOVERY_RETRIES) {
            w->misses = 0;
            w->id = w->id == 253U ? 0U : (uint8_t)(w->id + 1U);
        }
    }
    // A confirmed ID is never changed because of a timeout or corrupted frame.
}

static inline float Wrist_ParsePosition(const uint8_t *bytes, size_t count, uint8_t id) {
    for (size_t i = 0; i + 8 <= count; ++i) {
        const uint8_t *p = bytes + i;
        if (p[0] != 0x55 || p[1] != 0x55 || p[2] != id || p[3] != 5 || p[4] != 28)
            continue;
        uint8_t sum = (uint8_t)(p[2] + p[3] + p[4] + p[5] + p[6]);
        if ((uint8_t)~sum != p[7])
            continue;
        int16_t position = (int16_t)((uint16_t)p[5] | ((uint16_t)p[6] << 8));
        // Current mechanical/software range uses the servo's 0..240 degree sector.
        if (position < 0 || position > 1000)
            continue;
        return (float)position * 0.24f;
    }
    return NAN;
}

typedef struct {
    char lines[COMMAND_QUEUE_SIZE][COMMAND_LINE_SIZE];
    char partial[COMMAND_LINE_SIZE];
    volatile uint8_t head, tail, emergency, overflow;
    uint16_t partial_length;
    uint8_t discard;
} CommandQueue;

// The caller supplies a memory barrier before publishing/consuming queue slots.
#ifndef FEEDBACK_MEMORY_BARRIER
#define FEEDBACK_MEMORY_BARRIER() ((void)0)
#endif

static inline void CommandQueue_Push(CommandQueue *q, const uint8_t *bytes, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        uint8_t c = bytes[i];
        if (c == '\r')
            continue;
        if (c == '\n') {
            if (!q->discard && q->partial_length) {
                q->partial[q->partial_length] = '\0';
                if (strcmp(q->partial, "S:ESTOP") == 0) {
                    q->emergency = 1;
                } else if (!q->emergency && !q->overflow) {
                    uint8_t next = (uint8_t)((q->head + 1U) % COMMAND_QUEUE_SIZE);
                    if (next == q->tail)
                        q->overflow |= RX_FAULT_QUEUE_FULL;
                    else {
                        memcpy(q->lines[q->head], q->partial, q->partial_length + 1U);
                        FEEDBACK_MEMORY_BARRIER();
                        q->head = next;
                    }
                }
            }
            q->partial_length = 0;
            q->discard = 0;
        } else if (!q->discard) {
            if (c == '\0' || q->partial_length >= COMMAND_LINE_SIZE - 1U) {
                q->discard = 1;
                q->overflow |= c == '\0' ? RX_FAULT_NUL : RX_FAULT_LINE_LENGTH;
            } else
                q->partial[q->partial_length++] = (char)c;
        }
    }
}

static inline int CommandQueue_Pop(CommandQueue *q, char *line) {
    if (q->emergency || q->overflow || q->head == q->tail)
        return 0;
    FEEDBACK_MEMORY_BARRIER();
    strcpy(line, q->lines[q->tail]);
    FEEDBACK_MEMORY_BARRIER();
    q->tail = (uint8_t)((q->tail + 1U) % COMMAND_QUEUE_SIZE);
    return 1;
}

#endif
