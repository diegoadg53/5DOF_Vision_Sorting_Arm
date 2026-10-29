#ifndef BUS_SERVO_FEEDBACK_H
#define BUS_SERVO_FEEDBACK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Register read reply: 55 55 ID LEN CMD DATA... CHECKSUM.
// Broadcast ID discovery is safe here because each UART has exactly one servo.
static inline int Servo_ParseReply(const uint8_t *bytes, size_t count, uint8_t id,
                                  uint8_t command, uint8_t payload_size,
                                  uint8_t *data, uint8_t *source_id) {
    size_t size = payload_size + 6U;
    if (!payload_size || payload_size > 4U || (id == 254U && command != 14U)) return 0;
    for (size_t offset = 0; offset + size <= count; ++offset) {
        const uint8_t *p = bytes + offset;
        if (p[0] != 0x55 || p[1] != 0x55 || p[2] > 253U ||
            (id != 254U && p[2] != id) || p[3] != payload_size+3U || p[4] != command)
            continue;
        uint8_t sum = 0;
        for (size_t i = 2; i < size-1U; ++i) sum += p[i];
        if ((uint8_t)~sum != p[size-1U]) continue;
        if (command == 14U && (payload_size != 1U || p[5] != p[2])) continue;
        memcpy(data, p+5, payload_size);
        *source_id = p[2];
        return 1;
    }
    return 0;
}

#endif
