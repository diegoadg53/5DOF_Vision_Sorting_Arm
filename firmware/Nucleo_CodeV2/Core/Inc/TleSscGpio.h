#ifndef TLE_SSC_GPIO_H
#define TLE_SSC_GPIO_H
#include <stdint.h>

// Mode 1, MSB first. Each word has exactly 16 complete clock periods.
// Clocked loops do not mask interrupts: an interrupt can lengthen a phase,
// never add an edge. The board adapter can guard its pin-mode register writes.
typedef struct {
    void (*select)(int active);
    void (*clock)(int high);
    void (*drive)(int high);
    void (*output)(int enabled);
    int (*read)(void);
    int (*delay)(void); // Minimum half-period; false if timing source failed.
    int (*cancelled)(void); // Emergency or transaction deadline.
} TleSscGpioIo;

static inline int TleSscGpioTransfer(const TleSscGpioIo *io, const uint16_t *tx,
                                    unsigned tx_count, uint16_t *rx, unsigned rx_count) {
    if (!io || !tx || !rx || !tx_count || tx_count > 2 || !rx_count || rx_count > 9)
        return 0;
    int ok = 0;
    io->select(0);
    io->clock(0);
    io->output(0);
    if (!io->delay() || io->cancelled()) goto done; // CS inactive time.
    io->drive(0); // Preload the output latch while the DATA pin is still input.
    io->output(1);
    io->select(1);
    if (!io->delay()) goto done; // CS setup time.
    for (unsigned w = 0; w < tx_count; ++w) {
        if (io->cancelled()) goto done;
        for (int bit = 15; bit >= 0; --bit) {
            io->drive((tx[w] >> bit) & 1);
            io->clock(1);
            if (!io->delay()) goto done;
            io->clock(0); // Slave samples command/write data on falling edge.
            if (!io->delay()) goto done;
        }
    }
    io->output(0); // Release DATA before generating any reply clocks.
    if (!io->delay()) goto done; // Command/write-to-read turnaround.
    for (unsigned w = 0; w < rx_count; ++w) {
        if (io->cancelled()) goto done;
        uint16_t value = 0;
        for (unsigned bit = 0; bit < 16; ++bit) {
            io->clock(1); // Sensor changes its output on rising edge.
            if (!io->delay()) goto done;
            io->clock(0);
            value = (uint16_t)((value << 1) | (io->read() != 0));
            if (!io->delay()) goto done;
        }
        rx[w] = value;
    }
    ok = !io->cancelled();
done:
    // Deselect BEFORE normalizing the clock, including an abort at SCK=1.
    io->select(0);
    io->clock(0);
    io->output(0);
    if (!io->delay()) ok = 0; // CS hold/off time before another transaction.
    return ok;
}
#endif
