#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Main-loop only. Keep pending text when the UART is busy, and bound each
// drain so a diagnostic cannot wait for an entire line ahead of heartbeats.
class DiagnosticQueue {
public:
  bool push(const char *line) {
    const size_t length = strlen(line);
    if (length > sizeof(buffer_) - used_) {
      ++dropped_;
      return false;
    }
    for (size_t i = 0; i < length; ++i)
      buffer_[(head_ + used_ + i) % sizeof(buffer_)] = uint8_t(line[i]);
    used_ += length;
    return true;
  }
  template <class SerialPort> void drain(SerialPort &port) {
    const int available = port.availableForWrite();
    if (available <= 0 || !used_)
      return;
    size_t length = used_ < 32 ? used_ : 32;
    if (length > size_t(available)) length = size_t(available);
    if (length > sizeof(buffer_) - head_) length = sizeof(buffer_) - head_;
    size_t written = port.write(buffer_ + head_, length);
    if (written > length) written = length;
    head_ = (head_ + written) % sizeof(buffer_);
    used_ -= written;
  }
  uint32_t dropped() const { return dropped_; }
  size_t pending() const { return used_; }

private:
  uint8_t buffer_[768]{};
  size_t head_ = 0, used_ = 0;
  uint32_t dropped_ = 0;
};
