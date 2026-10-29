#pragma once
#include <stdint.h>

// RAM only. An ACK means command receipt; the completion event means the ESP32
// finished its existing camera-switch procedure. Both are required for reuse.
class CameraAlgorithmSelection {
public:
  bool matches(uint8_t id) const { return known_ && confirmed_ == id; }
  void invalidate() { known_ = waiting_ = acked_ = switched_ = false; }
  void begin(uint8_t id) {
    invalidate();
    requested_ = id;
    waiting_ = true;
  }
  void acknowledge() {
    if (waiting_) { acked_ = true; complete(); }
  }
  void switched() {
    if (waiting_) { switched_ = true; complete(); }
  }
private:
  uint8_t requested_ = 0, confirmed_ = 0;
  bool known_ = false, waiting_ = false, acked_ = false, switched_ = false;
  void complete() {
    if (acked_ && switched_) {
      confirmed_ = requested_;
      known_ = true;
      waiting_ = false;
    }
  }
};
