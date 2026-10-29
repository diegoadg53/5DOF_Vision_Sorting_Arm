#pragma once
#include "../trajectory/Arduino.h"
struct FakeSerial {
  template <class T> void print(const T &) {}
  template <class T> void println(const T &) {}
};
extern FakeSerial Serial;
uint32_t millis();
