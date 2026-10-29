#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846

// Capture text diagnostics; other print overloads remain irrelevant to motion tests.
class Print {
public:
  char text[2048]{};
  size_t used = 0;
  void clear() { used = 0; text[0] = '\0'; }
  size_t print(const char *value) {
    size_t count = strlen(value);
    if (count > sizeof(text) - used - 1) count = sizeof(text) - used - 1;
    memcpy(text + used, value, count);
    used += count;
    text[used] = '\0';
    return count;
  }
  size_t println(const char *value) { return print(value) + print("\n"); }
  size_t print(char *value) { return print(static_cast<const char *>(value)); }
  template <typename T> size_t print(const T &) {
    return 0;
  }
  template <typename T> size_t println(const T &) {
    return 0;
  }
  template <typename T> size_t println(const T &, int) {
    return 0;
  }
};
