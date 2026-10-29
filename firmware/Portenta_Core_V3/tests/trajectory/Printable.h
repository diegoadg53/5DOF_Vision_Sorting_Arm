#pragma once
#include <stddef.h>
class Print;
class Printable {
public:
  virtual size_t printTo(Print &) const = 0;
};
