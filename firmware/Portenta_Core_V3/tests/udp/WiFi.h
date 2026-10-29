#pragma once
#include "FakeMbedUdp.h"
struct FakeWifi {
  void *getNetwork() { return nullptr; }
};
extern FakeWifi WiFi;
