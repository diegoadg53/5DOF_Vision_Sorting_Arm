#pragma once
#include <stdint.h>

// AprilTag's physical ID is supplied in content, not the learned/custom class
// ID. Reject malformed strings rather than letting atoi() turn them into ID 0.
inline bool parseAprilTagId(const char *text, uint8_t &id) {
  if (!text) return false;
  auto space = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
  while (space(*text)) ++text;
  if (*text < '0' || *text > '9') return false;
  unsigned value = 0;
  while (*text >= '0' && *text <= '9') {
    value = value * 10 + unsigned(*text++ - '0');
    if (value > 255) return false;
  }
  while (space(*text)) ++text;
  if (*text) return false;
  id = uint8_t(value);
  return true;
}

// Preserve every detection, including duplicate physical IDs and frames with
// fewer than four tags. Portenta owns homography/velocity region selection.
template <typename Frame, typename Packet>
void packAprilTags(const Frame &frame, Packet &packet) {
  packet.vision.amountoObjects = 0;
  const unsigned available = sizeof(frame.boxes) / sizeof(frame.boxes[0]);
  const unsigned capacity = sizeof(packet.vision.objList) / sizeof(packet.vision.objList[0]);
  for (unsigned i = 0; i < frame.count && i < available; ++i) {
    const auto &box = frame.boxes[i];
    uint8_t id;
    if (!parseAprilTagId(box.content, id) || box.width <= 0 || box.height <= 0 ||
        box.xCenter < 0 || box.yCenter < 0) continue;
    if (packet.vision.amountoObjects >= capacity) break;
    auto &out = packet.vision.objList[packet.vision.amountoObjects++];
    out.xCenter = box.xCenter; out.yCenter = box.yCenter;
    out.width = box.width; out.height = box.height; out.ID = id;
  }
}
