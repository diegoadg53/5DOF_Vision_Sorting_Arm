#pragma once
#include <stdint.h>

namespace CameraTracking {
// Applied ONCE to detected parts after pixel -> robot-mm homography, before
// tracking, workspace admission and pickup-height calculation.
// Positive moves the pickup toward larger robot X; negative toward smaller X.
// Example: measured X=300 mm, offset=+15 mm -> tracked/pickup X=315 mm.
// Does not shift April-tag calibration, HOME, bins or the synthetic test part.
constexpr float partXOffsetMm = -9.0f; // Build default; controller tuning/saved settings override it.

namespace ClassificationConfidence {
// EDIT HERE: ID-aware AUTO waits before reserving a part. The logs showed
// initial ID 8 changing to the correct class within about 550 ms.
constexpr uint32_t minimumStableMs = 500;
constexpr uint8_t minimumSamples = 3;
constexpr uint32_t sampleIntervalMs = 80, maximumGapMs = 750;
static_assert(minimumSamples >= 2 && minimumStableMs >= sampleIntervalMs &&
              maximumGapMs >= sampleIntervalMs, "Invalid classification confidence settings");
}

// Pixel coordinates, BEFORE homography. Require the WHOLE bounding box inside
// the region, not just its centre. Plain integer arithmetic also handles odd
// box dimensions without rounding an edge back into the region.
struct PixelRectangle {
  int32_t left, top, right, bottom;
  bool containsBox(uint16_t x, uint16_t y, uint16_t width, uint16_t height) const {
    return left >= 0 && top >= 0 && right > left && bottom > top && width && height &&
        2 * int32_t(x) - width >= 2 * left &&
        2 * int32_t(x) + width <= 2 * right &&
        2 * int32_t(y) - height >= 2 * top &&
        2 * int32_t(y) + height <= 2 * bottom;
  }
};
namespace PartRegion {
// EDIT HERE: central 80% of 640x480. Reject the entire observation if any
// box edge falls outside; prediction continues from the last accepted box.
constexpr PixelRectangle rectangle = {64, 48, 576, 432};
}
namespace VelocityRegion {
constexpr int32_t imageWidth = 640, imageHeight = 480;
// EDIT HERE: tall moving-AprilTag lane. Fixed homography tags must stay outside.
constexpr PixelRectangle rectangle = {224, 48, 416, 432};
constexpr uint8_t minimumTagId = 0, maximumTagId = 3;
static_assert(minimumTagId <= maximumTagId, "Invalid moving-tag ID interval");
static_assert(rectangle.left >= 0 && rectangle.top >= 0 &&
              rectangle.right <= imageWidth && rectangle.bottom <= imageHeight &&
              rectangle.left < rectangle.right && rectangle.top < rectangle.bottom,
              "Velocity rectangle must fit inside the camera image");
}

// AUTO uses ONLY the speed calibrated with a flat moving tag. Detection-box
// motion never estimates/verifies belt speed. Recalibrate after every change.
} // namespace CameraTracking
