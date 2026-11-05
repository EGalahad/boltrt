// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/motion.hpp"
namespace boltrt {
struct TrackingContext final : TaskContext {
  explicit TrackingContext(const MotionWindow &reference) : motion(reference) {}
  const MotionWindow &motion;
};
inline const MotionWindow &tracking_motion(const ObservationContext &context) {
  const auto *task = dynamic_cast<const TrackingContext *>(context.task);
  if (!task)
    throw std::runtime_error("tracking observation requires a TrackingContext");
  return task->motion;
}
} // namespace boltrt
