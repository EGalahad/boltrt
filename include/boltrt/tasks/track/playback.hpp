// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/tasks/track/motion.hpp"
namespace boltrt {
// Owns reference playback only. It never observes backend time or viewer state.
// A reference tick is driven by the consumer's clock (policy or standalone teleop).
class MotionPlayback {
public:
  explicit MotionPlayback(MotionSource &source) : source_(source) { restart(); }
  void restart() {
    source_.reset();
    paused_ = true;
    window_ = source_.sample(false, true);
  }
  void play() { paused_ = false; }
  void toggle_pause() { paused_ = !paused_; }
  bool paused() const { return paused_; }
  const MotionWindow &current() const { return window_; }
  const MotionWindow &tick() {
    window_ = source_.sample(true, paused_);
    if (window_.ended)
      paused_ = true;
    return window_;
  }

private:
  MotionSource &source_;
  bool paused_ = true;
  MotionWindow window_;
};
} // namespace boltrt
