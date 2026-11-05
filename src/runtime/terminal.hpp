// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/runtime/control.hpp"
#include <chrono>
#include <termios.h>
namespace boltrt {
// Terminal-only controls; simulation windows remain dedicated to visualization.
class TerminalControl {
public:
  explicit TerminalControl(bool enabled);
  ~TerminalControl();
  TerminalControl(const TerminalControl &) = delete;
  TerminalControl &operator=(const TerminalControl &) = delete;
  std::vector<Event> poll();
  void render(const RuntimeStatus &);
  bool interactive() const { return display_; }
  void close() noexcept;

private:
  bool active_ = false, display_ = false;
  termios old_{};
  std::string last_key_ = "none", previous_event_;
  std::chrono::steady_clock::time_point last_draw_{};
};
} // namespace boltrt
