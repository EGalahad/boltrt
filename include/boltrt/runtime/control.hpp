// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/core/types.hpp"
namespace boltrt {
enum class Mode { Hold, Init, Policy };
enum class Event { Hold, Init, Policy, TogglePause, Restart, Stop };
// Snapshot produced by the policy loop; viewers never own controller state.
struct RuntimeStatus {
  std::string robot, policy, task, robot_model, policy_path;
  std::string last_event = "Ready";
  Mode mode = Mode::Hold;
  std::vector<std::string> task_status, task_controls;
  double backend_time = 0, policy_ms = 0;
};
} // namespace boltrt
