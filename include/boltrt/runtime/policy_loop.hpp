// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/backend/backend.hpp"
#include "boltrt/observation/observation.hpp"
#include "boltrt/runtime/control.hpp"
#include "boltrt/utils/inference.hpp"
#include <nlohmann/json.hpp>
namespace boltrt {
struct LoopOptions {
  double rate = 50;
  size_t steps = 0;
  int init_steps = 500;
  bool start_policy = false, keyboard = true, run_once = false, realtime = true;
  std::string task_name;
  std::filesystem::path record, events, timings;
};
class PolicyLoop {
public:
  PolicyLoop(const RobotSpec &, const PolicyConfig &, RobotBackend &, Inference &,
             const ObservationRegistry &, LoopOptions);
  virtual ~PolicyLoop() = default;
  void validate();
  void run();
  static void request_stop();

protected:
  // Task hooks contain optional domain behavior; the base loop has no motion dependency.
  virtual void prepare_task() {}
  virtual void start_task(bool) {}
  virtual void bind_task_context(ObservationContext &) const {}
  virtual void enter_policy_task() {}
  virtual void update_task() {}
  virtual bool handle_task_event(Event, std::string &) { return false; }
  virtual bool task_finished() const { return false; }
  virtual void update_task_status(RuntimeStatus &) const {}
  virtual void append_task_record(nlohmann::json &, bool) const {}

private:
  const RobotSpec &robot_;
  const PolicyConfig &policy_;
  RobotBackend &backend_;
  Inference &inference_;
  const ObservationRegistry &registry_;
  LoopOptions options_;
};
} // namespace boltrt
