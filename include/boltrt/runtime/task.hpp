// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/runtime/policy_loop.hpp"
#include <functional>
namespace boltrt {
struct TaskServices {
  const RobotSpec &robot;
  const PolicyConfig &policy;
  RobotBackend &backend;
  Inference &inference;
  ObservationRegistry &observations;
  LoopOptions options;
  const std::map<std::string, std::string> &arguments;
};
class TaskRegistry {
public:
  using Factory = std::function<std::unique_ptr<PolicyLoop>(TaskServices)>;
  using Observations = std::function<void(ObservationRegistry &)>;
  TaskRegistry();
  void add(std::string name, Factory, Observations = {}, Names options = {});
  void prepare_observations(const std::string &, ObservationRegistry &) const;
  std::unique_ptr<PolicyLoop> create(const std::string &, TaskServices) const;
  Names names() const;
  Names option_names() const;

private:
  struct Definition {
    Factory factory;
    Observations observations;
    Names options;
  };
  std::map<std::string, Definition> definitions_;
};
} // namespace boltrt
