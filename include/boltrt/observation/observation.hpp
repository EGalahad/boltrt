// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/config/config.hpp"
#include <functional>
#include <memory>
namespace boltrt {
struct TaskContext {
  virtual ~TaskContext() = default;
};
struct ObservationContext {
  const RobotSpec &robot;
  const PolicyConfig &policy;
  const RobotState &state;
  const Vec &previous_action;
  const TensorMap *previous_output = nullptr;
  // Optional task extension; the generic graph knows no task-specific types.
  const TaskContext *task = nullptr;
};
class Observation {
public:
  virtual ~Observation() = default;
  virtual void reset(const ObservationContext &) {}
  virtual void update(const ObservationContext &) {}
  virtual Tensor compute(const ObservationContext &) const = 0;
};
class ObservationRegistry {
public:
  using Factory =
      std::function<std::unique_ptr<Observation>(const YAML::Node &, const ObservationContext &)>;
  void add(const std::string &, Factory);
  void alias(const std::string &, const std::string &);
  std::unique_ptr<Observation> create(const std::string &, const YAML::Node &,
                                      const ObservationContext &) const;
  Names names() const;

private:
  std::map<std::string, Factory> factories_;
  std::map<std::string, std::string> aliases_;
};
void register_common_observations(ObservationRegistry &);
class ObservationGraph {
public:
  ObservationGraph(const ObservationRegistry &, const YAML::Node &, const ObservationContext &);
  void reset(const ObservationContext &);
  TensorMap compute(const ObservationContext &, TensorMap *components = nullptr);

private:
  struct Term {
    std::string name;
    std::unique_ptr<Observation> obs;
  };
  struct Group {
    std::string name;
    std::vector<Term> terms;
  };
  std::vector<Group> groups_;
};
} // namespace boltrt
