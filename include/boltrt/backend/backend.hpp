// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/config/config.hpp"
#include <functional>
#include <memory>
#include <optional>
namespace boltrt {
class RobotBackend {
public:
  virtual ~RobotBackend() = default;
  virtual void start() = 0;
  // Latest robot feedback; nullopt means this backend session has closed.
  virtual std::optional<RobotState> read_state() = 0;
  virtual void submit(const JointCommand &) = 0;
  virtual void stop() noexcept = 0;
};
using BackendFactory =
    std::function<std::unique_ptr<RobotBackend>(const RobotSpec &, const BackendConfig &, double)>;
class BackendRegistry {
public:
  void add(const std::string &, BackendFactory);
  std::unique_ptr<RobotBackend> create(const RobotSpec &, const BackendConfig &, double) const;
  Names names() const;

private:
  std::map<std::string, BackendFactory> factories_;
};
void register_mujoco_backend(BackendRegistry &);
void register_g1_backend(BackendRegistry &);
void register_rp1_backend(BackendRegistry &);
void register_replay_backend(BackendRegistry &);
void validate_command(const JointCommand &, size_t);
} // namespace boltrt
