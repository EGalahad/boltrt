// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/backend/backend.hpp"
#include <chrono>
namespace boltrt {
// Implemented by SDK adapters only. start() is the first method allowed to
// touch devices.
class HardwareDriver {
public:
  virtual ~HardwareDriver() = default;
  virtual void start() = 0;
  virtual RobotState read() = 0;
  virtual void write(const JointCommand &) = 0;
  virtual JointCommand prime(const RobotState &) = 0;
  virtual void fail_safe() noexcept = 0;
  virtual void stop() noexcept = 0;
};
std::unique_ptr<RobotBackend> inline_hardware(std::unique_ptr<HardwareDriver>, const RobotSpec &,
                                              const BackendConfig &, double);
} // namespace boltrt
