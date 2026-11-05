// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/utils/mujoco/model.hpp"
#include <memory>
#include <mutex>
namespace boltrt {
class MujocoViewer {
public:
  explicit MujocoViewer(RobotModel &);
  ~MujocoViewer();
  void render();
  bool paused() const;
  std::unique_lock<std::recursive_mutex> lock();
  bool closed() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace boltrt
