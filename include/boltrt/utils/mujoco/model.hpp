// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/config/config.hpp"
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
namespace boltrt {
struct ModelDeleter {
  void operator()(mjModel *p) const { mj_deleteModel(p); }
};
struct DataDeleter {
  void operator()(mjData *p) const { mj_deleteData(p); }
};
class RobotModel {
public:
  explicit RobotModel(const RobotSpec &, std::function<void(mjSpec *)> configure = {});
  void set_pose(const Vec &, const V3 &, const Quat &);
  std::unique_ptr<mjModel, ModelDeleter> model;
  std::unique_ptr<mjData, DataDeleter> data;
  std::vector<int> qadr, vadr, bodies;
};
} // namespace boltrt
