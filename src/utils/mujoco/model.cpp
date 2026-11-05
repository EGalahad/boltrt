// SPDX-License-Identifier: Apache-2.0
#include "boltrt/utils/mujoco/model.hpp"
namespace boltrt {
RobotModel::RobotModel(const RobotSpec &r, std::function<void(mjSpec *)> configure) {
  char error[2048]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXML(r.model.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec)
    throw std::runtime_error("MJCF parse: " + std::string(error));
  if (configure)
    configure(spec.get());
  model.reset(mj_compile(spec.get(), nullptr));
  if (!model)
    throw std::runtime_error("MJCF compile: " + std::string(mjs_getError(spec.get())));
  if (model->nq < 7 || model->nv < 6 || model->jnt_type[0] != mjJNT_FREE)
    throw std::runtime_error("expected a floating-base robot");
  for (size_t i = 0; i < r.joints.size(); ++i) {
    int j = mj_name2id(model.get(), mjOBJ_JOINT, r.joints[i].c_str());
    if (j < 0 || model->jnt_type[j] == mjJNT_FREE || model->jnt_type[j] == mjJNT_BALL)
      throw std::runtime_error("missing/non-scalar joint: " + r.joints[i]);
    qadr.push_back(model->jnt_qposadr[j]);
    vadr.push_back(model->jnt_dofadr[j]);
    model->dof_armature[vadr.back()] = r.armature[i];
    model->dof_frictionloss[vadr.back()] = r.friction[i];
  }
  for (auto &n : r.bodies) {
    int b = mj_name2id(model.get(), mjOBJ_BODY, n.c_str());
    if (b < 0)
      throw std::runtime_error("missing body: " + n);
    bodies.push_back(b);
  }
  data.reset(mj_makeData(model.get()));
  if (!data)
    throw std::runtime_error("mj_makeData failed");
}
void RobotModel::set_pose(const Vec &q, const V3 &p, const Quat &ori) {
  if (q.size() != qadr.size())
    throw std::runtime_error("FK joint shape mismatch");
  for (int i = 0; i < 3; ++i)
    data->qpos[i] = p[i];
  data->qpos[3] = ori.w();
  data->qpos[4] = ori.x();
  data->qpos[5] = ori.y();
  data->qpos[6] = ori.z();
  for (size_t i = 0; i < q.size(); ++i)
    data->qpos[qadr[i]] = q[i];
  mj_forward(model.get(), data.get());
}
} // namespace boltrt
