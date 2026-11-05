// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/observation.hpp"
#include <algorithm>
#include <deque>
#include <mujoco/mujoco.h>
#include <numeric>
namespace boltrt::ext {
template <class T> T get(const YAML::Node &n, const char *k, T fallback) {
  return n[k] ? n[k].as<T>() : fallback;
}
inline void add(Vec &v, const Vec &x) { v.insert(v.end(), x.begin(), x.end()); }
inline Tensor tensor(Vec v, std::vector<int64_t> shape) {
  Tensor t;
  t.shape = std::move(shape);
  t.data = std::move(v);
  t.validate();
  return t;
}
inline void quat(Vec &v, const Quat &q) {
  add(v, {float(q.w()), float(q.x()), float(q.y()), float(q.z())});
}
inline void columns(Vec &v, const Quat &q) {
  auto m = q.normalized().toRotationMatrix();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j)
      v.push_back(float(m(i, j)));
}
inline void transposed_columns(Vec &v, const Quat &q) {
  auto m = q.normalized().toRotationMatrix();
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 3; ++i)
      v.push_back(float(m(i, j)));
}
inline V3 gravity(const Quat &q) { return q.normalized().conjugate() * V3(0, 0, -1); }
inline V3 angular_delta(const Quat &a, const Quat &b, double dt) {
  Quat d = (b.normalized() * a.normalized().conjugate()).normalized();
  if (d.w() < 0)
    d.coeffs() *= -1;
  Eigen::AngleAxisd aa(d);
  return aa.axis() * aa.angle() / dt;
}
inline Vec roll_pitch(const Quat &q) {
  return {float(std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                           1 - 2 * (q.x() * q.x() + q.y() * q.y()))),
          float(std::asin(std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1., 1.)))};
}
inline int bounded(int n, const char *label, int max = 10000) {
  if (n <= 0 || n > max)
    throw std::runtime_error(std::string("invalid ") + label);
  return n;
}
struct Selection {
  Names names;
  std::vector<size_t> state_ids, motion_ids;
  size_t root;
  Selection(const YAML::Node &n, const ObservationContext &c, bool simulation = false) {
    names = select_names(n["joint_names"], c.robot.joints, simulation ? c.policy.joints : Names{});
    if (!n["joint_names"])
      names = simulation ? c.policy.joints : c.policy.actions;
    for (auto &name : names) {
      state_ids.push_back(index_of(c.robot.joints, name));
      motion_ids.push_back(index_of(tracking_motion(c).joints, name));
    }
    root = index_of(tracking_motion(c).bodies,
                    get<std::string>(n, "root_body_name", c.robot.root_body));
  }
  Vec q(const ObservationContext &c, bool velocity = false, bool relative = false) const {
    Vec out;
    for (auto id : state_ids)
      out.push_back(velocity ? c.state.dq.at(id)
                             : c.state.q.at(id) - (relative ? c.policy.default_q.at(id) : 0));
    return out;
  }
  Vec ref(const MotionFrame &f, bool velocity = false) const {
    Vec out;
    for (auto id : motion_ids)
      out.push_back(velocity ? f.dq.at(id) : f.q.at(id));
    return out;
  }
  const MotionFrame &at(const ObservationContext &c, int offset) const {
    auto it =
        std::find(tracking_motion(c).offsets.begin(), tracking_motion(c).offsets.end(), offset);
    if (it == tracking_motion(c).offsets.end())
      throw std::runtime_error("required motion offset absent: " + std::to_string(offset));
    return tracking_motion(c).frames.at(size_t(it - tracking_motion(c).offsets.begin()));
  }
};
struct History {
  std::deque<Vec> frames;
  void reset(int length, const Vec &v) {
    bounded(length, "history length");
    frames.assign(size_t(length), v);
  }
  void push(const Vec &v) {
    if (frames.empty() || frames.front().size() != v.size())
      throw std::runtime_error("uninitialized history/layout mismatch");
    frames.push_front(v);
    frames.pop_back();
  }
  Vec selected(const std::vector<int> &steps) const {
    Vec v;
    for (int i : steps) {
      if (i < 0)
        throw std::runtime_error("negative history step");
      add(v, frames.at(size_t(i)));
    }
    return v;
  }
  Vec all(bool oldest = false) const {
    Vec v;
    if (oldest) {
      for (auto it = frames.rbegin(); it != frames.rend(); ++it)
        add(v, *it);
    } else
      for (auto &f : frames)
        add(v, f);
    return v;
  }
};
struct NativeFk {
  mjModel *model = nullptr;
  mjData *data = nullptr;
  std::vector<int> qadr, vadr;
  NativeFk(const std::filesystem::path &path, const Names &names) {
    char error[2048]{};
    model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!model)
      throw std::runtime_error("reference MJCF: " + std::string(error));
    data = mj_makeData(model);
    try {
      for (auto &n : names) {
        auto id = mj_name2id(model, mjOBJ_JOINT, n.c_str());
        if (id < 0)
          throw std::runtime_error("reference joint absent: " + n);
        qadr.push_back(model->jnt_qposadr[id]);
        vadr.push_back(model->jnt_dofadr[id]);
      }
    } catch (...) {
      mj_deleteData(data);
      mj_deleteModel(model);
      throw;
    }
  }
  ~NativeFk() {
    mj_deleteData(data);
    mj_deleteModel(model);
  }
  int body(const std::string &n) const {
    int id = mj_name2id(model, mjOBJ_BODY, n.c_str());
    if (id < 0)
      throw std::runtime_error("reference body absent: " + n);
    return id;
  }
  void pose(const V3 &p, const Quat &q, const Vec &joints) {
    mju_zero(data->qvel, model->nv);
    for (int i = 0; i < 3; ++i)
      data->qpos[i] = p[i];
    auto unit = q.normalized();
    data->qpos[3] = unit.w();
    data->qpos[4] = unit.x();
    data->qpos[5] = unit.y();
    data->qpos[6] = unit.z();
    for (size_t i = 0; i < joints.size(); ++i)
      data->qpos[qadr[i]] = joints[i];
    mj_forward(model, data);
  }
  V3 position(int id) const { return Eigen::Map<const V3>(data->xpos + 3 * id); }
  Quat orientation(int id) const {
    auto q = data->xquat + 4 * id;
    return Quat(q[0], q[1], q[2], q[3]);
  }
};
inline std::vector<int> offsets(const YAML::Node &n, const ObservationContext &c) {
  if (!n["future_steps"])
    return tracking_motion(c).offsets;
  if (n["future_steps"].IsScalar())
    return {n["future_steps"].as<int>()};
  return n["future_steps"].as<std::vector<int>>();
}
inline V3 linear(const MotionFrame &f, size_t id) { return f.body_linear_velocity.at(id); }
inline V3 angular(const MotionFrame &f, size_t id) { return f.body_angular_velocity.at(id); }
} // namespace boltrt::ext
namespace boltrt {
void register_sonic_observations(ObservationRegistry &);
void register_heft_observations(ObservationRegistry &);
void register_twist2_observations(ObservationRegistry &);
void register_scalebfm_observations(ObservationRegistry &);
void register_teleopit_observations(ObservationRegistry &);
void register_humanoid_gpt_observations(ObservationRegistry &);
void register_holomotion_observations(ObservationRegistry &);
void register_bfm_zero_observations(ObservationRegistry &);
void register_grit_observations(ObservationRegistry &);
} // namespace boltrt
