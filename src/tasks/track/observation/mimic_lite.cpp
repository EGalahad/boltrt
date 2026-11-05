// SPDX-License-Identifier: Apache-2.0
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/observation.hpp"
#include <algorithm>
namespace boltrt {
namespace {
class MotionObservation : public Observation {
public:
  MotionObservation(const YAML::Node &n, const ObservationContext &c) {
    require_keys(n,
                 {"_target_", "future_steps", "joint_names", "body_names", "root_body_name",
                  "anchor_body_name", "joint_order", "body_order", "noise_std",
                  "exclude_current_step", "motion_duration_second"},
                 "motion observation");
    auto cfg = c.policy.yaml["motion"];
    auto f = n["future_steps"];
    if (f) {
      if (f.IsScalar())
        offsets_ = {f.as<int>()};
      else
        offsets_ = f.as<std::vector<int>>();
    } else
      offsets_ = tracking_motion(c).offsets;
    if (offsets_.empty())
      throw std::runtime_error("empty future steps");
    for (int offset : offsets_)
      if (std::find(tracking_motion(c).offsets.begin(), tracking_motion(c).offsets.end(), offset) ==
          tracking_motion(c).offsets.end())
        throw std::runtime_error("future step missing from motion");
    auto jo = n["joint_order"] ? n["joint_order"].as<std::string>() : "simulation",
         bo = n["body_order"] ? n["body_order"].as<std::string>() : "simulation";
    if (jo != "simulation" && jo != "policy" && jo != "given" && jo != "source")
      throw std::runtime_error("unsupported joint_order");
    if (bo != "simulation" && bo != "given" && bo != "source")
      throw std::runtime_error("unsupported body_order");
    auto joints = select_names(n["joint_names"] ? n["joint_names"] : cfg["joint_names"],
                               tracking_motion(c).joints,
                               jo == "simulation" ? c.policy.joints
                               : jo == "policy"   ? c.policy.actions
                                                  : Names{});
    auto bodies =
        select_names(n["body_names"] ? n["body_names"] : cfg["body_names"],
                     tracking_motion(c).bodies, bo == "simulation" ? c.policy.bodies : Names{});
    for (auto &j : joints)
      joints_.push_back(index_of(tracking_motion(c).joints, j));
    for (auto &b : bodies)
      bodies_.push_back(index_of(tracking_motion(c).bodies, b));
    root_ = index_of(tracking_motion(c).bodies,
                     n["root_body_name"]     ? n["root_body_name"].as<std::string>()
                     : cfg["root_body_name"] ? cfg["root_body_name"].as<std::string>()
                                             : "pelvis");
    anchor_ = index_of(tracking_motion(c).bodies,
                       n["anchor_body_name"]     ? n["anchor_body_name"].as<std::string>()
                       : cfg["anchor_body_name"] ? cfg["anchor_body_name"].as<std::string>()
                                                 : "torso_link");
  }

protected:
  const MotionFrame &at(const ObservationContext &c, int offset) const {
    auto it =
        std::find(tracking_motion(c).offsets.begin(), tracking_motion(c).offsets.end(), offset);
    return tracking_motion(c).frames.at(size_t(it - tracking_motion(c).offsets.begin()));
  }
  std::vector<int> offsets_;
  std::vector<size_t> joints_, bodies_;
  size_t root_, anchor_;
};
class RefJoint final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    for (int o : offsets_)
      for (auto i : joints_)
        v.push_back(at(c, o).q.at(i));
    return Tensor::floats(std::move(v));
  }
};
class RefJointVelocity final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    for (int o : offsets_)
      for (auto i : joints_)
        v.push_back(at(c, o).dq.at(i));
    return Tensor::floats(std::move(v));
  }
};
class JointPositionRelative final : public Observation {
  std::vector<size_t> ids_;

public:
  JointPositionRelative(const YAML::Node &n, const ObservationContext &c) {
    require_keys(n, {"_target_", "joint_names"}, "relative joint position");
    for (auto &name : select_names(n["joint_names"], c.robot.joints, c.policy.joints))
      ids_.push_back(index_of(c.robot.joints, name));
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    for (auto i : ids_)
      v.push_back(c.state.q.at(i) - c.policy.default_q.at(i));
    return Tensor::floats(std::move(v));
  }
};
class RefBodyPosition final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    auto &current = at(c, 0);
    auto origin = current.body_position[anchor_];
    origin.z() = 0;
    auto rot = projected_yaw(current.body_orientation[anchor_]).conjugate();
    for (int o : offsets_)
      for (auto b : bodies_)
        append(v, rot * (at(c, o).body_position[b] - origin));
    return Tensor::floats(std::move(v));
  }
};
class RefBodyOrientation final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    auto rot = projected_yaw(at(c, 0).body_orientation[anchor_]).conjugate();
    for (int o : offsets_)
      for (auto b : bodies_)
        append_rotation6(v, rot * at(c, o).body_orientation[b]);
    return Tensor::floats(std::move(v));
  }
};
class RefRootOrientation final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  void reset(const ObservationContext &c) override {
    alignment_ = projected_yaw(at(c, 0).body_orientation[root_]) *
                 projected_yaw(c.state.orientation).conjugate();
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    auto rot = (alignment_ * c.state.orientation).conjugate();
    for (int o : offsets_)
      append_rotation6(v, rot * at(c, o).body_orientation[root_]);
    return Tensor::floats(std::move(v));
  }

private:
  Quat alignment_ = Quat::Identity();
};
class RootZ final : public MotionObservation {
public:
  using MotionObservation::MotionObservation;
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    for (int o : offsets_)
      v.push_back(static_cast<float>(at(c, o).body_position[root_].z()));
    return Tensor::floats(std::move(v));
  }
};
class RootDiff final : public MotionObservation {
public:
  RootDiff(const YAML::Node &n, const ObservationContext &c)
      : MotionObservation(n, c),
        exclude_(n["exclude_current_step"] ? n["exclude_current_step"].as<bool>() : true) {}
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    auto &cur = at(c, 0);
    for (int o : offsets_)
      if (!exclude_ || o != 0)
        append(v, rotate(cur.body_orientation[root_].conjugate(),
                         at(c, o).body_position[root_] - cur.body_position[root_]));
    return Tensor::floats(std::move(v));
  }

private:
  bool exclude_;
};
class Phase final : public MotionObservation {
public:
  Phase(const YAML::Node &n, const ObservationContext &c)
      : MotionObservation(n, c), steps_(int(n["motion_duration_second"].as<double>() * 50)) {
    if (steps_ <= 0)
      throw std::runtime_error("invalid phase duration");
  }
  Tensor compute(const ObservationContext &c) const override {
    return Tensor::floats({float(tracking_motion(c).frame % steps_) / float(steps_)});
  }

private:
  int steps_;
};
template <class T> void add(ObservationRegistry &r, const std::string &name) {
  r.add("mimic_lite." + name, [](auto &n, auto &c) { return std::make_unique<T>(n, c); });
  r.alias("hdmi." + name, "mimic_lite." + name);
}
} // namespace
void register_mimic_observations(ObservationRegistry &r) {
  add<RefJoint>(r, "ref_joint_pos_future");
  add<RefJointVelocity>(r, "ref_joint_vel_future");
  add<JointPositionRelative>(r, "joint_pos_rel");
  add<RefBodyPosition>(r, "ref_body_pos_future_local");
  add<RefBodyOrientation>(r, "ref_body_ori_future_local");
  add<RefRootOrientation>(r, "ref_root_ori_future_b");
  add<RootZ>(r, "root_pos_future_z");
  add<RootDiff>(r, "root_pos_future_diff");
  add<Phase>(r, "ref_motion_phase");
}
} // namespace boltrt
