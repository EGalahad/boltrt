// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class HumanoidGPT final : public Observation {
  Selection select_;
  std::string component_, velocity_source_;
  double dt_;
  NativeFk fk_;
  int root_id_;
  bool aligned_ = false;
  Quat alignment_ = Quat::Identity();
  V3 origin_ = V3::Zero();
  Vec current_, proprio_, command_;

public:
  HumanoidGPT(const YAML::Node &n, const ObservationContext &c)
      : select_(n, c), component_(get<std::string>(n, "component", "")),
        velocity_source_(get<std::string>(n, "reference_cvel_source", "motion")),
        dt_(get<double>(n, "reference_dt_s", .02)),
        fk_(n["reference_mjcf_path"]
                ? resolve_policy_asset(n["reference_mjcf_path"].as<std::string>(), c.policy)
                : std::filesystem::path(c.robot.model),
            select_.names),
        root_id_(fk_.body(get<std::string>(n, "root_body_name", c.robot.root_body))) {
    if (dt_ <= 0 || (velocity_source_ != "motion" && velocity_source_ != "mujoco"))
      throw std::runtime_error("invalid Humanoid-GPT reference velocity configuration");
  }
  void reset(const ObservationContext &) override {
    aligned_ = false;
    current_ = Vec(136);
    proprio_ = Vec(93);
    command_ = Vec(43);
  }
  void update(const ObservationContext &c) override {
    auto &cur = select_.at(c, 0);
    int nextstep = std::find(tracking_motion(c).offsets.begin(), tracking_motion(c).offsets.end(),
                             1) == tracking_motion(c).offsets.end()
                       ? 0
                       : 1;
    auto &next = select_.at(c, nextstep);
    auto cp = cur.body_position.at(select_.root), np = next.body_position.at(select_.root);
    auto cq = cur.body_orientation.at(select_.root).normalized(),
         nq = next.body_orientation.at(select_.root).normalized();
    if (!aligned_) {
      alignment_ = yaw(c.state.orientation) * yaw(cq).conjugate();
      origin_ = cp;
      origin_.z() = 0;
      aligned_ = true;
    }
    cp = alignment_ * (cp - origin_);
    np = alignment_ * (np - origin_);
    cq = alignment_ * cq;
    nq = alignment_ * nq;
    V3 angular_gv, linear_gv;
    auto gv = yaw(nq).conjugate();
    if (velocity_source_ == "mujoco") {
      auto qcur = select_.ref(cur), qnext = select_.ref(next);
      fk_.pose(np, nq, qnext);
      for (int i = 0; i < 3; ++i)
        fk_.data->qvel[i] = (np[i] - cp[i]) / dt_;
      Quat delta = (cq.conjugate() * nq).normalized();
      if (delta.w() < 0)
        delta.coeffs() *= -1;
      Eigen::AngleAxisd aa(delta);
      V3 omega = cq * (aa.axis() * aa.angle() / dt_);
      for (int i = 0; i < 3; ++i)
        fk_.data->qvel[3 + i] = omega[i];
      for (size_t i = 0; i < qnext.size(); ++i)
        fk_.data->qvel[fk_.vadr[i]] = (qnext[i] - qcur[i]) / dt_;
      mj_forward(fk_.model, fk_.data);
      angular_gv = gv * Eigen::Map<const V3>(fk_.data->cvel + 6 * root_id_);
      linear_gv = gv * Eigen::Map<const V3>(fk_.data->cvel + 6 * root_id_ + 3);
    } else {
      angular_gv = gv * (alignment_ * angular(next, select_.root));
      linear_gv = gv * (alignment_ * linear(next, select_.root));
    }
    Vec p;
    append(p, c.state.angular_velocity);
    append(p, gravity(c.state.orientation));
    add(p, select_.q(c, false, true));
    add(p, select_.q(c, true));
    add(p, c.previous_action);
    proprio_ = p;
    Vec cmd = select_.ref(next);
    for (size_t i = 0; i < cmd.size(); ++i)
      cmd[i] -= c.policy.default_q.at(select_.state_ids[i]);
    cmd.push_back(float(np.z()));
    append(cmd, gravity(nq));
    append(cmd, angular_gv);
    append(cmd, linear_gv);
    auto d = yaw(cq) * yaw(c.state.orientation).conjugate();
    double a = 2 * std::atan2(d.z(), d.w());
    add(cmd, {float(std::cos(a)), float(std::sin(a))});
    V3 xy = yaw(c.state.orientation).conjugate() * (cp - c.state.root_position);
    add(cmd, {float(xy.x()), float(xy.y())});
    command_ = cmd;
    current_ = p;
    add(current_, cmd);
    if (current_.size() != 136)
      throw std::runtime_error("Humanoid-GPT observation must have 136 values");
  }
  Tensor compute(const ObservationContext &) const override {
    return Tensor::floats(component_ == "proprioception"   ? proprio_
                          : component_ == "motion_command" ? command_
                                                           : current_);
  }
};
} // namespace
void register_humanoid_gpt_observations(ObservationRegistry &r) {
  for (auto name :
       {"humanoid_gpt.humanoid_gpt_pns_obs", "humanoid_gpt.humanoid_gpt_pns_component_obs"})
    r.add(name, [](auto &n, auto &c) { return std::make_unique<HumanoidGPT>(n, c); });
}
} // namespace boltrt
