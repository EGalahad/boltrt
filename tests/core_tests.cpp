// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/backend.hpp"
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/observation.hpp"
#include <functional>
#include <iostream>
using namespace boltrt;
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void rejects(std::function<void()> f) {
  bool rejected = false;
  try {
    f();
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected, "expected rejection");
}
int main() {
  try {
    auto mapping = YAML::Load("b: 2\na: 1");
    auto v = resolve_values(mapping, {"a", "b"}, 0, true);
    check(v == Vec({1, 2}), "name mapping changed by YAML order");
    rejects([] { resolve_values(YAML::Load("'.*': 1\na: 2"), {"a"}); });
    rejects([] { resolve_values(YAML::Load("[1]"), {"a", "b"}); });
    Tensor invalid = Tensor::floats({1, 2});
    invalid.shape = {3};
    rejects([&] { invalid.validate(); });
    JointCommand command{{1}, {0}, {0}, {1}, {1}};
    validate_command(command, 1);
    command.kp[0] = -1;
    rejects([&] { validate_command(command, 1); });
    RobotSpec robot;
    robot.joints = {"b", "a"};
    PolicyConfig policy;
    policy.joints = {"a", "b"};
    policy.actions = {"b", "a"};
    RobotState state;
    state.q = {20, 10};
    state.dq = {2, 1};
    MotionWindow motion;
    Vec prev{0, 0};
    TrackingContext tracking_context(motion);
    ObservationContext ctx{robot, policy, state, prev, nullptr, &tracking_context};
    ObservationRegistry reg;
    register_tracking_proprioception(reg);
    auto term =
        reg.create("mimic_lite.joint_pos_history", YAML::Load("history_steps: [0, 2]"), ctx);
    term->reset(ctx);
    state.q = {40, 30};
    term->update(ctx);
    check(std::get<Vec>(term->compute(ctx).data) == Vec({30, 40, 10, 20}),
          "history/layout mismatch");
    auto actions = reg.create("mimic_lite.prev_actions", YAML::Load("steps: 2"), ctx);
    actions->reset(ctx);
    check(bool(reg.create("prev_actions", YAML::Load("steps: 1"), ctx)),
          "namespace alias caused short-name ambiguity");
    prev = {1, 2};
    actions->update(ctx);
    prev = {3, 4};
    actions->update(ctx);
    check(std::get<Vec>(actions->compute(ctx).data) == Vec({3, 4, 1, 2}),
          "action history flatten order");
    rejects([&] {
      reg.add("mimic_lite.prev_actions",
              [](auto &, auto &) -> std::unique_ptr<Observation> { return {}; });
    });
    rejects([&] { reg.create("missing", YAML::Node{}, ctx); });
    register_mimic_observations(reg);
    motion.joints = {"b", "a"};
    motion.bodies = {"pelvis"};
    motion.offsets = {0};
    motion.frames.resize(1);
    motion.frames[0].q = {100, 200};
    motion.frames[0].dq = {8, 9};
    policy.yaml = YAML::Load("motion: {root_body_name: pelvis, anchor_body_name: pelvis, "
                             "joint_names: [a, b], body_names: [pelvis]}");
    auto refvel =
        reg.create("mimic_lite.ref_joint_vel_future", YAML::Load("future_steps: [0]"), ctx);
    check(std::get<Vec>(refvel->compute(ctx).data) == Vec({9, 8}),
          "reference velocity/layout mismatch");
    policy.default_q = {1, 2};
    auto relpos = reg.create("mimic_lite.joint_pos_rel", YAML::Load("{}"), ctx);
    check(std::get<Vec>(relpos->compute(ctx).data) == Vec({28, 39}),
          "relative position/default layout mismatch");
    std::cout << "core contracts passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
