// SPDX-License-Identifier: Apache-2.0
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/observation.hpp"
#include "pose_packet.hpp"
#include <fstream>
#include <iostream>
#include <unistd.h>
using namespace boltrt;
void check(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
int main() {
  try {
    RobotSpec robot;
    robot.joints = {"a", "b"};
    robot.bodies = {"pelvis"};
    robot.root_body = "pelvis";
    PolicyConfig policy;
    policy.joints = robot.joints;
    policy.actions = robot.joints;
    policy.bodies = robot.bodies;
    policy.default_q = {0, 0};
    RobotState state;
    state.q = {1, 2};
    state.dq = {0, 0};
    MotionWindow motion;
    motion.joints = robot.joints;
    motion.bodies = robot.bodies;
    motion.offsets = {0};
    MotionFrame f;
    f.q = {0, 0};
    f.dq = {0, 0};
    f.body_position = {V3::Zero()};
    f.body_orientation = {Quat::Identity()};
    motion.frames = {f};
    Vec action{0, 0};
    TensorMap previous;
    TrackingContext tracking_context(motion);
    ObservationContext ctx{robot, policy, state, action, &previous, &tracking_context};
    ObservationRegistry reg;
    register_external_observations(reg);
    auto kv =
        reg.create("holomotion.holomotion_zero_state",
                   YAML::Load("shape: [1, 2]\nrope_max_seq_len: 4\nrope_reset_margin: 2"), ctx);
    auto step = reg.create("holomotion.holomotion_step_idx",
                           YAML::Load("rope_max_seq_len: 4\nrope_reset_margin: 2"), ctx);
    kv->reset(ctx);
    step->reset(ctx);
    check(std::get<Vec>(kv->compute(ctx).data) == Vec({0, 0}), "KV cache must start zero");
    check(std::get<std::vector<int64_t>>(step->compute(ctx).data) == std::vector<int64_t>({0}),
          "RoPE index starts at 0");
    previous["next/past_key_values"] = Tensor::floats({3, 4});
    check(std::get<Vec>(kv->compute(ctx).data) == Vec({3, 4}), "KV feedback lost");
    check(std::get<std::vector<int64_t>>(step->compute(ctx).data) == std::vector<int64_t>({1}),
          "RoPE index did not advance");
    check(std::get<Vec>(kv->compute(ctx).data) == Vec({0, 0}), "RoPE wrap did not clear KV cache");
    check(std::get<std::vector<int64_t>>(step->compute(ctx).data) == std::vector<int64_t>({0}),
          "RoPE wrap did not reset index");
    kv->reset(ctx);
    step->reset(ctx);
    check(std::get<Vec>(kv->compute(ctx).data) == Vec({0, 0}),
          "policy reset did not clear KV cache");
    auto history = reg.create("sonic.sonic_joint_pos_rel_history",
                              YAML::Load("history_steps: [0, 1, 2]"), ctx);
    history->reset(ctx);
    history->update(ctx);
    state.q = {3, 4};
    history->update(ctx);
    check(std::get<Vec>(history->compute(ctx).data) == Vec({1, 2, 1, 2, 3, 4}),
          "SONIC must be oldest-to-newest with first-frame padding");
    auto path = std::filesystem::temp_directory_path() /
                ("boltrt-yaml-merge-" + std::to_string(getpid()) + ".yaml");
    {
      std::ofstream out(path);
      out << "joint_names_simulation: [a, b]\npolicy_joint_names: [a, "
             "b]\nbody_names_simulation: [pelvis]\naction_scale: 1\nshared: "
             "&shared {history_steps: [0, 2], noise_std: 0.2}\nobservation:\n  "
             "obs:\n    joint_pos_history:\n      <<: *shared\n      "
             "noise_std: 0.0\n";
    }
    robot.effort = Vec(robot.joints.size(), 10);
    auto loaded = load_policy(path, robot);
    std::filesystem::remove(path);
    check(loaded.yaml["observation"]["obs"]["joint_pos_history"]["history_steps"].size() == 2,
          "YAML merge did not carry shared parameters");
    check(loaded.yaml["observation"]["obs"]["joint_pos_history"]["noise_std"].as<float>() == 0,
          "explicit YAML key must override merge");
    nlohmann::json header = {
        {"endian", "le"},
        {"fields", {{{"name", "joint_pos"}, {"dtype", "f32"}, {"shape", {1, 2}}}}}};
    std::string packet = "pose" + header.dump();
    packet.resize(4 + 1280, '\0');
    float values[] = {1.25f, -2.5f};
    packet.append(reinterpret_cast<const char *>(values), sizeof(values));
    auto decoded = decode_pose_packet(packet.data(), packet.size());
    check(decoded["joint_pos"][0][1] == -2.5, "packed pose field decode failed");
    bool rejected = false;
    try {
      decode_pose_packet(packet.data(), packet.size() - 1);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "truncated packed pose was accepted");
    check(pose_bool(nlohmann::json{{"first", {true}}}, "first"),
          "publisher array boolean not decoded");
    std::cout << "external history, KV/RoPE reset, YAML merge and packed pose "
                 "contracts passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
