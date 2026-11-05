// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/motion.hpp"
#include <iostream>
using namespace boltrt;
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("fixture directory required");
    std::filesystem::path root = argv[1];
    RobotSpec robot;
    robot.joints = {"b", "a"};
    robot.bodies = {"pelvis"};
    robot.root_body = "pelvis";
    robot.model = (root / "robot.xml").string();
    robot.armature = Vec(2);
    robot.friction = Vec(2);
    robot.effort = {10, 10};
    robot.initial_q = {0, 0};
    for (auto file : {"c_float32.npz", "f_float64.npz"}) {
      auto source = file_motion(root / file, robot, {-1, 0, 1}, 50, false);
      auto start = source->sample(false, true);
      for (auto &f : start.frames)
        if (f.q != Vec{0, 0})
          throw std::runtime_error("paused lookahead not collapsed");
      auto w = source->sample(true, false);
      if (!w.ended || w.frame != 1 || std::abs(w.frames[1].q[0] + 0.2f) > 1e-6 ||
          std::abs(w.frames[1].q[1] - 0.4f) > 1e-6)
        throw std::runtime_error("NPZ storage/name mapping mismatch");
      for (int i = 0; i < 5; ++i)
        w = source->sample(true, false);
      if (w.frame != 1)
        throw std::runtime_error("final frame not held");
      source->reset();
      if (source->sample(false, true).frame != 0)
        throw std::runtime_error("motion reset failed");
    }
    robot.model = (root / "named_root/robot.xml").string();
    auto named = file_motion(root / "named_root/motion.npz", robot, {0}, 50, false);
    auto canonical = file_motion(root / "c_float32.npz", robot, {0}, 50, false);
    const auto named_frame = named->sample(true, false).frames.at(0);
    const auto canonical_frame = canonical->sample(true, false).frames.at(0);
    if (named_frame.q != canonical_frame.q ||
        (named_frame.body_position.at(0) - canonical_frame.body_position.at(0)).norm() > 1e-7)
      throw std::runtime_error("named freejoint changes reference pose");
    robot.model = (root / "robot.xml").string();
    auto pose = pose_motion(robot, {.1f, -.1f}, {-1, 0, 1}, 50);
    const auto pose_window = pose->sample(true, false);
    if (pose_window.frames.size() != 3 || pose_window.ended ||
        pose_window.frames[1].q != Vec{.1f, -.1f} || pose_window.frames[1].dq != Vec{0, 0})
      throw std::runtime_error("default pose reference contract failed");
    auto source = file_motion(root / "source_velocity/source_velocity.npz", robot, {0}, 50, false);
    auto velocity = source->sample(true, false).frames.at(0).dq;
    if (std::abs(velocity[0] + 10.f) > 1e-5 || std::abs(velocity[1] - 5.f) > 1e-5)
      throw std::runtime_error("source-fps qvel must be interpolated before FK");
    auto supplied = file_motion(root / "supplied_velocity/motion.npz", robot, {0}, 50, false);
    auto supplied_dq = supplied->sample(true, false).frames.at(0).dq;
    if (supplied_dq != Vec{9, 7})
      throw std::runtime_error("supplied named velocity convention not preserved");
    std::cout << "NPZ C/F-order, named remapping, pause/reset/end and "
                 "source-fps velocity contracts passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
