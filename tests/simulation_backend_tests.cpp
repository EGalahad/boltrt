// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/backend.hpp"
#include <chrono>
#include <iostream>
#include <thread>
using namespace boltrt;
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("robot fixture required");
    RobotSpec r;
    r.joints = {"b", "a"};
    r.bodies = {"pelvis"};
    r.root_body = "pelvis";
    r.model = argv[1];
    r.armature = {0, 0};
    r.friction = {0, 0};
    r.effort = {10, 10};
    r.initial_q = {.1f, -.2f};
    BackendConfig cfg;
    cfg.yaml = YAML::Load("type: mujoco\nphysics_dt: 0.005\nviewer: false\nfloor: true");
    cfg.type = "mujoco";
    cfg.dt = .005;
    BackendRegistry reg;
    register_mujoco_backend(reg);
    auto b = reg.create(r, cfg, 50);
    b->start();
    auto before = b->read_state().value();
    JointCommand c{{.5f, .5f}, {0, 0}, {0, 0}, {10, 10}, {1, 1}};
    b->submit(c);
    auto moved = b->read_state().value();
    if (std::abs(moved.time - before.time - .02) > 1e-10)
      throw std::runtime_error("simulation did not advance");
    for (float torque : moved.torque)
      if (std::abs(torque) > 10)
        throw std::runtime_error("fixed robot torque limit exceeded");
    b->stop();
    if (b->read_state())
      throw std::runtime_error("stopped backend still returns feedback");
    // Online physics must progress without any policy reads or new commands.
    cfg.realtime = true;
    // Online clocks are independent: 60 Hz policy need not divide 200 Hz physics.
    auto online = reg.create(r, cfg, 60);
    online->start();
    online->submit(c);
    const auto start = online->read_state().value();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    const auto end = online->read_state().value();
    online->stop();
    if (end.time <= start.time || end.tick <= start.tick)
      throw std::runtime_error("physics still depends on policy reads");
    std::cout << "offline deterministic stepping, fixed limits, online independent physics and "
                 "close passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
