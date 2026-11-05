// SPDX-License-Identifier: Apache-2.0
#include "viewer.hpp"
#include "glfw_adapter.h"
#include "simulate.h"
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
namespace boltrt {
struct MujocoViewer::Impl {
  RobotModel &robot;
  mjvCamera camera{};
  mjvOption option{};
  mjvPerturb perturb{};
  std::unique_ptr<mujoco::Simulate> sim;
  std::thread render_thread;
  std::atomic<bool> exited{false};
  explicit Impl(RobotModel &r) : robot(r) {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultPerturb(&perturb);
    camera.type = mjCAMERA_TRACKING;
    camera.trackbodyid = robot.bodies.front();
    camera.distance = 3;
    camera.azimuth = 120;
    camera.elevation = -20;
    std::promise<void> ready;
    auto future = ready.get_future();
    render_thread = std::thread([this, &ready] {
      try {
        sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &camera,
                                                 &option, &perturb, false);
        sim->ui0_enable = 0;
        sim->ui1_enable = 0;
        ready.set_value();
      } catch (...) {
        ready.set_exception(std::current_exception());
        return;
      }
      sim->RenderLoop();
      exited = true;
    });
    try {
      future.get();
    } catch (...) {
      render_thread.join();
      throw;
    }
    sim->Load(robot.model.get(), robot.data.get(), "boltrt");
    {
      mujoco::MutexLock lock(sim->mtx);
      camera.type = mjCAMERA_TRACKING;
      camera.trackbodyid = robot.bodies.front();
      camera.distance = 3;
      camera.azimuth = 120;
      camera.elevation = -20;
    }
    std::cout << "MuJoCo official viewer: native keys; Space=pause simulation, "
                 "Backspace=reset simulation; F1=help. "
                 "Motion and policy controls are in the terminal.\n";
  }
  ~Impl() {
    if (sim)
      sim->exitrequest.store(1);
    if (render_thread.joinable())
      render_thread.join();
  }
};
MujocoViewer::MujocoViewer(RobotModel &r) : impl_(std::make_unique<Impl>(r)) {}
MujocoViewer::~MujocoViewer() = default;
void MujocoViewer::render() {
  auto &s = *impl_;
  if (s.exited)
    return;
  mujoco::MutexLock lock(s.sim->mtx);
  s.sim->Sync();
  s.sim->scn.flags[mjRND_HAZE] = 0;
}
bool MujocoViewer::paused() const {
  if (impl_->exited)
    return false;
  mujoco::MutexLock lock(impl_->sim->mtx);
  return !impl_->sim->run;
}
std::unique_lock<std::recursive_mutex> MujocoViewer::lock() {
  return std::unique_lock<std::recursive_mutex>(impl_->sim->mtx);
}
bool MujocoViewer::closed() const { return impl_->exited || impl_->sim->exitrequest.load(); }
} // namespace boltrt
