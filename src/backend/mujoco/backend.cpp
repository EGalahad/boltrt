// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/backend.hpp"
#include "boltrt/utils/mujoco/model.hpp"
#include "scene.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#ifdef BOLTRT_VIEWER
#include "viewer.hpp"
#endif
namespace boltrt {
namespace {
class MujocoBackend final : public RobotBackend {
  using Clock = std::chrono::steady_clock;

public:
  MujocoBackend(const RobotSpec &r, const BackendConfig &c, double rate)
      : spec_(r), model_(r,
                         [&c](mjSpec *spec) {
                           if (!c.yaml["floor"] || c.yaml["floor"].as<bool>())
                             configure_studio_scene(spec);
                         }),
        realtime_(c.realtime) {
    require_keys(c.yaml, {"type", "physics_dt", "realtime", "floor", "viewer"}, "mujoco backend");
    show_viewer_ = c.yaml["viewer"] ? c.yaml["viewer"].as<bool>() : true;
    realtime_ = realtime_ || show_viewer_;
    if (!realtime_) {
      const double ratio = 1 / rate / c.dt;
      if (std::abs(ratio - std::round(ratio)) > 1e-8 || ratio < 1)
        throw std::runtime_error("offline policy period must be integer multiple of physics_dt");
      steps_ = int(std::round(ratio));
    }
    model_.model->opt.timestep = c.dt;
    model_.model->opt.disableflags |= mjDSBL_ACTUATION;
  }
  ~MujocoBackend() override { stop(); }
  void start() override {
    if (started_)
      throw std::runtime_error("MuJoCo backend sessions are single-use");
    started_ = true;
    mj_resetData(model_.model.get(), model_.data.get());
    model_.set_pose(spec_.initial_q, spec_.initial_position, Quat::Identity());
    command_ = {spec_.initial_q, Vec(spec_.joints.size()), Vec(spec_.joints.size()),
                Vec(spec_.joints.size()), Vec(spec_.joints.size())};
    tick_ = 0;
    pending_ = false;
    if (show_viewer_) {
#ifdef BOLTRT_VIEWER
      viewer_ = std::make_unique<MujocoViewer>(model_);
#else
      throw std::runtime_error("viewer was disabled at build time; use --headless "
                               "or build with BOLTRT_VIEWER=ON");
#endif
    }
    running_ = true;
    if (realtime_)
      worker_ = std::thread([this] { simulate(); });
  }
  std::optional<RobotState> read_state() override {
    auto lock = lock_model();
    if (error_)
      std::rethrow_exception(error_);
    if (!running_ || viewer_closed())
      return std::nullopt;
    // Offline evaluation uses deterministic command/state exchange. Online
    // physics has its own clock/thread and read_state is only a snapshot.
    if (!realtime_ && pending_) {
      for (int i = 0; i < steps_; ++i)
        physics_step();
      pending_ = false;
    }
    mj_forward(model_.model.get(), model_.data.get());
    RobotState state;
    state.tick = tick_;
    state.time = model_.data->time;
    for (size_t j = 0; j < spec_.joints.size(); ++j) {
      state.q.push_back(float(model_.data->qpos[model_.qadr[j]]));
      state.dq.push_back(float(model_.data->qvel[model_.vadr[j]]));
      state.torque.push_back(float(model_.data->qfrc_applied[model_.vadr[j]]));
    }
    finite(state.q, "sim state");
    finite(state.dq, "sim velocity");
    state.root_position = Eigen::Map<V3>(model_.data->qpos);
    state.orientation =
        Quat(model_.data->qpos[3], model_.data->qpos[4], model_.data->qpos[5], model_.data->qpos[6])
            .normalized();
    state.root_velocity = Eigen::Map<V3>(model_.data->qvel);
    state.angular_velocity = Eigen::Map<V3>(model_.data->qvel + 3);
    state.root_position_valid = true;
    state.root_velocity_valid = true;
    if (!state.root_position.allFinite() || !state.root_velocity.allFinite() ||
        !state.angular_velocity.allFinite() || !state.orientation.coeffs().allFinite())
      throw std::runtime_error("nonfinite floating-base simulation state");
    return state;
  }
  void submit(const JointCommand &command) override {
    validate_command(command, spec_.joints.size());
    auto lock = lock_model();
    if (error_)
      std::rethrow_exception(error_);
    command_ = command;
    pending_ = true;
  }
  void stop() noexcept override {
    running_ = false;
    if (worker_.joinable())
      worker_.join();
#ifdef BOLTRT_VIEWER
    viewer_.reset();
#endif
  }

private:
  std::unique_lock<std::recursive_mutex> lock_model() {
#ifdef BOLTRT_VIEWER
    if (viewer_)
      return viewer_->lock();
#endif
    return std::unique_lock<std::recursive_mutex>(mutex_);
  }
  bool viewer_closed() const {
#ifdef BOLTRT_VIEWER
    return viewer_ && viewer_->closed();
#else
    return false;
#endif
  }
  void physics_step() {
    for (size_t j = 0; j < spec_.joints.size(); ++j) {
      const auto q = model_.data->qpos[model_.qadr[j]], dq = model_.data->qvel[model_.vadr[j]];
      const double tau = command_.kp[j] * (command_.q[j] - q) +
                         command_.kd[j] * (command_.dq[j] - dq) + command_.tau[j];
      model_.data->qfrc_applied[model_.vadr[j]] =
          std::clamp(tau, -double(spec_.effort[j]), double(spec_.effort[j]));
    }
    mj_step(model_.model.get(), model_.data.get());
    ++tick_;
  }
  void simulate() {
    auto next = Clock::now();
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(model_.model->opt.timestep));
    while (running_) {
      try {
        auto lock = lock_model();
        if (viewer_closed())
          break;
        bool advance = true;
#ifdef BOLTRT_VIEWER
        if (viewer_) {
          // Native reset, pause, stepping and perturbations stay inside MuJoCo.
          viewer_->render();
          advance = !viewer_->paused();
        }
#endif
        if (advance)
          physics_step();
        else
          mj_forward(model_.model.get(), model_.data.get());
      } catch (...) {
        auto lock = lock_model();
        error_ = std::current_exception();
        break;
      }
      next = std::max(next + period, Clock::now());
      std::this_thread::sleep_until(next);
    }
    running_ = false;
  }
  RobotSpec spec_;
  RobotModel model_;
  std::recursive_mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  bool show_viewer_ = true, realtime_, started_ = false, pending_ = false;
#ifdef BOLTRT_VIEWER
  std::unique_ptr<MujocoViewer> viewer_;
#endif
  int steps_ = 0;
  uint64_t tick_ = 0;
  JointCommand command_;
  std::exception_ptr error_;
};
} // namespace
void register_mujoco_backend(BackendRegistry &r) {
  r.add("mujoco", [](auto &robot, auto &cfg, double rate) {
    return std::make_unique<MujocoBackend>(robot, cfg, rate);
  });
}
} // namespace boltrt
