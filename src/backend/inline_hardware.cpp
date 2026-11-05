// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/hardware.hpp"
#include <atomic>
#include <mutex>
#include <thread>
namespace boltrt {
namespace {
using Clock = std::chrono::steady_clock;
class InlineHardware final : public RobotBackend {
public:
  InlineHardware(std::unique_ptr<HardwareDriver> driver, const RobotSpec &r,
                 const BackendConfig &cfg, double rate)
      : driver_(std::move(driver)), n_(r.joints.size()), period_(1 / rate) {
    control_rate_ = cfg.yaml["control_rate_hz"].as<double>();
    timeout_ = cfg.yaml["command_timeout_s"].as<double>();
    if (!std::isfinite(control_rate_) || control_rate_ <= 0 || !std::isfinite(timeout_) ||
        timeout_ < period_ * 2)
      throw std::runtime_error("invalid hardware timing/watchdog");
  }
  ~InlineHardware() override { stop(); }
  void start() override {
    if (started_)
      throw std::runtime_error("hardware backend sessions are single-use");
    started_ = true;
    try {
      driver_->start();
      auto s = driver_->read();
      latest_ = driver_->prime(s);
      validate_command(latest_, n_);
      driver_->write(latest_);
      last_command_ = Clock::now();
      origin_ = last_command_;
      tick_ = 0;
      running_ = true;
      thread_ = std::thread([this] { writer(); });
    } catch (...) {
      driver_->stop();
      throw;
    }
  }
  std::optional<RobotState> read_state() override {
    if (!running_)
      throw std::runtime_error("hardware backend stopped");
    {
      std::lock_guard lock(mutex_);
      if (error_)
        std::rethrow_exception(error_);
    }
    auto s = driver_->read();
    finite(s.q, "hardware q");
    finite(s.dq, "hardware dq");
    if (s.q.size() != n_ || s.dq.size() != n_)
      throw std::runtime_error("hardware state shape mismatch");
    s.time = std::chrono::duration<double>(Clock::now() - origin_).count();
    s.tick = tick_++;
    return s;
  }
  void submit(const JointCommand &cmd) override {
    validate_command(cmd, n_);
    std::lock_guard lock(mutex_);
    if (error_)
      std::rethrow_exception(error_);
    latest_ = cmd;
    last_command_ = Clock::now();
  }
  void stop() noexcept override {
    running_ = false;
    if (thread_.joinable())
      thread_.join();
    if (started_stop_ || !started_)
      return;
    started_stop_ = true;
    driver_->stop();
  }

private:
  void writer() {
    auto next = Clock::now();
    while (running_) {
      try {
        JointCommand cmd;
        {
          std::lock_guard lock(mutex_);
          cmd = latest_;
          if (std::chrono::duration<double>(Clock::now() - last_command_).count() > timeout_)
            throw std::runtime_error("hardware command watchdog expired");
        }
        driver_->write(cmd);
      } catch (...) {
        driver_->fail_safe();
        std::lock_guard lock(mutex_);
        error_ = std::current_exception();
        running_ = false;
        break;
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(1 / control_rate_));
      if (next < Clock::now())
        next = Clock::now();
      std::this_thread::sleep_until(next);
    }
  }
  std::unique_ptr<HardwareDriver> driver_;
  size_t n_;
  double period_, control_rate_, timeout_;
  std::atomic<bool> running_{false};
  bool started_stop_ = false, started_ = false;
  std::thread thread_;
  std::mutex mutex_;
  JointCommand latest_;
  Clock::time_point last_command_, origin_;
  uint64_t tick_ = 0;
  std::exception_ptr error_;
};
} // namespace
std::unique_ptr<RobotBackend> inline_hardware(std::unique_ptr<HardwareDriver> d, const RobotSpec &r,
                                              const BackendConfig &cfg, double rate) {
  return std::make_unique<InlineHardware>(std::move(d), r, cfg, rate);
}
} // namespace boltrt
