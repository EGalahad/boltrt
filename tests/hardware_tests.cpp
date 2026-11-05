// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/hardware.hpp"
#include <atomic>
#include <iostream>
#include <thread>
using namespace boltrt;
namespace {
struct Probe {
  std::atomic<int> starts{0}, writes{0}, safe{0}, stops{0};
};
class Driver final : public HardwareDriver {
public:
  explicit Driver(Probe &p) : p_(p) {}
  void start() override { ++p_.starts; }
  RobotState read() override {
    RobotState s;
    s.q = {0.3f};
    s.dq = {0};
    return s;
  }
  JointCommand prime(const RobotState &s) override { return {s.q, {0}, {0}, {0}, {1}}; }
  void write(const JointCommand &c) override {
    validate_command(c, 1);
    ++p_.writes;
  }
  void fail_safe() noexcept override { ++p_.safe; }
  void stop() noexcept override { ++p_.stops; }

private:
  Probe &p_;
};
void check(bool ok, const char *s) {
  if (!ok)
    throw std::runtime_error(s);
}
} // namespace
int main() {
  try {
    RobotSpec r;
    r.joints = {"joint"};
    BackendConfig cfg;
    cfg.yaml = YAML::Load("control_rate_hz: 500\ncommand_timeout_s: 0.04");
    Probe p;
    auto backend = inline_hardware(std::make_unique<Driver>(p), r, cfg, 100);
    check(p.starts == 0 && p.writes == 0, "construction touched hardware");
    backend->start();
    check(p.starts == 1, "driver not started exactly once");
    auto state = backend->read_state();
    check(state->q == Vec{0.3f}, "lost initial feedback");
    backend->submit({{0.4f}, {0}, {0}, {4}, {1}});
    for (int i = 0; i < 100 && p.safe == 0; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    check(p.safe == 1, "command watchdog did not invoke driver fail-safe");
    bool rejected = false;
    try {
      backend->read_state();
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "watchdog failure not propagated");
    backend->stop();
    backend->stop();
    check(p.stops == 1, "stop not idempotent");
    check(p.writes > 1, "no high-frequency command hold");
    std::cout << "hardware scheduling contracts passed (mock only)\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
