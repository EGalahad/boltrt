// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/loop.hpp"
#include "boltrt/tasks/track/register.hpp"
#include <chrono>
#include <iostream>
using namespace boltrt;
namespace {
class FrozenBackend final : public RobotBackend {
public:
  int reads = 0, commands = 0, close_after = 100;
  bool closed = false;
  void start() override {}
  std::optional<RobotState> read_state() override {
    if (++reads > close_after)
      return std::nullopt;
    RobotState s;
    s.q = {0};
    s.dq = {0};
    s.torque = {0};
    // Simulates a paused backend followed by a native simulation reset.
    s.time = reads < 6 ? 2 : 0;
    return s;
  }
  void submit(const JointCommand &) override { ++commands; }
  void stop() noexcept override { closed = true; }
};
class Motion final : public MotionSource {
public:
  int64_t frame = 0;
  Names joints{"joint"}, bodies{"pelvis"};
  void reset() override { frame = 0; }
  MotionWindow sample(bool advance, bool paused) override {
    if (advance && !paused)
      ++frame;
    MotionWindow w;
    w.frame = frame;
    w.last_frame = 100;
    w.offsets = {0};
    w.joints = joints;
    w.bodies = bodies;
    w.frames.resize(1);
    w.frames[0].q = {float(frame)};
    w.frames[0].dq = {0};
    return w;
  }
  const Names &joint_names() const override { return joints; }
  const Names &body_names() const override { return bodies; }
};
class Frame final : public Observation {
  Tensor compute(const ObservationContext &c) const override {
    return Tensor::floats({c.task ? float(tracking_motion(c).frame) : -1.f});
  }
};
class Model final : public Inference {
public:
  int resets = 0, initial = 0;
  Vec frames;
  void reset() override { ++resets; }
  void validate(const TensorMap &) const override {}
  TensorMap run(const TensorMap &inputs) override {
    initial += std::get<std::vector<uint8_t>>(inputs.at("is_init").data)[0];
    frames.push_back(std::get<Vec>(inputs.at("obs").data)[0]);
    return {{"action", Tensor::floats({0})}};
  }
};
} // namespace
int main() {
  try {
    RobotSpec r;
    r.joints = {"joint"};
    r.initial_q = {0};
    PolicyConfig p;
    p.actions = r.joints;
    p.default_q = {0};
    p.kp = {1};
    p.kd = {1};
    p.scale = {1};
    p.yaml = YAML::Load("observation: {obs: {frame: {}}}");
    ObservationRegistry reg;
    reg.add("frame", [](const auto &, const auto &) { return std::make_unique<Frame>(); });
    LoopOptions o;
    o.keyboard = false;
    o.realtime = false;
    o.start_policy = true;
    o.steps = 10;
    FrozenBackend b;
    Motion motion;
    Model model;
    TrackingLoop(r, p, b, motion, model, reg, o).run();
    if (model.frames != Vec{1, 2, 3, 4, 5, 6, 7, 8, 9, 10} || model.resets != 1 ||
        model.initial != 1 || b.commands != 10 || !b.closed)
      throw std::runtime_error("backend clock changed policy/motion execution or history");
    FrozenBackend finite;
    finite.close_after = 3;
    Model second;
    TrackingLoop(r, p, finite, motion, second, reg, o).run();
    if (!finite.closed || finite.commands != 2)
      throw std::runtime_error("backend close did not cleanly end policy session");
    // A backend snapshot must not be responsible for pacing the policy clock.
    o.realtime = true;
    o.rate = 100;
    o.steps = 5;
    FrozenBackend paced;
    Model third;
    const auto start = std::chrono::steady_clock::now();
    TaskRegistry tasks;
    register_tracking_task(tasks);
    if (tasks.names() != Names{"base", "tracking"})
      throw std::runtime_error("task registrations missing");
    const std::map<std::string, std::string> no_arguments;
    auto base = tasks.create("base", {r, p, paced, third, reg, o, no_arguments});
    base->run();
    if (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(35))
      throw std::runtime_error("policy loop lost its independent clock");
    if (third.frames != Vec(5, -1.f))
      throw std::runtime_error("base policy loop unexpectedly needs motion");
    std::cout
        << "policy/motion independent of frozen/reset backend; feedback close terminates cleanly\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
