// SPDX-License-Identifier: Apache-2.0
#include "boltrt/runtime/policy_loop.hpp"
#include "terminal.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unistd.h>
namespace boltrt {
namespace {
using Json = nlohmann::json;
volatile std::sig_atomic_t stopped = 0;
void signal_handler(int) { stopped = 1; }
RobotState initial_state(const RobotSpec &r) {
  RobotState s;
  s.q = r.initial_q;
  s.dq = Vec(r.joints.size());
  s.torque = s.dq;
  s.root_position = r.initial_position;
  return s;
}
TensorMap special_inputs(TensorMap values, const Vec &action, bool reset) {
  values["action"] = Tensor::floats(action);
  Tensor t;
  t.shape = {1};
  t.data = std::vector<uint8_t>{uint8_t(reset)};
  values["is_init"] = std::move(t);
  return values;
}
Json tensor_json(const Tensor &t) {
  Json j;
  j["shape"] = t.shape;
  j["dtype"] = t.dtype() == DType::Float32 ? "float32"
               : t.dtype() == DType::Int64 ? "int64"
                                           : "bool";
  std::visit([&](auto &v) { j["data"] = v; }, t.data);
  return j;
}
class Recorder {
public:
  explicit Recorder(const std::filesystem::path &p) {
    if (p.empty())
      return;
    auto parent = p.parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
    output_.open(p, std::ios::out | std::ios::trunc);
    if (!output_)
      throw std::runtime_error("cannot create recording: " + p.string());
    active_ = true;
    worker_ = std::thread([this] { write(); });
  }
  ~Recorder() { close(); }
  void push(Json j) {
    if (!active_)
      return;
    std::lock_guard lock(mutex_);
    if (failed_)
      throw std::runtime_error("recording write failed");
    if (queue_.size() >= 256)
      throw std::runtime_error("recording queue full; refusing incomplete trace");
    queue_.push_back(std::move(j));
    cv_.notify_one();
  }
  void close() {
    if (!worker_.joinable())
      return;
    {
      std::lock_guard lock(mutex_);
      done_ = true;
    }
    cv_.notify_one();
    worker_.join();
  }
  void check() {
    std::lock_guard lock(mutex_);
    if (failed_)
      throw std::runtime_error("recording write failed");
  }
  bool active() const { return active_; }

private:
  void write() {
    for (;;) {
      Json j;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return done_ || !queue_.empty(); });
        if (queue_.empty() && done_)
          break;
        j = std::move(queue_.front());
        queue_.pop_front();
      }
      output_ << j.dump() << '\n';
      if (!output_) {
        std::lock_guard lock(mutex_);
        failed_ = true;
      }
    }
    output_.flush();
    if (!output_) {
      std::lock_guard lock(mutex_);
      failed_ = true;
    }
  }
  std::ofstream output_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Json> queue_;
  bool active_ = false, done_ = false, failed_ = false;
};
std::map<uint64_t, std::vector<Event>> load_events(const std::filesystem::path &path) {
  std::map<uint64_t, std::vector<Event>> out;
  if (path.empty())
    return out;
  std::ifstream f(path);
  if (!f)
    throw std::runtime_error("events file unavailable");
  Json array;
  f >> array;
  std::map<std::string, Event> names{{"init", Event::Init},       {"hold", Event::Hold},
                                     {"policy", Event::Policy},   {"pause", Event::TogglePause},
                                     {"restart", Event::Restart}, {"stop", Event::Stop}};
  for (auto &j : array) {
    auto name = j.at("event").get<std::string>();
    if (!names.count(name))
      throw std::runtime_error("unknown event: " + name);
    out[j.at("tick").get<uint64_t>()].push_back(names.at(name));
  }
  return out;
}
} // namespace
void PolicyLoop::request_stop() { stopped = 1; }
PolicyLoop::PolicyLoop(const RobotSpec &r, const PolicyConfig &p, RobotBackend &b, Inference &i,
                       const ObservationRegistry &reg, LoopOptions o)
    : robot_(r), policy_(p), backend_(b), inference_(i), registry_(reg), options_(std::move(o)) {
  if (!std::isfinite(options_.rate) || options_.rate <= 0 || options_.init_steps <= 0)
    throw std::runtime_error("invalid loop timing");
}
void PolicyLoop::validate() {
  auto state = initial_state(robot_);
  prepare_task();
  Vec action(policy_.actions.size());
  TensorMap previous_output;
  ObservationContext ctx{robot_, policy_, state, action, &previous_output};
  bind_task_context(ctx);
  ObservationGraph graph(registry_, policy_.yaml["observation"], ctx);
  graph.reset(ctx);
  inference_.validate(special_inputs(graph.compute(ctx), action, true));
}
void PolicyLoop::run() {
  validate();
  stopped = 0;
  auto oldint = std::signal(SIGINT, signal_handler), oldterm = std::signal(SIGTERM, signal_handler);
  struct SignalGuard {
    decltype(oldint) i, t;
    ~SignalGuard() {
      std::signal(SIGINT, i);
      std::signal(SIGTERM, t);
    }
  } signal_guard{oldint, oldterm};
  auto events = load_events(options_.events);
  Recorder recorder(options_.record);
  struct StopGuard {
    RobotBackend &b;
    ~StopGuard() { b.stop(); }
  } guard{backend_};
  backend_.start();
  TerminalControl keyboard(options_.keyboard);
  auto feedback = backend_.read_state();
  if (!feedback)
    return;
  auto state = *feedback;
  start_task(options_.start_policy);
  Vec action(policy_.actions.size());
  TensorMap previous_output;
  ObservationContext ctx{robot_, policy_, state, action, &previous_output};
  bind_task_context(ctx);
  ObservationGraph graph(registry_, policy_.yaml["observation"], ctx);
  graph.reset(ctx);
  inference_.reset();
  Mode mode = Mode::Hold;
  int init_count = 0;
  bool model_reset = true;
  size_t ticks = 0, misses = 0;
  double max_ms = 0;
  std::vector<double> times, inference_times;
  if (options_.start_policy) {
    mode = Mode::Policy;
  }
  RuntimeStatus status;
  status.robot = robot_.name;
  status.policy = policy_.path.stem().string();
  status.policy_path = policy_.path.string();
  status.robot_model = robot_.model;
  status.task = options_.task_name;
  auto publish_status = [&] {
    if (!keyboard.interactive())
      return;
    status.mode = mode;
    update_task_status(status);
    status.backend_time = state.time;
    keyboard.render(status);
  };
  publish_status();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1 / options_.rate));
  auto next_tick = std::chrono::steady_clock::now();
  while (!stopped && (options_.steps == 0 || ticks < options_.steps)) {
    if (options_.realtime) {
      std::this_thread::sleep_until(next_tick);
      const auto now = std::chrono::steady_clock::now();
      next_tick = std::max(next_tick + period, now + period);
    }
    feedback = backend_.read_state();
    if (!feedback)
      break;
    state = *feedback;
    auto start = std::chrono::steady_clock::now();
    auto input = keyboard.poll();
    if (auto it = events.find(ticks); it != events.end()) {
      input.insert(input.end(), it->second.begin(), it->second.end());
      events.erase(it);
    }
    for (auto event : input) {
      if (event == Event::Stop) {
        stopped = 1;
        break;
      }
      if (event == Event::Init) {
        mode = Mode::Init;
        init_count = 0;
      } else if (event == Event::Hold)
        mode = Mode::Hold;
      else if (event == Event::Policy) {
        enter_policy_task();
        std::fill(action.begin(), action.end(), 0);
        graph.reset(ctx);
        inference_.reset();
        previous_output.clear();
        model_reset = true;
        mode = Mode::Policy;
      }
      const std::map<Event, std::string> labels{{Event::Init, "Initializing"},
                                                {Event::Hold, "Holding pose"},
                                                {Event::Policy, "Policy active"}};
      if (auto label = labels.find(event); label != labels.end())
        status.last_event = label->second;
      else if (!handle_task_event(event, status.last_event))
        status.last_event = "No task handler for this control";
      update_task_status(status);
      if (!keyboard.interactive())
        std::cout << status.last_event << '\n';
    }
    if (stopped)
      break;
    publish_status();
    update_task();
    TensorMap components;
    auto inputs = special_inputs(graph.compute(ctx, recorder.active() ? &components : nullptr),
                                 action, model_reset);
    const auto inference_start = std::chrono::steady_clock::now();
    auto output = inference_.run(inputs);
    const double inference_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - inference_start)
                                    .count();
    inference_times.push_back(inference_ms);
    model_reset = false;
    auto it = output.find("action");
    if (it == output.end() || it->second.dtype() != DType::Float32)
      throw std::runtime_error("model must emit float32 action");
    auto next_action = std::get<Vec>(it->second.data);
    if (next_action.size() != policy_.actions.size())
      throw std::runtime_error("model action count mismatch");
    finite(next_action, "action");
    for (auto &a : next_action)
      a = std::clamp(a, -policy_.action_clip, policy_.action_clip);
    previous_output = output;
    JointCommand command;
    command.q = policy_.default_q;
    command.dq = Vec(robot_.joints.size());
    command.tau = command.dq;
    command.kp = policy_.kp;
    command.kd = policy_.kd;
    for (size_t j = 0; j < next_action.size(); ++j) {
      auto id = index_of(robot_.joints, policy_.actions[j]);
      command.q[id] += next_action[j] * policy_.scale[j];
    }
    auto target = command.q;
    if (mode == Mode::Hold)
      command.q = state.q;
    else if (mode == Mode::Init) {
      float a = float(std::min(++init_count, options_.init_steps)) / float(options_.init_steps);
      for (size_t j = 0; j < command.q.size(); ++j)
        command.q[j] = (1 - a) * state.q[j] + a * policy_.default_q[j];
    }
    validate_command(command, robot_.joints.size());
    backend_.submit(command);
    double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    status.policy_ms = ms;
    publish_status();
    times.push_back(ms);
    max_ms = std::max(max_ms, ms);
    if (ms > 1000 / options_.rate)
      ++misses;
    if (recorder.active()) {
      Json j{{"tick", ticks},
             {"time", state.time},
             {"mode", mode == Mode::Policy ? "policy"
                      : mode == Mode::Init ? "init"
                                           : "hold"},
             {"q", state.q},
             {"dq", state.dq},
             {"joint_torque", state.torque},
             {"root_position_valid", state.root_position_valid},
             {"root_velocity_valid", state.root_velocity_valid},
             {"angular_velocity",
              {state.angular_velocity.x(), state.angular_velocity.y(), state.angular_velocity.z()}},
             {"orientation",
              {state.orientation.w(), state.orientation.x(), state.orientation.y(),
               state.orientation.z()}},
             {"root_position",
              {state.root_position.x(), state.root_position.y(), state.root_position.z()}},
             {"action", next_action},
             {"target", target},
             {"command",
              {{"q", command.q},
               {"dq", command.dq},
               {"tau", command.tau},
               {"kp", command.kp},
               {"kd", command.kd}}},
             {"policy_ms", ms}};
      if (ticks == 0)
        j["layout"] = {{"joint_names", robot_.joints},
                       {"policy", policy_.path.string()},
                       {"model", policy_.model.string()},
                       {"robot_model", robot_.model}};
      append_task_record(j, ticks == 0);
      for (auto &[name, t] : components)
        j["components"][name] = tensor_json(t);
      for (auto &[name, t] : inputs)
        j["inputs"][name] = tensor_json(t);
      recorder.push(std::move(j));
    }
    action = std::move(next_action);
    ++ticks;
    if (options_.run_once && task_finished())
      break;
  }
  backend_.stop();
  recorder.close();
  recorder.check();
  keyboard.close();
  if (!options_.timings.empty()) {
    const auto parent = options_.timings.parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
    std::ofstream csv(options_.timings);
    csv << "tick,policy_ms,inference_ms\n";
    for (size_t i = 0; i < times.size(); ++i)
      csv << i << ',' << times[i] << ',' << inference_times[i] << '\n';
    csv.close();
    if (!csv)
      throw std::runtime_error("timing output write failed");
  }
  std::sort(times.begin(), times.end());
  std::cout << "ticks=" << ticks
            << " policy_ms_p50=" << (times.empty() ? 0 : times[times.size() / 2])
            << " policy_ms_max=" << max_ms << " deadline_misses=" << misses << '\n';
}
} // namespace boltrt
