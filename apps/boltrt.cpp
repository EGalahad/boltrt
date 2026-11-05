// SPDX-License-Identifier: Apache-2.0
#include "boltrt/runtime/task.hpp"
#include "boltrt/tasks/track/register.hpp"
#include <algorithm>
#include <iostream>
#include <set>
namespace s = boltrt;
namespace fs = std::filesystem;
int main(int argc, char **argv) {
  try {
    s::TaskRegistry tasks;
    s::register_tracking_task(tasks);
    std::map<std::string, std::string> args;
    const std::set<std::string> flags{"help",         "validate",          "start",
                                      "no-keyboard",  "run-once",          "headless",
                                      "print-config", "list-observations", "list-backends",
                                      "list-robots",  "list-tasks"};
    std::set<std::string> values{
        "robot",  "policy",      "backend",    "task",        "rate",        "steps",  "record",
        "events", "ort-threads", "init-steps", "config-root", "robot-model", "timings"};
    for (const auto &name : tasks.option_names())
      values.insert(name);
    for (int i = 1; i < argc; ++i) {
      std::string key = argv[i];
      if (key.rfind("--", 0) != 0)
        throw std::runtime_error("expected --option: " + key);
      key = key.substr(2);
      if (args.count(key))
        throw std::runtime_error("duplicate option: " + key);
      if (flags.count(key))
        args[key] = "true";
      else if (values.count(key)) {
        if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0)
          throw std::runtime_error("missing value: " + key);
        args[key] = argv[++i];
      } else
        throw std::runtime_error("unknown option: " + key);
    }
    if (args.count("help")) {
      std::cout << "boltrt [--task <name>] --robot <name|yaml> --policy <yaml> "
                   "--backend <name|yaml>\n"
                   "  --task tracking        Enable motion tracking; default: base\n"
                   "  --motion <npz|tcp://endpoint>   Tracking reference (optional)\n"
                   "  --validate            Check assets, observation and ONNX "
                   "contracts without starting devices\n"
                   "  --start               Enter policy and play motion (default: "
                   "hold, paused)\n"
                   "  --rate 50 --steps N   Policy frequency and bounded ticks (0: "
                   "until stopped)\n"
                   "  --run-once --record <jsonl> --events <json> --no-keyboard\n"
                   "  --ort-threads 1 --init-steps 500 --config-root <directory>\n"
                   "  --robot-model <xml|hf://asset>\n"
                   "  --timings <csv>       Save per-tick policy and inference latency\n"
                   "  --headless            Disable MuJoCo window and realtime "
                   "pacing\n"
                   "  --print-config --list-robots --list-observations "
                   "--list-backends --list-tasks\n"
                   "Terminal motion: Space=play/pause reference motion\n"
                   "Terminal policy: I=init, O=hold, ]=policy + reset/paused motion, Q=quit\n"
                   "MuJoCo window: native keys (Space=simulation pause, Backspace=reset)\n";
      return 0;
    }
    const auto task_name = args.count("task") ? args.at("task") : "base";
    if (args.count("list-tasks")) {
      for (const auto &name : tasks.names())
        std::cout << name << '\n';
      return 0;
    }
    if (args.count("list-robots")) {
      auto root = args.count("config-root") ? fs::path(args.at("config-root"))
                                            : fs::path(BOLTRT_CONFIG_DIR);
      s::Names names;
      for (const auto &entry : fs::directory_iterator(root / "robots"))
        if (entry.path().extension() == ".yaml")
          names.push_back(entry.path().stem().string());
      std::sort(names.begin(), names.end());
      for (const auto &name : names)
        std::cout << name << '\n';
      return 0;
    }
    s::ObservationRegistry observations;
    tasks.prepare_observations(task_name, observations);
    s::BackendRegistry backends;
    s::register_mujoco_backend(backends);
    s::register_replay_backend(backends);
#ifdef BOLTRT_G1
    s::register_g1_backend(backends);
#endif
#ifdef BOLTRT_RP1
    s::register_rp1_backend(backends);
#endif
    if (args.count("list-observations")) {
      for (auto &n : observations.names())
        std::cout << n << '\n';
      return 0;
    }
    if (args.count("list-backends")) {
      for (auto &n : backends.names())
        std::cout << n << '\n';
      return 0;
    }
    for (auto key : {"robot", "policy", "backend"})
      if (!args.count(key))
        throw std::runtime_error("required option: --" + std::string(key));
    auto config_root =
        args.count("config-root") ? fs::path(args.at("config-root")) : fs::path(BOLTRT_CONFIG_DIR);
    auto component = [&](const std::string &type, const std::string &value) {
      if (fs::exists(value) || value.find('/') != std::string::npos ||
          fs::path(value).extension() == ".yaml")
        return fs::absolute(value);
      auto path = config_root / type / (value + ".yaml");
      if (!fs::is_regular_file(path))
        throw std::runtime_error("config unavailable: " + path.string());
      return path;
    };
    auto robot = s::load_robot(component("robots", args.at("robot")));
    if (args.count("robot-model"))
      robot.model = s::resolve_asset(args.at("robot-model"), fs::current_path()).string();
    auto policy = s::load_policy(component("policies", args.at("policy")), robot);
    auto cfg = s::load_backend(component("backends", args.at("backend")));
    if (args.count("headless")) {
      if (cfg.type != "mujoco")
        throw std::runtime_error("--headless requires the mujoco backend");
      cfg.yaml["viewer"] = false;
      cfg.realtime = false;
    }
    s::LoopOptions opts;
    opts.realtime = cfg.type != "mujoco" || cfg.realtime ||
                    (!cfg.yaml["viewer"] || cfg.yaml["viewer"].as<bool>());
    if (cfg.type == "replay")
      opts.realtime = false;
    if (args.count("rate"))
      opts.rate = std::stod(args.at("rate"));
    if (!std::isfinite(opts.rate) || opts.rate <= 0 || opts.rate > 10000)
      throw std::runtime_error("invalid rate");
    if (args.count("steps")) {
      if (args.at("steps").front() == '-')
        throw std::runtime_error("negative steps");
      opts.steps = std::stoull(args.at("steps"));
    }
    if (args.count("init-steps"))
      opts.init_steps = std::stoi(args.at("init-steps"));
    opts.start_policy = args.count("start");
    opts.keyboard = !args.count("no-keyboard");
    opts.task_name = task_name;
    opts.run_once = args.count("run-once");
    if (args.count("record"))
      opts.record = fs::absolute(args.at("record"));
    if (args.count("events"))
      opts.events = fs::absolute(args.at("events"));
    if (args.count("timings"))
      opts.timings = fs::absolute(args.at("timings"));
    auto model = s::onnx_inference(
        policy, args.count("ort-threads") ? std::stoi(args.at("ort-threads")) : 1);
    auto backend = backends.create(robot, cfg, opts.rate);
    auto loop =
        tasks.create(task_name, {robot, policy, *backend, *model, observations, opts, args});
    if (args.count("print-config")) {
      YAML::Node resolved;
      resolved["robot"] = robot.name;
      resolved["robot_model"] = robot.model;
      resolved["policy"] = policy.path.string();
      resolved["model"] = policy.model.string();
      resolved["backend"] = cfg.yaml;
      resolved["task"] = task_name;
      resolved["rate"] = opts.rate;
      resolved["steps"] = opts.steps;
      std::cout << resolved << '\n';
    }
    if (args.count("validate")) {
      loop->validate();
      std::cout << "validated: robot=" << robot.name << " backend=" << cfg.type
                << " actions=" << policy.actions.size() << "; no device started\n";
      return 0;
    }
    loop->run();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "boltrt: " << e.what() << '\n';
    return 1;
  }
}
