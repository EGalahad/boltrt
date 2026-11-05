// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/backend.hpp"
#include <fstream>
#include <optional>
#include <nlohmann/json.hpp>
namespace boltrt {
namespace {
class ReplayBackend final : public RobotBackend {
public:
  ReplayBackend(const RobotSpec &r, const BackendConfig &cfg)
      : robot_(r),
        path_(resolve_asset(cfg.yaml["path"].as<std::string>(), cfg.path.parent_path())) {
    require_keys(cfg.yaml, {"type", "path"}, "replay backend");
  }
  void start() override {
    input_.open(path_);
    if (!input_)
      throw std::runtime_error("state replay unavailable");
  }
  // Like the MuJoCo backend, feedback advances only after a command is submitted:
  // repeated reads return the same recorded state, so tick i sees recorded line i.
  std::optional<RobotState> read_state() override {
    if (current_ && !advance_)
      return current_;
    advance_ = false;
    std::string line;
    if (!std::getline(input_, line))
      return current_ = std::nullopt;
    current_ = parse(line);
    return current_;
  }
  void submit(const JointCommand &c) override {
    validate_command(c, robot_.joints.size());
    advance_ = true;
  }
  void stop() noexcept override {
    if (input_.is_open())
      input_.close();
  }

private:
  RobotState parse(const std::string &line) const {
    auto j = nlohmann::json::parse(line);
    RobotState s;
    s.q = j.at("q").get<Vec>();
    s.dq = j.at("dq").get<Vec>();
    s.torque = Vec(robot_.joints.size());
    if (s.q.size() != robot_.joints.size() || s.dq.size() != s.q.size())
      throw std::runtime_error("replay state size mismatch");
    finite(s.q, "replay q");
    finite(s.dq, "replay dq");
    auto q = j.at("orientation").get<Vec>(), v = j.at("angular_velocity").get<Vec>();
    if (q.size() != 4 || v.size() != 3)
      throw std::runtime_error("replay IMU shape mismatch");
    finite(q, "replay orientation");
    finite(v, "replay omega");
    s.orientation = Quat(q[0], q[1], q[2], q[3]);
    if (s.orientation.norm() < 1e-8)
      throw std::runtime_error("zero replay quaternion");
    s.orientation.normalize();
    s.angular_velocity = V3(v[0], v[1], v[2]);
    if (j.contains("root_position")) {
      const auto position = j.at("root_position").get<Vec>();
      if (position.size() != 3)
        throw std::runtime_error("replay root position shape mismatch");
      finite(position, "replay root position");
      s.root_position = V3(position[0], position[1], position[2]);
      s.root_position_valid = j.value("root_position_valid", true);
    }
    s.tick = j.at("tick").get<uint64_t>();
    s.time = j.at("time").get<double>();
    return s;
  }

  RobotSpec robot_;
  std::filesystem::path path_;
  std::ifstream input_;
  std::optional<RobotState> current_;
  bool advance_ = false;
};
} // namespace
void register_replay_backend(BackendRegistry &r) {
  r.add("replay",
        [](auto &robot, auto &cfg, double) { return std::make_unique<ReplayBackend>(robot, cfg); });
}
} // namespace boltrt
