// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/hardware.hpp"
#include <algorithm>
#include <fcntl.h>
#include <mutex>
#include <robot_interface.hpp>
#include <set>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
namespace boltrt {
namespace {
// This adapter targets the calibrated rp1_deploy RobotInterface API.
// Motor ordering and the coupled ankle solver remain owned by that SDK.
class RP1Driver final : public HardwareDriver {
public:
  RP1Driver(const RobotSpec &r, const BackendConfig &c) : robot_(r), config_(c.yaml) {
    require_keys(config_,
                 {"type", "sdk_config", "joint_names", "control_rate_hz", "command_timeout_s"},
                 "rp1 backend");
    sdk_names_ = config_["joint_names"].as<Names>();
    if (sdk_names_.size() != r.joints.size() ||
        std::set<std::string>(sdk_names_.begin(), sdk_names_.end()).size() != sdk_names_.size())
      throw std::runtime_error("invalid RP1 SDK joint layout");
    for (auto &name : r.joints)
      ids_.push_back(index_of(sdk_names_, name));
    path_ = resolve_asset(config_["sdk_config"].as<std::string>(), c.path.parent_path());
    auto sdk = YAML::LoadFile(path_.string());
    auto motors = sdk["motors"];
    auto cfg = sdk["robot"];
    auto motor_ids = motors["motor_id"].as<std::vector<int>>();
    auto mapping = cfg["urdf2motor"].as<std::vector<int>>();
    auto kp = cfg["kp"].as<Vec>(), kd = cfg["kd"].as<Vec>();
    auto n = r.joints.size();
    if (motor_ids.size() != n || mapping.size() != n || kp.size() != n || kd.size() != n)
      throw std::runtime_error("RP1 SDK dimension mismatch");
    std::set<int> seen;
    sdk_kp_.resize(n);
    sdk_kd_.resize(n);
    for (size_t j = 0; j < n; ++j) {
      auto i = mapping[j];
      if (i < 0 || size_t(i) >= n || !seen.insert(i).second)
        throw std::runtime_error("RP1 urdf2motor must be a permutation");
      sdk_kp_[j] = kp[size_t(i)];
      sdk_kd_[j] = kd[size_t(i)];
    }
    finite(sdk_kp_, "RP1 SDK kp");
    finite(sdk_kd_, "RP1 SDK kd");
    auto close = cfg["close_chain_motor_idx"].as<std::vector<int>>();
    if (close.size() != 4 || std::set<int>(close.begin(), close.end()).size() != 4)
      throw std::runtime_error("RP1 expects four coupled ankle motors");
    for (int m : close) {
      auto it = std::find(mapping.begin(), mapping.end(), m);
      if (it == mapping.end())
        throw std::runtime_error("RP1 ankle missing from joint mapping");
      auto id = size_t(it - mapping.begin());
      if (sdk_names_[id].find("_ankle_") == std::string::npos || sdk_kp_[id] <= 0 ||
          sdk_kd_[id] < 0)
        throw std::runtime_error("RP1 coupled ankle semantic/gain mismatch");
      ankles_.push_back(id);
    }
    for (size_t i = 0; i < n; ++i)
      if (sdk_kp_[i] < 0 || sdk_kd_[i] < 0)
        throw std::runtime_error("negative RP1 SDK gains");
  }
  ~RP1Driver() override { stop(); }
  void start() override {
    const auto lock_path = (std::filesystem::temp_directory_path() / "boltrt-rp1.lock").string();
    lock_ = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0600);
    if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("RP1 runtime already active or lock unavailable");
    sdk_ = std::make_unique<RobotInterface>(path_.string(),
                                            1.0 / config_["control_rate_hz"].as<double>());
    sdk_->init_motors();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    sdk_->refresh_joints();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RobotState read() override {
    std::lock_guard io_guard(io_mutex_);
    sdk_->read_imu();
    auto q = sdk_->get_joint_q(), dq = sdk_->get_joint_vel(), tau = sdk_->get_joint_tau(),
         quat = sdk_->get_quat(), omega = sdk_->get_ang_vel();
    if (q.size() != ids_.size() || dq.size() != q.size() || tau.size() != q.size() ||
        quat.size() != 4 || omega.size() != 3)
      throw std::runtime_error("RP1 feedback shape mismatch");
    finite(q, "RP1 q");
    finite(dq, "RP1 dq");
    finite(tau, "RP1 torque");
    finite(quat, "RP1 IMU quaternion");
    finite(omega, "RP1 IMU omega");
    RobotState s;
    for (auto i : ids_) {
      s.q.push_back(q[i]);
      s.dq.push_back(dq[i]);
      s.torque.push_back(tau[i]);
    }
    s.orientation = Quat(quat[0], quat[1], quat[2], quat[3]);
    if (s.orientation.norm() < 1e-8)
      throw std::runtime_error("zero RP1 quaternion");
    s.orientation.normalize();
    s.angular_velocity = V3(omega[0], omega[1], omega[2]);
    {
      std::lock_guard guard(mutex_);
      last_ = s;
    }
    return s;
  }
  JointCommand prime(const RobotState &s) override {
    JointCommand c{s.q, Vec(ids_.size()), Vec(ids_.size()), Vec(ids_.size()), Vec(ids_.size())};
    for (size_t j = 0; j < ids_.size(); ++j) {
      auto id = ids_[j];
      bool ankle = std::find(ankles_.begin(), ankles_.end(), id) != ankles_.end();
      c.kp[j] = ankle ? sdk_kp_[id] : std::min(sdk_kp_[id], 5.0f);
      c.kd[j] = ankle ? sdk_kd_[id] : std::min(sdk_kd_[id], 0.5f);
    }
    return c;
  }
  void write(const JointCommand &c) override {
    std::lock_guard io_guard(io_mutex_);
    Vec q(ids_.size()), dq(q.size()), kp(q.size()), kd(q.size()), tau(q.size());
    for (size_t j = 0; j < ids_.size(); ++j) {
      auto id = ids_[j];
      q[id] = c.q[j];
      dq[id] = c.dq[j];
      kp[id] = c.kp[j];
      kd[id] = c.kd[j];
      tau[id] = c.tau[j];
    }
    sdk_->apply_action(q, dq, kp, kd, tau);
  }
  void fail_safe() noexcept override {
    if (!sdk_)
      return;
    try {
      RobotState s;
      {
        std::lock_guard guard(mutex_);
        s = last_;
      }
      if (s.q.size() == ids_.size())
        write(prime(s));
    } catch (...) {
    }
  }
  void stop() noexcept override {
    if (sdk_) {
      fail_safe();
      sdk_.reset(); // Official SDK owns motor deinitialization and CAN
                    // resources.
    }
    if (lock_ >= 0) {
      flock(lock_, LOCK_UN);
      ::close(lock_);
      lock_ = -1;
    }
  }

private:
  RobotSpec robot_;
  YAML::Node config_;
  Names sdk_names_;
  std::filesystem::path path_;
  std::vector<size_t> ids_, ankles_;
  Vec sdk_kp_, sdk_kd_;
  std::unique_ptr<RobotInterface> sdk_;
  std::mutex mutex_, io_mutex_;
  RobotState last_;
  int lock_ = -1;
};
} // namespace
void register_rp1_backend(BackendRegistry &r) {
  r.add("rp1", [](auto &robot, auto &cfg, double rate) {
    return inline_hardware(std::make_unique<RP1Driver>(robot, cfg), robot, cfg, rate);
  });
}
} // namespace boltrt
