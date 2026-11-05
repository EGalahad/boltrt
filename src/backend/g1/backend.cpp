// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/hardware.hpp"
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
namespace boltrt {
namespace {
uint32_t crc(const void *bytes, size_t words) {
  auto p = static_cast<const uint8_t *>(bytes);
  uint32_t value = 0xffffffff;
  for (size_t i = 0; i < words; ++i) {
    uint32_t word;
    std::memcpy(&word, p + 4 * i, 4);
    for (uint32_t bit = 0x80000000; bit; bit >>= 1) {
      bool top = (value & 0x80000000) != 0;
      value <<= 1;
      if (top)
        value ^= 0x04c11db7;
      if (word & bit)
        value ^= 0x04c11db7;
    }
  }
  return value;
}
const Names sdk_names = {
    "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",      "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",  "right_elbow_joint",          "right_wrist_roll_joint",
    "right_wrist_pitch_joint",   "right_wrist_yaw_joint"};
class G1Driver final : public HardwareDriver {
public:
  G1Driver(const RobotSpec &r, const BackendConfig &cfg) : config_(cfg.yaml) {
    require_keys(config_,
                 {"type", "interface", "domain_id", "control_rate_hz", "command_timeout_s",
                  "state_timeout_s", "startup_timeout_s", "release_motion_mode"},
                 "g1 backend");
    if (r.joints.size() != sdk_names.size())
      throw std::runtime_error("G1 backend requires the complete SDK joint layout");
    for (auto &n : r.joints)
      ids_.push_back(index_of(sdk_names, n));
    state_timeout_ = config_["state_timeout_s"].as<double>();
    startup_timeout_ = config_["startup_timeout_s"].as<double>();
    if (state_timeout_ <= 0 || startup_timeout_ <= 0 || !std::isfinite(state_timeout_) ||
        !std::isfinite(startup_timeout_))
      throw std::runtime_error("invalid G1 timeout");
  }
  ~G1Driver() override { stop(); }
  void start() override {
    auto path = (std::filesystem::temp_directory_path() /
                 ("boltrt-g1-" + std::to_string(config_["domain_id"].as<int>()) + ".lock"))
                    .string();
    lock_ = ::open(path.c_str(), O_CREAT | O_RDWR, 0600);
    if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("G1 runtime is already active or lock unavailable");
    unitree::robot::ChannelFactory::Instance()->Init(config_["domain_id"].as<int>(),
                                                     config_["interface"].as<std::string>());
    // Use one native SDK2 DDS participant for RPC and low-level traffic.
    unitree::robot::b2::MotionSwitcherClient switcher;
    switcher.SetTimeout(float(startup_timeout_));
    switcher.Init();
    std::string form, name;
    for (int attempt = 0; attempt < 5; ++attempt) {
      if (switcher.CheckMode(form, name) != 0)
        throw std::runtime_error("G1 CheckMode failed");
      if (name.empty())
        break;
      if (!config_["release_motion_mode"].as<bool>())
        throw std::runtime_error("G1 high-level motion mode is active");
      if (switcher.ReleaseMode() != 0)
        throw std::runtime_error("G1 ReleaseMode failed");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (switcher.CheckMode(form, name) != 0 || !name.empty())
      throw std::runtime_error("G1 debug mode not confirmed");
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
        "rt/lowcmd");
    publisher_->InitChannel();
    subscriber_ =
        std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
            "rt/lowstate");
    subscriber_->InitChannel(
        [this](const void *p) {
          auto &msg = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(p);
          if (msg.crc() != crc(&msg, (sizeof(msg) >> 2) - 1))
            return;
          {
            std::lock_guard guard(mutex_);
            state_ = msg;
            received_ = std::chrono::steady_clock::now();
            ready_ = true;
          }
          cv_.notify_one();
        },
        1);
    std::unique_lock guard(mutex_);
    if (!cv_.wait_for(guard, std::chrono::duration<double>(startup_timeout_),
                      [this] { return ready_; }))
      throw std::runtime_error("G1 low-state startup timeout");
  }
  RobotState read() override {
    std::lock_guard guard(mutex_);
    if (!ready_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
            state_timeout_)
      throw std::runtime_error("G1 low-state stale");
    RobotState s;
    for (auto i : ids_) {
      auto &m = state_.motor_state()[i];
      s.q.push_back(m.q());
      s.dq.push_back(m.dq());
      s.torque.push_back(m.tau_est());
    }
    auto &q = state_.imu_state().quaternion();
    s.orientation = Quat(q[0], q[1], q[2], q[3]);
    if (s.orientation.norm() < 1e-8)
      throw std::runtime_error("invalid G1 quaternion");
    s.orientation.normalize();
    auto &v = state_.imu_state().gyroscope();
    s.angular_velocity = V3(v[0], v[1], v[2]);
    if (!s.orientation.coeffs().allFinite() || !s.angular_velocity.allFinite())
      throw std::runtime_error("nonfinite G1 IMU");
    return s;
  }
  JointCommand prime(const RobotState &s) override {
    return {s.q, Vec(ids_.size()), Vec(ids_.size()), Vec(ids_.size()), Vec(ids_.size(), 1)};
  }
  void write(const JointCommand &c) override {
    unitree_hg::msg::dds_::LowCmd_ msg{};
    {
      std::lock_guard guard(mutex_);
      if (!ready_ ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
              state_timeout_)
        throw std::runtime_error("G1 low-state watchdog expired");
      msg.mode_machine() = state_.mode_machine();
    }
    msg.mode_pr() = 0;
    for (size_t j = 0; j < ids_.size(); ++j) {
      auto &m = msg.motor_cmd()[ids_[j]];
      m.mode() = 1;
      m.q() = c.q[j];
      m.dq() = c.dq[j];
      m.tau() = c.tau[j];
      m.kp() = c.kp[j];
      m.kd() = c.kd[j];
    }
    msg.crc() = crc(&msg, (sizeof(msg) >> 2) - 1);
    if (!publisher_->Write(msg))
      throw std::runtime_error("G1 DDS command write failed");
  }
  void fail_safe() noexcept override {
    if (publisher_) {
      try {
        unitree_hg::msg::dds_::LowCmd_ msg{};
        {
          std::lock_guard guard(mutex_);
          msg.mode_machine() = state_.mode_machine();
        }
        msg.mode_pr() = 0;
        for (auto i : ids_) {
          msg.motor_cmd()[i].mode() = 1;
          msg.motor_cmd()[i].kp() = 0;
          msg.motor_cmd()[i].kd() = 1;
        }
        msg.crc() = crc(&msg, (sizeof(msg) >> 2) - 1);
        publisher_->Write(msg);
      } catch (...) {
      }
    }
  }
  void stop() noexcept override {
    fail_safe();
    subscriber_.reset();
    publisher_.reset();
    if (lock_ >= 0) {
      flock(lock_, LOCK_UN);
      ::close(lock_);
      lock_ = -1;
    }
  }

private:
  YAML::Node config_;
  std::vector<size_t> ids_;
  double state_timeout_, startup_timeout_;
  int lock_ = -1;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool ready_ = false;
  unitree_hg::msg::dds_::LowState_ state_{};
  std::chrono::steady_clock::time_point received_;
  std::unique_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> publisher_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> subscriber_;
};
} // namespace
void register_g1_backend(BackendRegistry &r) {
  r.add("g1", [](auto &robot, auto &cfg, double rate) {
    return inline_hardware(std::make_unique<G1Driver>(robot, cfg), robot, cfg, rate);
  });
}
} // namespace boltrt
