// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/motion.hpp"
#include "pose_packet.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <zmq.h>
namespace boltrt {
namespace {
using Json = nlohmann::json;
int64_t smpl_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class SmplMotion final : public MotionSource {
  Names joints_, bodies_;
  std::vector<int> offsets_;
  int64_t dt_, frame_ = 0;
  void *context_ = nullptr, *socket_ = nullptr;
  struct Sample {
    int64_t time;
    MotionFrame frame;
  };
  std::deque<Sample> samples_;
  std::optional<MotionFrame> held_;
  Quat segment_ = Quat::Identity();
  int64_t origin_time_ = 0;
  bool file_ = false;
  void close() {
    if (socket_)
      zmq_close(socket_);
    if (context_)
      zmq_ctx_term(context_);
    socket_ = context_ = nullptr;
  }
  void ingest(const Json &payload, int64_t receive) {
    if (!payload.contains("smpl_joint_pos_root") || !payload.contains("smpl_root_quat_w") ||
        !payload.contains("joint_pos"))
      throw std::runtime_error("SMPL stream requires smpl_joint_pos_root[N,24,3], "
                               "smpl_root_quat_w[N,4], joint_pos[N,J]");
    const auto &joint = payload.at("joint_pos"), &smpl = payload.at("smpl_joint_pos_root"),
               &ori = payload.at("smpl_root_quat_w");
    const size_t count = joint.size();
    if (!count || count > 4096 || smpl.size() != count || ori.size() != count)
      throw std::runtime_error("SMPL frame batch mismatch");
    auto names = payload.value("joint_names", joints_);
    if (names.size() != joints_.size())
      throw std::runtime_error("SMPL joint layout mismatch");
    std::vector<MotionFrame> parsed;
    parsed.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      MotionFrame f;
      auto q = joint[i].get<Vec>();
      finite(q, "SMPL robot joints");
      if (q.size() != names.size() || smpl[i].size() != 24)
        throw std::runtime_error("SMPL joint shape mismatch");
      for (auto &name : joints_)
        f.q.push_back(q[index_of(names, name)]);
      f.dq = Vec(joints_.size());
      if (payload.contains("joint_vel")) {
        auto dq = payload.at("joint_vel")[i].get<Vec>();
        finite(dq, "SMPL robot velocities");
        if (dq.size() != names.size())
          throw std::runtime_error("SMPL velocity shape mismatch");
        for (size_t j = 0; j < joints_.size(); ++j)
          f.dq[j] = dq[index_of(names, joints_[j])];
      }
      for (auto &row : smpl[i]) {
        auto v = row.get<Vec>();
        if (v.size() != 3)
          throw std::runtime_error("SMPL joints must have 3 coordinates");
        finite(v, "SMPL human joints");
        f.smpl_joints.emplace_back(v[0], v[1], v[2]);
      }
      auto qv = ori[i].get<Vec>();
      if (qv.size() != 4)
        throw std::runtime_error("SMPL orientation must be wxyz");
      finite(qv, "SMPL orientation");
      f.smpl_orientation = Quat(qv[0], qv[1], qv[2], qv[3]);
      if (f.smpl_orientation.norm() < 1e-8)
        throw std::runtime_error("zero SMPL quaternion");
      f.smpl_orientation.normalize();
      f.smpl_valid = true;
      // Robot reference body poses are unavailable in the SMPL stream. Leave
      // those fields absent rather than synthesizing robot FK from human data.
      parsed.push_back(std::move(f));
    }
    bool first = pose_bool(payload, "motion_first_frame", pose_bool(payload, "first_frame", false));
    auto rotation = segment_;
    if (first)
      rotation =
          (samples_.empty() ? Quat::Identity() : yaw(samples_.back().frame.smpl_orientation)) *
          yaw(parsed.front().smpl_orientation).conjugate();
    if (first) {
      segment_ = rotation;
      samples_.clear();
    }
    for (size_t i = 0; i < count; ++i) {
      int64_t t = receive - (int64_t(count - 1 - i)) * dt_;
      auto f = std::move(parsed[i]);
      f.smpl_orientation = rotation * f.smpl_orientation;
      if (!samples_.empty() && t <= samples_.back().time)
        continue;
      samples_.push_back({t, std::move(f)});
    }
    while (samples_.size() > 4096)
      samples_.pop_front();
  }
  MotionFrame interpolate(int64_t t) const {
    if (t <= samples_.front().time)
      return samples_.front().frame;
    if (t >= samples_.back().time) {
      auto f = samples_.back().frame;
      std::fill(f.dq.begin(), f.dq.end(), 0);
      return f;
    }
    auto hi = std::upper_bound(samples_.begin(), samples_.end(), t,
                               [](int64_t t, const Sample &s) { return t < s.time; }),
         lo = hi - 1;
    double a = double(t - lo->time) / double(hi->time - lo->time);
    auto f = lo->frame;
    for (size_t j = 0; j < f.q.size(); ++j) {
      f.q[j] = float((1 - a) * lo->frame.q[j] + a * hi->frame.q[j]);
      f.dq[j] = float((1 - a) * lo->frame.dq[j] + a * hi->frame.dq[j]);
    }
    for (size_t j = 0; j < 24; ++j)
      f.smpl_joints[j] = (1 - a) * lo->frame.smpl_joints[j] + a * hi->frame.smpl_joints[j];
    f.smpl_orientation = lo->frame.smpl_orientation.slerp(a, hi->frame.smpl_orientation);
    return f;
  }

public:
  SmplMotion(const std::string &address, const RobotSpec &r, const std::vector<int> &offsets,
             double rate)
      : joints_(r.joints), bodies_(r.bodies), offsets_(offsets), dt_(int64_t(1e9 / rate)) {
    if (address.rfind("tcp://", 0) != 0) {
      file_ = true;
      std::ifstream f(address);
      if (!f)
        throw std::runtime_error("SMPL reference must be a JSON motion capture or tcp:// stream; "
                                 "robot NPZ is not an SMPL reference");
      Json j;
      f >> j;
      if (j.is_array()) {
        int64_t t = 0;
        for (auto &packet : j) {
          ingest(packet, t);
          t += dt_;
        }
      } else
        ingest(j, 0);
      if (samples_.empty())
        throw std::runtime_error("empty SMPL recording");
      origin_time_ = samples_.front().time;
      return;
    }
    context_ = zmq_ctx_new();
    if (!context_)
      throw std::runtime_error("SMPL ZMQ context failed");
    socket_ = zmq_socket(context_, ZMQ_SUB);
    if (!socket_) {
      close();
      throw std::runtime_error("SMPL ZMQ socket failed");
    }
    int hwm = 64, linger = 0;
    int64_t max_size = 8 * 1024 * 1024;
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket_, ZMQ_MAXMSGSIZE, &max_size, sizeof(max_size));
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(socket_, address.c_str())) {
      close();
      throw std::runtime_error("SMPL ZMQ connect failed");
    }
  }
  ~SmplMotion() override { close(); }
  void reset() override {
    frame_ = 0;
    held_.reset();
  }
  const Names &joint_names() const override { return joints_; }
  const Names &body_names() const override { return bodies_; }
  MotionWindow sample(bool advance, bool paused) override {
    if (!file_) {
      if (samples_.empty()) {
        zmq_pollitem_t item{socket_, 0, ZMQ_POLLIN, 0};
        if (zmq_poll(&item, 1, 5000) <= 0)
          throw std::runtime_error("no SMPL motion received within 5s; no device started");
      }
      for (int i = 0; i < 128; ++i) {
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        if (zmq_msg_recv(&msg, socket_, ZMQ_DONTWAIT) < 0) {
          zmq_msg_close(&msg);
          break;
        }
        try {
          auto text = static_cast<const char *>(zmq_msg_data(&msg));
          auto j = decode_pose_packet(text, zmq_msg_size(&msg));
          ingest(j, smpl_now());
        } catch (const std::exception &e) {
          std::cerr << "SMPL teleop: ignored payload: " << e.what() << '\n';
        }
        zmq_msg_close(&msg);
      }
    }
    if (samples_.empty())
      throw std::runtime_error("SMPL stream has no valid frames");
    auto last = file_ ? (samples_.back().time - origin_time_) / dt_ : int64_t(-1);
    if (advance && !paused)
      frame_ = file_ ? std::min(frame_ + 1, last) : frame_ + 1;
    int64_t current = file_ ? origin_time_ + frame_ * dt_ : smpl_now() - 40000000;
    MotionWindow w;
    w.joints = joints_;
    w.bodies = bodies_;
    w.offsets = offsets_;
    w.frame = frame_;
    w.last_frame = last;
    w.dt = dt_ * 1e-9;
    w.ended = file_ && frame_ == last;
    if (paused && !held_)
      held_ = interpolate(current);
    if (!paused)
      held_.reset();
    for (int offset : offsets_) {
      auto f = paused ? *held_ : interpolate(current + offset * dt_);
      if (paused)
        std::fill(f.dq.begin(), f.dq.end(), 0);
      w.frames.push_back(std::move(f));
    }
    return w;
  }
};
} // namespace
std::unique_ptr<MotionSource> smpl_motion(const std::string &address, const RobotSpec &r,
                                          const std::vector<int> &offsets, double rate) {
  return std::make_unique<SmplMotion>(address, r, offsets, rate);
}
} // namespace boltrt
