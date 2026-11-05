// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/motion.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <zmq.h>
namespace boltrt {
namespace {
int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class ZmqMotion final : public MotionSource {
public:
  ZmqMotion(const std::string &address, const RobotSpec &r, std::vector<int> offsets, double rate)
      : joints_(r.joints), bodies_(r.bodies), offsets_(std::move(offsets)),
        dt_(int64_t(1e9 / rate)), root_(index_of(r.bodies, r.root_body)) {
    context_ = zmq_ctx_new();
    if (!context_)
      throw std::runtime_error("zmq_ctx_new failed");
    socket_ = zmq_socket(context_, ZMQ_SUB);
    if (!socket_) {
      close();
      throw std::runtime_error("zmq_socket failed");
    }
    int hwm = 64, linger = 0, heartbeat = 1000, reconnect = 100;
    int64_t max_size = 8 * 1024 * 1024;
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket_, ZMQ_MAXMSGSIZE, &max_size, sizeof(max_size));
    zmq_setsockopt(socket_, ZMQ_HEARTBEAT_IVL, &heartbeat, sizeof(heartbeat));
    zmq_setsockopt(socket_, ZMQ_RECONNECT_IVL, &reconnect, sizeof(reconnect));
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(socket_, address.c_str()) != 0) {
      close();
      throw std::runtime_error("ZMQ connect failed");
    }
  }
  ~ZmqMotion() override { close(); }
  const Names &joint_names() const override { return joints_; }
  const Names &body_names() const override { return bodies_; }
  void reset() override {
    frame_ = 0;
    held_.reset();
  }
  MotionWindow sample(bool advance, bool paused) override {
    if (frames_.empty()) {
      zmq_pollitem_t item{socket_, 0, ZMQ_POLLIN, 0};
      if (zmq_poll(&item, 1, 5000) <= 0)
        throw std::runtime_error("no teleop motion received within 5s; no device started");
    }
    for (int n = 0; n < 128; ++n) {
      zmq_msg_t msg;
      zmq_msg_init(&msg);
      int size = zmq_msg_recv(&msg, socket_, ZMQ_DONTWAIT);
      if (size < 0) {
        zmq_msg_close(&msg);
        break;
      }
      try {
        auto j = nlohmann::json::parse(static_cast<const char *>(zmq_msg_data(&msg)),
                                       static_cast<const char *>(zmq_msg_data(&msg)) +
                                           zmq_msg_size(&msg));
        ingest(j);
      } catch (const std::exception &error) {
        const auto now = now_ns();
        if (now - last_warning_ > 1000000000LL) {
          std::cerr << "teleop: ignored invalid payload: " << error.what() << '\n';
          last_warning_ = now;
        }
      }
      zmq_msg_close(&msg);
    }
    if (frames_.empty())
      throw std::runtime_error("teleop has no valid frames");
    if (advance && !paused)
      ++frame_;
    MotionWindow w;
    w.joints = joints_;
    w.bodies = bodies_;
    w.offsets = offsets_;
    w.frame = frame_;
    w.dt = dt_ * 1e-9;
    if (paused) {
      if (!held_)
        held_ = interpolate(now_ns() - delay_);
      for (auto o : offsets_) {
        (void)o;
        w.frames.push_back(*held_);
        std::fill(w.frames.back().dq.begin(), w.frames.back().dq.end(), 0);
      }
    } else {
      held_.reset();
      for (auto o : offsets_)
        w.frames.push_back(interpolate(now_ns() - delay_ + int64_t(o) * dt_));
    }
    return w;
  }

private:
  void close() {
    if (socket_)
      zmq_close(socket_);
    if (context_)
      zmq_ctx_term(context_);
    socket_ = nullptr;
    context_ = nullptr;
  }
  void ingest(const nlohmann::json &j) {
    auto jn = j.at("joint_names").get<Names>(), bn = j.at("body_names").get<Names>();
    for (const auto &name : joints_)
      index_of(jn, name);
    for (const auto &name : bodies_)
      index_of(bn, name);
    auto q = j.at("joint_pos").get<Vec>();
    if (q.size() != jn.size())
      throw std::runtime_error("teleop joint shape mismatch");
    finite(q, "teleop joints");
    MotionFrame f;
    f.dq = Vec(joints_.size());
    for (auto &name : joints_)
      f.q.push_back(q[index_of(jn, name)]);
    auto pos = j.at("body_pos_w"), ori = j.at("body_quat_w");
    if (pos.size() != bn.size() || ori.size() != bn.size())
      throw std::runtime_error("teleop body shape mismatch");
    for (auto &name : bodies_) {
      auto id = index_of(bn, name);
      auto p = pos[id].get<Vec>(), qv = ori[id].get<Vec>();
      if (p.size() != 3 || qv.size() != 4)
        throw std::runtime_error("teleop pose shape mismatch");
      finite(p, "teleop position");
      finite(qv, "teleop orientation");
      Quat quat(qv[0], qv[1], qv[2], qv[3]);
      if (quat.norm() < 1e-8)
        throw std::runtime_error("zero teleop quaternion");
      f.body_position.emplace_back(p[0], p[1], p[2]);
      f.body_orientation.push_back(quat.normalized());
    }
    if (j.contains("joint_vel")) {
      auto dq = j.at("joint_vel").get<Vec>();
      if (dq.size() != jn.size())
        throw std::runtime_error("teleop joint velocity shape mismatch");
      finite(dq, "teleop velocity");
      for (size_t i = 0; i < joints_.size(); ++i)
        f.dq[i] = dq[index_of(jn, joints_[i])];
    }
    const auto recv = now_ns();
    int64_t seq = j.value("seq", int64_t(last_seq_ + 1));
    bool restarted = seq < last_seq_;
    if (seq == last_seq_ && !j.value("motion_first_frame", false))
      return;
    auto source = j.value("pico_recv_time_ns", j.value("publish_t_ns", int64_t(0)));
    int64_t t = recv;
    if (source) {
      if (!clock_aligned_ || restarted || std::abs(source + clock_offset_ - recv) > 5000000000LL) {
        clock_offset_ = recv - source;
        clock_aligned_ = true;
      }
      t = source + clock_offset_;
    }
    bool first = j.value("motion_first_frame", j.value("first_frame", false));
    if (first) {
      src_ = f.body_position[root_];
      src_.z() = 0;
      dst_ = src_;
      rotation_ = Quat::Identity();
      if (!frames_.empty()) {
        auto &previous = frames_.back().frame;
        dst_ = previous.body_position[root_];
        dst_.z() = 0;
        rotation_ =
            yaw(previous.body_orientation[root_]) * yaw(f.body_orientation[root_]).conjugate();
      }
      frames_.clear();
    } else if (restarted)
      frames_.clear();
    for (size_t b = 0; b < bodies_.size(); ++b) {
      f.body_position[b] = rotation_ * (f.body_position[b] - src_) + dst_;
      f.body_orientation[b] = rotation_ * f.body_orientation[b];
    }
    if (!frames_.empty() && t <= frames_.back().time)
      return;
    f.body_linear_velocity.assign(bodies_.size(), V3::Zero());
    f.body_angular_velocity.assign(bodies_.size(), V3::Zero());
    if (!frames_.empty()) {
      double dt = double(t - frames_.back().time) * 1e-9;
      for (size_t b = 0; b < bodies_.size(); ++b) {
        const auto &previous = frames_.back().frame;
        f.body_linear_velocity[b] = (f.body_position[b] - previous.body_position[b]) / dt;
        auto d = f.body_orientation[b] * previous.body_orientation[b].conjugate();
        if (d.w() < 0)
          d.coeffs() *= -1;
        Eigen::AngleAxisd aa(d.normalized());
        f.body_angular_velocity[b] = aa.axis() * aa.angle() / dt;
      }
    }
    frames_.push_back({t, std::move(f), j.contains("joint_vel")});
    while (frames_.size() > 512)
      frames_.pop_front();
    last_seq_ = seq;
  }
  MotionFrame interpolate(int64_t target) const {
    if (target <= frames_.front().time) {
      auto f = frames_.front().frame;
      std::fill(f.dq.begin(), f.dq.end(), 0);
      for (auto *values : {&f.body_linear_velocity, &f.body_angular_velocity})
        for (auto &v : *values)
          v.setZero();
      return f;
    }
    if (target >= frames_.back().time) {
      auto f = frames_.back().frame;
      std::fill(f.dq.begin(), f.dq.end(), 0);
      for (auto *values : {&f.body_linear_velocity, &f.body_angular_velocity})
        for (auto &v : *values)
          v.setZero();
      return f;
    }
    auto hi = std::upper_bound(frames_.begin(), frames_.end(), target,
                               [](int64_t t, const Sample &s) { return t < s.time; });
    auto lo = hi - 1;
    double a = double(target - lo->time) / double(hi->time - lo->time);
    MotionFrame f = lo->frame;
    for (size_t k = 0; k < f.q.size(); ++k) {
      f.q[k] = float((1 - a) * lo->frame.q[k] + a * hi->frame.q[k]);
      f.dq[k] = lo->velocity && hi->velocity
                    ? float((1 - a) * lo->frame.dq[k] + a * hi->frame.dq[k])
                    : float((hi->frame.q[k] - lo->frame.q[k]) * 1e9 / double(hi->time - lo->time));
    }
    for (size_t k = 0; k < bodies_.size(); ++k) {
      f.body_position[k] = (1 - a) * lo->frame.body_position[k] + a * hi->frame.body_position[k];
      f.body_orientation[k] = lo->frame.body_orientation[k].slerp(a, hi->frame.body_orientation[k]);
      f.body_linear_velocity[k] =
          (1 - a) * lo->frame.body_linear_velocity[k] + a * hi->frame.body_linear_velocity[k];
      f.body_angular_velocity[k] =
          (1 - a) * lo->frame.body_angular_velocity[k] + a * hi->frame.body_angular_velocity[k];
    }
    return f;
  }
  struct Sample {
    int64_t time;
    MotionFrame frame;
    bool velocity;
  };
  void *context_ = nullptr;
  void *socket_ = nullptr;
  Names joints_, bodies_;
  std::vector<int> offsets_;
  int64_t dt_, frame_ = 0, delay_ = 40000000, last_seq_ = -1, clock_offset_ = 0, last_warning_ = 0;
  bool clock_aligned_ = false;
  size_t root_;
  std::deque<Sample> frames_;
  std::optional<MotionFrame> held_;
  Quat rotation_ = Quat::Identity();
  V3 src_ = V3::Zero(), dst_ = V3::Zero();
};
} // namespace
std::unique_ptr<MotionSource> zmq_motion(const std::string &address, const RobotSpec &r,
                                         const std::vector<int> &offsets, double rate) {
  return std::make_unique<ZmqMotion>(address, r, offsets, rate);
}
} // namespace boltrt
