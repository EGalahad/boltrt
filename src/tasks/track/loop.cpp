// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/loop.hpp"
#include <algorithm>
#include <iomanip>
#include <sstream>
namespace boltrt {
namespace {
using Json = nlohmann::json;
MotionSource &require_source(const std::unique_ptr<MotionSource> &source) {
  if (!source)
    throw std::runtime_error("tracking requires a motion source");
  return *source;
}
Json motion_json(const MotionWindow &window) {
  Json result;
  result["offsets"] = window.offsets;
  result["frame"] = window.frame;
  result["last_frame"] = window.last_frame;
  result["dt"] = window.dt;
  result["joint_names"] = window.joints;
  result["body_names"] = window.bodies;
  for (const auto &f : window.frames) {
    Json frame{{"q", f.q}, {"dq", f.dq}, {"smpl_valid", f.smpl_valid}};
    auto vectors = [&](const char *key, const std::vector<V3> &values) {
      frame[key] = Json::array();
      for (const auto &v : values)
        frame[key].push_back({v.x(), v.y(), v.z()});
    };
    vectors("body_position", f.body_position);
    vectors("body_linear_velocity", f.body_linear_velocity);
    vectors("body_angular_velocity", f.body_angular_velocity);
    vectors("smpl_joints", f.smpl_joints);
    frame["body_orientation"] = Json::array();
    for (const auto &q : f.body_orientation)
      frame["body_orientation"].push_back({q.w(), q.x(), q.y(), q.z()});
    frame["smpl_orientation"] = {f.smpl_orientation.w(), f.smpl_orientation.x(),
                                 f.smpl_orientation.y(), f.smpl_orientation.z()};
    result["frames"].push_back(std::move(frame));
  }
  return result;
}
} // namespace
TrackingLoop::TrackingLoop(const RobotSpec &r, const PolicyConfig &p, RobotBackend &b,
                           MotionSource &m, Inference &i, const ObservationRegistry &registry,
                           LoopOptions options)
    : PolicyLoop(r, p, b, i, registry, std::move(options)), source_(m) {}
TrackingLoop::TrackingLoop(const RobotSpec &r, const PolicyConfig &p, RobotBackend &b,
                           std::unique_ptr<MotionSource> source, Inference &i,
                           const ObservationRegistry &registry, LoopOptions options)
    : TrackingLoop(r, p, b, require_source(source), i, registry, std::move(options)) {
  owned_source_ = std::move(source);
}
void TrackingLoop::prepare_task() {
  if (!playback_)
    playback_ = std::make_unique<MotionPlayback>(source_);
  window_ = playback_->current();
}
void TrackingLoop::start_task(bool start) {
  playback_->restart();
  window_ = playback_->current();
  initial_motion_ = window_;
  if (start)
    playback_->play();
}
void TrackingLoop::bind_task_context(ObservationContext &context) const {
  context.task = &tracking_context_;
}
void TrackingLoop::enter_policy_task() {
  playback_->restart();
  window_ = playback_->current();
}
void TrackingLoop::update_task() { window_ = playback_->tick(); }
bool TrackingLoop::handle_task_event(Event event, std::string &message) {
  if (event == Event::TogglePause) {
    playback_->toggle_pause();
    message = playback_->paused() ? "Reference paused" : "Reference resumed";
    return true;
  }
  if (event == Event::Restart) {
    playback_->restart();
    window_ = playback_->current();
    message = "Motion restarted";
    return true;
  }
  return false;
}
bool TrackingLoop::task_finished() const { return window_.ended; }
void TrackingLoop::update_task_status(RuntimeStatus &status) const {
  auto number = [](double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
  };
  const double progress =
      window_.last_frame > 0 ? std::clamp(double(window_.frame) / window_.last_frame, 0., 1.) : 0;
  const int filled = int(progress * 40);
  status.task_status = {"Motion : " + status.task,
                        std::string("Reference: ") + (playback_->paused() ? "PAUSED" : "PLAYING"),
                        "[" + std::string(filled, '=') + std::string(40 - filled, '-') + "] " +
                            number(progress * 100) + "%",
                        "Motion : " + number(window_.frame * window_.dt) + " / " +
                            number(window_.last_frame * window_.dt) + " s   Frame " +
                            std::to_string(window_.frame) + "/" +
                            std::to_string(window_.last_frame)};
  status.task_controls = {"Motion : [Space] Play / Pause reference motion"};
  if (status.last_event == "Policy active")
    status.last_event = "Policy active; motion reset and paused (Space plays)";
}
void TrackingLoop::append_task_record(nlohmann::json &j, bool first) const {
  j["motion_frame"] = window_.frame;
  j["paused"] = playback_->paused();
  if (first) {
    j["layout"]["body_names"] = window_.bodies;
    j["layout"]["reference_math"] = "fp32";
  }
  const auto current = std::find(window_.offsets.begin(), window_.offsets.end(), 0);
  const auto &reference = window_.frames.at(size_t(current - window_.offsets.begin()));
  j["reference"]["q"] = reference.q;
  j["motion"] = motion_json(window_);
  if (first)
    j["initial_motion"] = motion_json(initial_motion_);
  for (size_t b = 0; b < reference.body_position.size(); ++b) {
    const auto &p = reference.body_position[b];
    const auto &q = reference.body_orientation[b];
    j["reference"]["body_position"].push_back({p.x(), p.y(), p.z()});
    j["reference"]["body_orientation"].push_back({q.w(), q.x(), q.y(), q.z()});
  }
}
} // namespace boltrt
