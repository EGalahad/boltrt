// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/runtime/policy_loop.hpp"
#include "boltrt/tasks/track/context.hpp"
#include "boltrt/tasks/track/playback.hpp"
namespace boltrt {
class TrackingLoop final : public PolicyLoop {
public:
  TrackingLoop(const RobotSpec &, const PolicyConfig &, RobotBackend &, MotionSource &, Inference &,
               const ObservationRegistry &, LoopOptions);

  TrackingLoop(const RobotSpec &, const PolicyConfig &, RobotBackend &,
               std::unique_ptr<MotionSource>, Inference &, const ObservationRegistry &,
               LoopOptions);

protected:
  void prepare_task() override;
  void start_task(bool) override;
  void bind_task_context(ObservationContext &) const override;
  void enter_policy_task() override;
  void update_task() override;
  bool handle_task_event(Event, std::string &) override;
  bool task_finished() const override;
  void update_task_status(RuntimeStatus &) const override;
  void append_task_record(nlohmann::json &, bool) const override;

private:
  std::unique_ptr<MotionSource> owned_source_;
  MotionSource &source_;
  std::unique_ptr<MotionPlayback> playback_;
  MotionWindow window_, initial_motion_;
  TrackingContext tracking_context_{window_};
};
} // namespace boltrt
