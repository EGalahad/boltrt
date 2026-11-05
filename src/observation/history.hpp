// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/observation/observation.hpp"
#include <algorithm>
namespace boltrt::observation_detail {
class History final : public Observation {
public:
  using Reader = std::function<Vec(const ObservationContext &)>;
  History(std::vector<int> steps, Reader reader)
      : steps_(std::move(steps)), reader_(std::move(reader)) {
    if (steps_.empty() || *std::min_element(steps_.begin(), steps_.end()) < 0 ||
        *std::max_element(steps_.begin(), steps_.end()) > 10000)
      throw std::runtime_error("invalid history steps");
  }
  void reset(const ObservationContext &c) override {
    history_.assign(size_t(*std::max_element(steps_.begin(), steps_.end()) + 1), reader_(c));
  }
  void update(const ObservationContext &c) override {
    if (history_.empty())
      reset(c);
    std::rotate(history_.rbegin(), history_.rbegin() + 1, history_.rend());
    history_[0] = reader_(c);
  }
  Tensor compute(const ObservationContext &) const override {
    Vec v;
    for (int step : steps_)
      v.insert(v.end(), history_.at(size_t(step)).begin(), history_.at(size_t(step)).end());
    return Tensor::floats(std::move(v));
  }

private:
  std::vector<int> steps_;
  Reader reader_;
  std::vector<Vec> history_;
};
class PreviousActions final : public Observation {
public:
  explicit PreviousActions(int steps) : steps_(steps) {
    if (steps <= 0 || steps > 10000)
      throw std::runtime_error("invalid prev_action steps");
  }
  void reset(const ObservationContext &c) override {
    history_.assign(size_t(steps_), Vec(c.previous_action.size(), 0));
  }
  void update(const ObservationContext &c) override {
    if (history_.empty())
      reset(c);
    std::rotate(history_.rbegin(), history_.rbegin() + 1, history_.rend());
    history_[0] = c.previous_action;
  }
  Tensor compute(const ObservationContext &) const override {
    Vec v;
    for (auto &a : history_)
      v.insert(v.end(), a.begin(), a.end());
    return Tensor::floats(std::move(v));
  }

private:
  int steps_;
  std::vector<Vec> history_;
};
} // namespace boltrt::observation_detail
