// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class Grit final : public Observation {
  Selection select_;
  std::vector<int> offsets_;
  bool reference_;
  std::vector<History> histories_;
  std::vector<Vec> proprio(const ObservationContext &c, bool zero = false) const {
    Vec a, g, r;
    append(a, c.state.angular_velocity);
    append(g, gravity(c.state.orientation));
    columns(r,
            c.state.orientation.conjugate() * select_.at(c, 0).body_orientation.at(select_.root));
    return {a,
            g,
            r,
            select_.q(c, false, true),
            select_.q(c, true),
            zero ? Vec(c.previous_action.size()) : c.previous_action};
  }

public:
  Grit(const YAML::Node &n, const ObservationContext &c, bool reference)
      : select_(n, c), offsets_(offsets(n, c)), reference_(reference) {
    if (select_.names.size() != 29)
      throw std::runtime_error("GRIT needs 29 joints");
    if (reference_ && offsets_.size() != 9)
      throw std::runtime_error("GRIT needs nine reference frames");
  }
  void reset(const ObservationContext &c) override {
    histories_.clear();
    for (auto &v : proprio(c, true)) {
      History h;
      h.reset(10, v);
      histories_.push_back(h);
    }
  }
  void update(const ObservationContext &c) override {
    if (!reference_) {
      auto values = proprio(c);
      for (size_t i = 0; i < values.size(); ++i)
        histories_[i].push(values[i]);
    }
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    if (reference_) {
      for (int step : offsets_) {
        auto &f = select_.at(c, step);
        add(v, select_.ref(f));
        add(v, select_.ref(f, true));
        auto q = f.body_orientation.at(select_.root);
        columns(v, q);
        append(v, rotate(q.conjugate(), linear(f, select_.root)));
        append(v, rotate(q.conjugate(), angular(f, select_.root)));
      }
      return tensor(v, {1, 9, 70});
    }
    for (auto &h : histories_)
      add(v, h.all(true));
    return Tensor::floats(v);
  }
};
} // namespace
void register_grit_observations(ObservationRegistry &r) {
  r.add("grit.grit_reference_context",
        [](auto &n, auto &c) { return std::make_unique<Grit>(n, c, true); });
  r.add("grit.grit_proprio_history",
        [](auto &n, auto &c) { return std::make_unique<Grit>(n, c, false); });
}
} // namespace boltrt
