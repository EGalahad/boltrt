// SPDX-License-Identifier: Apache-2.0
#include "boltrt/observation/observation.hpp"
#include <algorithm>
#include <set>
namespace boltrt {
void ObservationRegistry::add(const std::string &name, Factory f) {
  if (aliases_.count(name) || !factories_.emplace(name, std::move(f)).second)
    throw std::runtime_error("duplicate observation: " + name);
}
void ObservationRegistry::alias(const std::string &name, const std::string &target) {
  if (!factories_.count(target) || factories_.count(name) || !aliases_.emplace(name, target).second)
    throw std::runtime_error("invalid observation alias: " + name);
}
std::unique_ptr<Observation> ObservationRegistry::create(const std::string &name,
                                                         const YAML::Node &cfg,
                                                         const ObservationContext &c) const {
  auto alias = aliases_.find(name);
  auto i = factories_.find(alias == aliases_.end() ? name : alias->second);
  if (i != factories_.end())
    return i->second(cfg, c);
  if (name.find('.') == std::string::npos) {
    const Factory *found = nullptr;
    for (auto &[k, f] : factories_)
      if (k.substr(k.find_last_of('.') + 1) == name) {
        if (found)
          throw std::runtime_error("ambiguous observation: " + name);
        found = &f;
      }
    if (found)
      return (*found)(cfg, c);
  }
  throw std::runtime_error("unknown observation: " + name);
}
Names ObservationRegistry::names() const {
  Names n;
  for (auto &[k, f] : factories_)
    n.push_back(k);
  for (auto &[k, target] : aliases_)
    n.push_back(k);
  return n;
}
ObservationGraph::ObservationGraph(const ObservationRegistry &reg, const YAML::Node &cfg,
                                   const ObservationContext &c) {
  if (!cfg.IsMap() || cfg.size() == 0)
    throw std::runtime_error("observation must contain groups");
  std::set<std::string> seen;
  for (auto g : cfg) {
    Group group;
    group.name = g.first.as<std::string>();
    if (!seen.insert(group.name).second)
      throw std::runtime_error("duplicate observation group");
    std::set<std::string> terms;
    if (!g.second.IsMap() || g.second.size() == 0)
      throw std::runtime_error("observation group must contain terms");
    for (auto item : g.second) {
      auto name = item.first.as<std::string>();
      if (!terms.insert(name).second)
        throw std::runtime_error("duplicate observation term");
      auto target = item.second["_target_"] ? item.second["_target_"].as<std::string>() : name;
      group.terms.push_back({name, reg.create(target, item.second, c)});
    }
    groups_.push_back(std::move(group));
  }
}
void ObservationGraph::reset(const ObservationContext &c) {
  for (auto &g : groups_)
    for (auto &term : g.terms)
      term.obs->reset(c);
}
TensorMap ObservationGraph::compute(const ObservationContext &c, TensorMap *components) {
  TensorMap out;
  for (auto &g : groups_) {
    Tensor combined;
    bool first = true;
    for (auto &term : g.terms) {
      term.obs->update(c);
      auto t = term.obs->compute(c);
      t.validate();
      if (t.shape.empty())
        throw std::runtime_error("scalar observation needs an explicit dimension");
      if (components)
        (*components)[g.name + "/" + term.name] = t;
      if (first) {
        combined = t;
        first = false;
        continue;
      }
      if (t.dtype() != combined.dtype() || t.shape.size() != combined.shape.size() ||
          !std::equal(t.shape.begin(), t.shape.end() - 1, combined.shape.begin()))
        throw std::runtime_error("incompatible observation group shapes/types: " + g.name);
      auto width = t.shape.back(), oldwidth = combined.shape.back();
      if (width <= 0 || oldwidth <= 0)
        throw std::runtime_error("empty observation");
      size_t rows = t.size() / size_t(width);
      std::visit(
          [&](auto &dest) {
            using T = std::decay_t<decltype(dest)>;
            auto &src = std::get<T>(t.data);
            T joined;
            joined.reserve(dest.size() + src.size());
            for (size_t r = 0; r < rows; ++r) {
              joined.insert(joined.end(), dest.begin() + r * oldwidth,
                            dest.begin() + (r + 1) * oldwidth);
              joined.insert(joined.end(), src.begin() + r * width, src.begin() + (r + 1) * width);
            }
            dest = std::move(joined);
          },
          combined.data);
      combined.shape.back() += width;
    }
    out.emplace(g.name, std::move(combined));
  }
  return out;
}
} // namespace boltrt
