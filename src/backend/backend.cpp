// SPDX-License-Identifier: Apache-2.0
#include "boltrt/backend/backend.hpp"
namespace boltrt {
void BackendRegistry::add(const std::string &name, BackendFactory f) {
  if (!factories_.emplace(name, std::move(f)).second)
    throw std::runtime_error("duplicate backend: " + name);
}
std::unique_ptr<RobotBackend> BackendRegistry::create(const RobotSpec &r, const BackendConfig &b,
                                                      double rate) const {
  auto f = factories_.find(b.type);
  if (f == factories_.end())
    throw std::runtime_error("backend unavailable in this build: " + b.type);
  return f->second(r, b, rate);
}
Names BackendRegistry::names() const {
  Names n;
  for (auto &[name, f] : factories_)
    n.push_back(name);
  return n;
}
void validate_command(const JointCommand &c, size_t n) {
  for (auto *v : {&c.q, &c.dq, &c.tau, &c.kp, &c.kd}) {
    if (v->size() != n)
      throw std::runtime_error("command joint count mismatch");
    finite(*v, "command");
  }
  for (auto *v : {&c.kp, &c.kd})
    for (float x : *v)
      if (x < 0)
        throw std::runtime_error("negative command gains");
}
} // namespace boltrt
