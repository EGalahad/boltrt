// SPDX-License-Identifier: Apache-2.0
#include "boltrt/runtime/task.hpp"
#include <set>
namespace boltrt {
TaskRegistry::TaskRegistry() {
  add("base", [](TaskServices s) {
    return std::make_unique<PolicyLoop>(s.robot, s.policy, s.backend, s.inference, s.observations,
                                        std::move(s.options));
  });
}
void TaskRegistry::add(std::string name, Factory factory, Observations observations,
                       Names options) {
  if (name.empty() || !factory ||
      !definitions_
           .emplace(std::move(name),
                    Definition{std::move(factory), std::move(observations), std::move(options)})
           .second)
    throw std::runtime_error("invalid or duplicate task registration");
}
void TaskRegistry::prepare_observations(const std::string &name,
                                        ObservationRegistry &registry) const {
  const auto found = definitions_.find(name);
  if (found == definitions_.end())
    throw std::runtime_error("unknown task: " + name);
  register_common_observations(registry);
  if (found->second.observations)
    found->second.observations(registry);
}
std::unique_ptr<PolicyLoop> TaskRegistry::create(const std::string &name,
                                                 TaskServices services) const {
  const auto found = definitions_.find(name);
  if (found == definitions_.end())
    throw std::runtime_error("unknown task: " + name);
  for (const auto &[task, definition] : definitions_)
    if (task != name)
      for (const auto &option : definition.options)
        if (services.arguments.count(option))
          throw std::runtime_error("--" + option + " requires --task " + task);
  return found->second.factory(std::move(services));
}
Names TaskRegistry::names() const {
  Names result;
  for (const auto &[name, _] : definitions_)
    result.push_back(name);
  return result;
}
Names TaskRegistry::option_names() const {
  std::set<std::string> result;
  for (const auto &[_, definition] : definitions_)
    result.insert(definition.options.begin(), definition.options.end());
  return Names(result.begin(), result.end());
}
} // namespace boltrt
