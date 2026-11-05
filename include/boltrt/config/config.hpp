// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/core/types.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>
namespace boltrt {
struct RobotSpec {
  std::string name;
  Names joints, bodies;
  std::string model, root_body;
  Vec initial_q, effort, armature, friction;
  V3 initial_position{0, 0, 0.8};
};
struct PolicyConfig {
  YAML::Node yaml;
  std::filesystem::path path, model;
  Names joints, bodies, actions;
  Vec default_q, scale, kp, kd;
  float action_clip = std::numeric_limits<float>::infinity();
};
struct BackendConfig {
  YAML::Node yaml;
  std::filesystem::path path;
  std::string type;
  double dt = 0.005;
  bool realtime = false;
};
RobotSpec load_robot(const std::filesystem::path &);
PolicyConfig load_policy(const std::filesystem::path &, const RobotSpec &);
BackendConfig load_backend(const std::filesystem::path &);
Vec resolve_values(const YAML::Node &, const Names &, float fallback = 0, bool require_all = false);
Names select_names(const YAML::Node &, const Names &source, const Names &preferred);
std::filesystem::path resolve_asset(const std::string &, const std::filesystem::path &base);
std::filesystem::path resolve_policy_asset(const std::string &, const PolicyConfig &);
void require_keys(const YAML::Node &, const Names &, const std::string &);
} // namespace boltrt
