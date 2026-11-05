// SPDX-License-Identifier: Apache-2.0
#include "boltrt/config/config.hpp"
#include "hf_hub.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <set>
namespace boltrt {
void Tensor::validate() const {
  size_t n = 1;
  for (auto d : shape) {
    if (d < 0 || (d > 0 && n > std::numeric_limits<size_t>::max() / size_t(d)))
      throw std::runtime_error("invalid tensor shape");
    n *= size_t(d);
  }
  if (n != size())
    throw std::runtime_error("tensor shape/data mismatch");
}
size_t index_of(const Names &n, const std::string &s) {
  auto i = std::find(n.begin(), n.end(), s);
  if (i == n.end())
    throw std::runtime_error("missing name: " + s);
  return size_t(i - n.begin());
}
void finite(const Vec &v, const std::string &name) {
  for (float x : v)
    if (!std::isfinite(x))
      throw std::runtime_error("nonfinite " + name);
}
void require_keys(const YAML::Node &n, const Names &allowed, const std::string &where) {
  if (!n.IsMap())
    throw std::runtime_error(where + " must be a mapping");
  std::set<std::string> seen;
  for (auto kv : n) {
    auto k = kv.first.as<std::string>();
    if (!seen.insert(k).second)
      throw std::runtime_error("duplicate key " + where + "." + k);
    if (std::find(allowed.begin(), allowed.end(), k) == allowed.end())
      throw std::runtime_error("unknown key " + where + "." + k);
  }
}
static void unique(const Names &n, const std::string &label) {
  if (n.empty() || std::set<std::string>(n.begin(), n.end()).size() != n.size())
    throw std::runtime_error("empty or duplicate " + label);
}
static YAML::Node expand_merges(const YAML::Node &n, int depth = 0) {
  if (depth > 64)
    throw std::runtime_error("YAML merge nesting exceeds 64");
  if (n.IsSequence()) {
    YAML::Node result(YAML::NodeType::Sequence);
    for (auto item : n)
      result.push_back(expand_merges(item, depth + 1));
    return result;
  }
  if (!n.IsMap())
    return YAML::Clone(n);
  YAML::Node result(YAML::NodeType::Map);
  auto merge = n["<<"];
  if (merge) {
    auto insert = [&](const YAML::Node &map) {
      if (!map.IsMap())
        throw std::runtime_error("YAML merge source must be a mapping");
      for (auto kv : expand_merges(map, depth + 1)) {
        auto key = kv.first.as<std::string>();
        if (!result[key])
          result[key] = kv.second;
      }
    };
    if (merge.IsSequence())
      for (auto map : merge)
        insert(map);
    else
      insert(merge);
  }
  for (auto kv : n) {
    auto key = kv.first.as<std::string>();
    if (key != "<<")
      result[key] = expand_merges(kv.second, depth + 1);
  }
  return result;
}
std::filesystem::path resolve_asset(const std::string &value, const std::filesystem::path &base) {
  if (value.rfind("hf://", 0) == 0) {
    auto rest = value.substr(5);
    auto a = rest.find('/'), b = rest.find('/', a + 1);
    if (a == std::string::npos || b == std::string::npos)
      throw std::runtime_error("HF asset needs owner/repo/file");
    auto repo = rest.substr(0, b), file = rest.substr(b + 1), rev = std::string("main");
    auto at = repo.find('@');
    if (at != std::string::npos) {
      rev = repo.substr(at + 1);
      repo.resize(at);
    }
    return hf::resolve(repo, rev, file, value); // snapshot directory keeps mesh paths valid
  }
  if (value.find("://") != std::string::npos)
    throw std::runtime_error("unsupported asset URI: " + value);
  auto path = std::filesystem::path(value);
  if (value.rfind("~/", 0) == 0) {
    auto h = std::getenv("HOME");
    if (!h)
      throw std::runtime_error("HOME missing");
    path = std::filesystem::path(h) / value.substr(2);
  }
  return std::filesystem::absolute(path.is_absolute() ? path : base / path).lexically_normal();
}
Vec resolve_values(const YAML::Node &n, const Names &names, float fallback, bool require_all) {
  Vec out(names.size(), fallback);
  if (!n)
    return out;
  if (n.IsScalar())
    std::fill(out.begin(), out.end(), n.as<float>());
  else if (n.IsSequence()) {
    out = n.as<Vec>();
    if (out.size() != names.size())
      throw std::runtime_error("joint vector size mismatch");
  } else if (n.IsMap()) {
    std::vector<bool> seen(names.size());
    for (auto kv : n) {
      std::regex pattern(kv.first.as<std::string>());
      bool matched = false;
      for (size_t i = 0; i < names.size(); ++i)
        if (std::regex_match(names[i], pattern)) {
          if (seen[i])
            throw std::runtime_error("overlapping joint regex: " + names[i]);
          seen[i] = true;
          matched = true;
          out[i] = kv.second.as<float>();
        }
      if (!matched && require_all)
        throw std::runtime_error("unmatched joint regex: " + kv.first.as<std::string>());
    }
    if (require_all && std::find(seen.begin(), seen.end(), false) != seen.end())
      throw std::runtime_error("incomplete joint mapping");
  } else
    throw std::runtime_error("expected scalar/list/joint regex map");
  finite(out, "joint values");
  return out;
}
Names select_names(const YAML::Node &query, const Names &source, const Names &preferred) {
  Names chosen;
  if (!query)
    chosen = source;
  else if (query.IsSequence())
    chosen = query.as<Names>();
  else {
    std::regex p(query.as<std::string>());
    for (auto &n : source)
      if (std::regex_match(n, p))
        chosen.push_back(n);
  }
  unique(chosen, "selected names");
  for (auto &n : chosen)
    index_of(source, n);
  if (preferred.empty())
    return chosen;
  Names out;
  for (auto &n : preferred)
    if (std::find(chosen.begin(), chosen.end(), n) != chosen.end())
      out.push_back(n);
  if (out.size() != chosen.size())
    throw std::runtime_error("preferred layout does not cover selected names");
  return out;
}
RobotSpec load_robot(const std::filesystem::path &path) {
  auto n = YAML::LoadFile(path.string());
  require_keys(n,
               {"name", "joint_names", "body_names", "model", "root_body", "initial_q",
                "initial_position", "effort_limit", "joint_armature", "joint_frictionloss",
                "hardware"},
               "robot");
  RobotSpec r;
  r.path = std::filesystem::absolute(path);
  if (n["hardware"]) {
    if (!n["hardware"].IsMap() || !n["hardware"]["driver"])
      throw std::runtime_error("robot hardware section needs a driver");
    r.hardware = n["hardware"];
  }
  r.name = n["name"].as<std::string>();
  r.joints = n["joint_names"].as<Names>();
  r.bodies = n["body_names"].as<Names>();
  unique(r.joints, "robot joints");
  unique(r.bodies, "robot bodies");
  r.model = resolve_asset(n["model"].as<std::string>(), path.parent_path()).string();
  r.root_body = n["root_body"].as<std::string>();
  index_of(r.bodies, r.root_body);
  r.initial_q = resolve_values(n["initial_q"], r.joints);
  r.effort = resolve_values(n["effort_limit"], r.joints, 0, true);
  r.armature = resolve_values(n["joint_armature"], r.joints);
  r.friction = resolve_values(n["joint_frictionloss"], r.joints);
  for (float x : r.effort)
    if (x < 0)
      throw std::runtime_error("effort limit must be nonnegative (zero for passive joints)");
  if (n["initial_position"]) {
    auto v = n["initial_position"].as<Vec>();
    if (v.size() != 3)
      throw std::runtime_error("initial_position needs 3 values");
    r.initial_position = V3(v[0], v[1], v[2]);
  }
  return r;
}
PolicyConfig load_policy(const std::filesystem::path &path, const RobotSpec &r) {
  PolicyConfig p;
  p.path = std::filesystem::absolute(path);
  p.yaml = expand_merges(YAML::LoadFile(path.string()));
  auto n = p.yaml;
  if (n["compatible_robots"]) {
    const auto compatible = n["compatible_robots"].as<Names>();
    if (std::find(compatible.begin(), compatible.end(), r.name) == compatible.end())
      throw std::runtime_error("policy is incompatible with robot: " + r.name);
  }
  p.joints = n["joint_names_simulation"].as<Names>();
  p.bodies = n["body_names_simulation"].as<Names>();
  p.actions = n["policy_joint_names"].as<Names>();
  unique(p.joints, "policy joints");
  unique(p.actions, "action joints");
  unique(p.bodies, "policy bodies");
  for (auto &j : p.joints)
    index_of(r.joints, j);
  for (auto &j : p.actions)
    index_of(r.joints, j);
  // Body layouts describe the reference model. Selected names are checked by
  // observations against the actual motion source, which may use a policy MJCF.
  if (n["joint_effort_limit"] || n["joint_armature"] || n["joint_frictionloss"])
    std::cerr << "Policy physics fields are ignored; fixed dynamics and torque "
                 "limits come from robot config: "
              << path << '\n';
  for (const auto &name : p.actions)
    if (r.effort[index_of(r.joints, name)] <= 0)
      throw std::runtime_error("policy cannot actuate passive joint: " + name);
  p.default_q = resolve_values(n["default_joint_pos"], r.joints);
  p.scale = resolve_values(n["action_scale"], p.actions, 1, true);
  p.kp = resolve_values(n["joint_kp"], r.joints);
  p.kd = resolve_values(n["joint_kd"], r.joints);
  for (auto *v : {&p.kp, &p.kd})
    for (auto x : *v)
      if (x < 0)
        throw std::runtime_error("negative PD gain");
  p.model = resolve_asset(n["model_path"] ? n["model_path"].as<std::string>()
                                          : path.stem().string() + ".onnx",
                          p.path.parent_path());
  auto clip = n["action_clip"] ? n["action_clip"] : n["clip_actions"];
  if (clip) {
    p.action_clip = clip.as<float>();
    if (!(p.action_clip > 0) || !std::isfinite(p.action_clip))
      throw std::runtime_error("action clip must be finite and positive");
  }
  return p;
}
std::filesystem::path resolve_policy_asset(const std::string &value, const PolicyConfig &p) {
  if (value.find("://") != std::string::npos || std::filesystem::path(value).is_absolute() ||
      value.rfind("~/", 0) == 0)
    return resolve_asset(value, p.path.parent_path());
  for (auto dir = p.path.parent_path();; dir = dir.parent_path()) {
    auto candidate = resolve_asset(value, dir);
    if (std::filesystem::is_regular_file(candidate))
      return candidate;
    if (dir == dir.root_path())
      break;
  }
  throw std::runtime_error("policy asset absent: " + value + " declared in " + p.path.string());
}
BackendConfig load_backend(const std::filesystem::path &path) {
  BackendConfig b;
  b.path = std::filesystem::absolute(path);
  b.yaml = YAML::LoadFile(path.string());
  b.type = b.yaml["type"].as<std::string>();
  if (b.yaml["physics_dt"])
    b.dt = b.yaml["physics_dt"].as<double>();
  if (b.yaml["realtime"])
    b.realtime = b.yaml["realtime"].as<bool>();
  if (!std::isfinite(b.dt) || b.dt <= 0)
    throw std::runtime_error("invalid physics_dt");
  return b;
}
BackendConfig hardware_backend(const RobotSpec &robot) {
  if (!robot.hardware)
    throw std::runtime_error("robot " + robot.name + " has no hardware section for --backend real");
  BackendConfig b;
  b.path = robot.path; // relative hardware paths resolve against the robot YAML
  b.yaml = YAML::Clone(robot.hardware);
  b.type = b.yaml["driver"].as<std::string>();
  b.realtime = true;
  return b;
}
} // namespace boltrt
