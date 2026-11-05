// SPDX-License-Identifier: Apache-2.0
// Minimal Hugging Face Hub snapshot resolver: reads the standard HF cache and,
// unless HF_HUB_OFFLINE is set, downloads a missing repo snapshot with curl.
#include "hf_hub.hpp"
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace boltrt::hf {
namespace fs = std::filesystem;
namespace {
std::string env(const char *name) {
  auto v = std::getenv(name);
  return v ? v : "";
}
bool truthy(const std::string &v) {
  std::string s;
  for (char c : v)
    s += char(std::tolower(static_cast<unsigned char>(c)));
  return s == "1" || s == "true" || s == "yes" || s == "on";
}
bool is_commit(const std::string &rev) {
  return rev.size() == 40 &&
         std::all_of(rev.begin(), rev.end(), [](char c) { return std::isxdigit(static_cast<unsigned char>(c)); });
}
fs::path cache_root() {
  if (auto p = env("HF_HUB_CACHE"); !p.empty())
    return p;
  if (auto p = env("HF_HOME"); !p.empty())
    return fs::path(p) / "hub";
  auto home = env("HOME");
  if (home.empty())
    throw std::runtime_error("HOME missing");
  return fs::path(home) / ".cache/huggingface/hub";
}
std::string token() {
  if (auto t = env("HF_TOKEN"); !t.empty())
    return t;
  auto home = env("HF_HOME");
  auto file = home.empty() ? (env("HOME").empty() ? fs::path() : fs::path(env("HOME")) / ".cache/huggingface/token")
                           : fs::path(home) / "token";
  std::string t;
  if (!file.empty() && fs::exists(file)) {
    std::ifstream f(file);
    std::getline(f, t);
  }
  return t;
}
std::string url_path(const std::string &s) {
  static const char *hex = "0123456789ABCDEF";
  std::string out;
  for (unsigned char c : s) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~' || c == '/')
      out += char(c);
    else
      out += {'%', hex[c >> 4], hex[c & 15]};
  }
  return out;
}
std::string quote(const std::string &s) {
  std::string out = "\"";
  for (char c : s)
    out += (c == '"' || c == '\\') ? std::string{'\\', c} : std::string{c};
  return out + '"';
}
// Runs curl without a shell. Options, URLs and the auth header are passed as a
// config on stdin so the token never appears in argv or error messages.
std::string curl(const std::string &requests, const std::string &what, bool capture) {
  std::string config = "fail\nsilent\nshow-error\nlocation\nretry = 5\nretry-all-errors\n"
                       "connect-timeout = 20\nparallel\nparallel-max = 8\n";
  if (auto t = token(); !t.empty())
    config += "header = " + quote("Authorization: Bearer " + t) + "\n";
  config += requests;
  int in[2], out[2];
  if (pipe(in) != 0 || (capture && pipe(out) != 0))
    throw std::runtime_error("pipe failed");
  pid_t pid = fork();
  if (pid < 0)
    throw std::runtime_error("fork failed");
  if (pid == 0) {
    dup2(in[0], STDIN_FILENO);
    close(in[0]);
    close(in[1]);
    if (capture) {
      dup2(out[1], STDOUT_FILENO);
      close(out[0]);
      close(out[1]);
    }
    char *argv[] = {const_cast<char *>("curl"), const_cast<char *>("--config"), const_cast<char *>("-"), nullptr};
    execvp("curl", argv);
    _exit(127);
  }
  close(in[0]);
  for (size_t off = 0; off < config.size();) {
    auto n = write(in[1], config.data() + off, config.size() - off);
    if (n <= 0)
      break;
    off += size_t(n);
  }
  close(in[1]);
  std::string result;
  if (capture) {
    close(out[1]);
    char buf[4096];
    for (ssize_t n; (n = read(out[0], buf, sizeof buf)) > 0;)
      result.append(buf, size_t(n));
    close(out[0]);
  }
  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    throw std::runtime_error(WIFEXITED(status) && WEXITSTATUS(status) == 127
                                 ? "curl not found; install curl or pre-populate the HF cache"
                                 : "download failed: " + what);
  return result;
}
} // namespace

fs::path resolve(const std::string &repo, const std::string &revision, const std::string &file,
                 const std::string &reference) {
  auto sep = repo.find('/');
  auto dir = cache_root() / ("models--" + repo.substr(0, sep) + "--" + repo.substr(sep + 1));
  auto commit = revision;
  if (fs::exists(dir / "refs" / revision)) {
    std::ifstream f(dir / "refs" / revision);
    std::getline(f, commit);
  }
  if (fs::is_regular_file(dir / "snapshots" / commit / file))
    return fs::absolute(dir / "snapshots" / commit / file);
  if (truthy(env("HF_HUB_OFFLINE")))
    throw std::runtime_error("asset absent from HF cache and HF_HUB_OFFLINE is set: " + reference);

  auto endpoint = env("HF_ENDPOINT").empty() ? std::string("https://huggingface.co") : env("HF_ENDPOINT");
  const std::string hint = "cannot download " + reference + " from " + endpoint +
                           ". Check the network, set HF_ENDPOINT to a reachable mirror (e.g. "
                           "https://hf-mirror.com), or populate the HF cache and set HF_HUB_OFFLINE=1";
  auto api = endpoint + "/api/models/" + repo + "/revision/" + url_path(revision) + "?blobs=true";
  nlohmann::json info;
  try {
    info = nlohmann::json::parse(curl("url = " + quote(api) + "\n", api, true));
    commit = info.at("sha").get<std::string>();
  } catch (const std::exception &e) {
    throw std::runtime_error(hint + " (" + e.what() + ")");
  }
  auto snapshot = dir / "snapshots" / commit;
  struct Pending {
    std::string name;
    fs::path tmp, target;
    std::int64_t size;
  };
  std::vector<Pending> pending;
  for (auto &s : info.at("siblings")) {
    auto name = s.at("rfilename").get<std::string>();
    auto target = snapshot / name;
    if (fs::is_regular_file(target))
      continue;
    fs::create_directories(target.parent_path());
    auto tmp = target;
    tmp += ".incomplete." + std::to_string(getpid());
    pending.push_back({name, tmp, target, s.value("size", std::int64_t(-1))});
  }
  // A transfer is complete only when the file exists with the size the API reported; curl's exit
  // status alone is not reliable for parallel transfers and mirrors can return empty bodies.
  auto complete = [](const Pending &p) {
    std::error_code ec;
    const auto n = fs::file_size(p.tmp, ec);
    return !ec && (p.size < 0 ? n > 0 || p.size == 0 : std::int64_t(n) == p.size);
  };
  auto request = [&](const Pending &p) {
    return "url = " + quote(endpoint + "/" + repo + "/resolve/" + commit + "/" + url_path(p.name)) +
           "\noutput = " + quote(p.tmp.string()) + "\n";
  };
  if (!pending.empty()) {
    std::cerr << "Downloading hf://" << repo << '@' << commit.substr(0, 7) << " (" << pending.size()
              << " files) from " << endpoint << " to " << dir.string() << std::endl;
    std::string all;
    for (auto &p : pending)
      all += request(p);
    try {
      curl(all, "hf://" + repo + "@" + revision, false);
    } catch (...) {
      // Individual failures are retried below.
    }
    std::vector<Pending> failed;
    for (auto &p : pending)
      if (!complete(p))
        failed.push_back(p);
    for (int attempt = 0; attempt < 2 && !failed.empty(); ++attempt) {
      std::vector<Pending> still;
      for (auto &p : failed) {
        try {
          curl(request(p), p.name, false);
        } catch (...) {
        }
        if (!complete(p))
          still.push_back(p);
      }
      failed.swap(still);
    }
    if (!failed.empty()) {
      for (auto &p : pending)
        fs::remove(p.tmp);
      throw std::runtime_error(hint + " (" + std::to_string(failed.size()) + " of " +
                               std::to_string(pending.size()) + " files incomplete, e.g. " +
                               failed.front().name + ")");
    }
    for (auto &p : pending)
      fs::rename(p.tmp, p.target);
  }
  if (!is_commit(revision)) {
    fs::create_directories(dir / "refs");
    std::ofstream(dir / "refs" / revision) << commit;
  }
  if (!fs::is_regular_file(snapshot / file))
    throw std::runtime_error("file not found in HF repo " + repo + "@" + revision + ": " + file);
  return fs::absolute(snapshot / file);
}
} // namespace boltrt::hf
