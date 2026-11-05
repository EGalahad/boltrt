// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include <string>

namespace boltrt::hf {
// Path of `file` inside the cached snapshot of `repo@revision`. Downloads the
// snapshot into the standard HF cache when missing, unless HF_HUB_OFFLINE is set.
std::filesystem::path resolve(const std::string &repo, const std::string &revision, const std::string &file,
                              const std::string &reference);
}
