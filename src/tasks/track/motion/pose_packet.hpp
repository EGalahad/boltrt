// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <bit>
#include <cmath>
#include <cstring>
#include <functional>
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace boltrt {
inline nlohmann::json decode_pose_packet(const char *bytes, size_t size) {
  using Json = nlohmann::json;
  size_t start = 0;
  while (start < size && std::isspace(static_cast<unsigned char>(bytes[start])))
    ++start;
  if (start < size && bytes[start] == '{')
    return Json::parse(bytes + start, bytes + size);
  size_t header = 0;
  while (header < std::min(size, size_t(256)) && bytes[header] != '{')
    ++header;
  constexpr size_t header_size = 1280;
  if (header + header_size > size)
    throw std::runtime_error("pose packet header missing/truncated");
  size_t end = header;
  while (end < header + header_size && bytes[end])
    ++end;
  auto meta = Json::parse(bytes + header, bytes + end);
  if (meta.value("endian", std::string("")) != "le" || std::endian::native != std::endian::little)
    throw std::runtime_error("pose packet requires little endian");
  size_t position = header + header_size;
  Json result;
  for (auto &field : meta.at("fields")) {
    auto dtype = field.at("dtype").get<std::string>();
    auto shape = field.at("shape").get<std::vector<size_t>>();
    size_t width = dtype == "f64" || dtype == "i64"   ? 8
                   : dtype == "f32" || dtype == "i32" ? 4
                   : dtype == "bool"                  ? 1
                                                      : 0;
    if (!width)
      throw std::runtime_error("pose packet dtype unsupported");
    size_t count = 1;
    for (size_t d : shape) {
      if (d > 1000000 || count > 1000000 / std::max(d, size_t(1)))
        throw std::runtime_error("pose field exceeds size limit");
      count *= d;
    }
    if (position > size || count > (size - position) / width)
      throw std::runtime_error("pose packet field truncated");
    size_t offset = 0;
    auto scalar = [&]() -> Json {
      const char *p = bytes + position + offset++ * width;
      if (dtype == "bool")
        return bool(*p);
      if (dtype == "i64") {
        int64_t x;
        std::memcpy(&x, p, 8);
        return x;
      }
      if (dtype == "i32") {
        int32_t x;
        std::memcpy(&x, p, 4);
        return x;
      }
      double x;
      if (dtype == "f64")
        std::memcpy(&x, p, 8);
      else {
        float f;
        std::memcpy(&f, p, 4);
        x = f;
      }
      if (!std::isfinite(x))
        throw std::runtime_error("pose packet nonfinite field");
      return x;
    };
    std::function<Json(size_t)> build = [&](size_t axis) -> Json {
      if (axis == shape.size())
        return scalar();
      Json array = Json::array();
      for (size_t i = 0; i < shape[axis]; ++i)
        array.push_back(build(axis + 1));
      return array;
    };
    result[field.at("name").get<std::string>()] = build(0);
    position += count * width;
  }
  if (position != size)
    throw std::runtime_error("pose packet trailing payload");
  return result;
}
inline bool pose_bool(const nlohmann::json &j, const char *key, bool fallback = false) {
  if (!j.contains(key))
    return fallback;
  auto v = j.at(key);
  while (v.is_array()) {
    if (v.empty())
      return fallback;
    v = v[0];
  }
  return v.is_boolean() ? v.get<bool>() : v.get<int>() != 0;
}
} // namespace boltrt
