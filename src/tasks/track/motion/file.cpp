// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/motion.hpp"
#include "boltrt/utils/mujoco/model.hpp"
#include <algorithm>
#include <bit>
#include <cstring>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <regex>
#include <zlib.h>
namespace boltrt {
namespace {
struct Array {
  size_t rows = 0, cols = 0;
  Vec values;
};
uint32_t u32(const std::vector<uint8_t> &b, size_t p) {
  if (p + 4 > b.size())
    throw std::runtime_error("truncated zip");
  return uint32_t(b[p]) | (uint32_t(b[p + 1]) << 8) | (uint32_t(b[p + 2]) << 16) |
         (uint32_t(b[p + 3]) << 24);
}
uint16_t u16(const std::vector<uint8_t> &b, size_t p) { return uint16_t(u32(b, p) & 65535); }
Array load_qpos(const std::filesystem::path &path, const std::string &entry = "qpos.npy") {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("motion file not found: " + path.string());
  std::vector<uint8_t> zip((std::istreambuf_iterator<char>(in)), {});
  if (zip.size() < 22)
    throw std::runtime_error("invalid NPZ");
  size_t end = zip.size() - 22;
  while (u32(zip, end) != 0x06054b50) {
    if (end == 0 || zip.size() - end > 65557)
      throw std::runtime_error("NPZ central directory missing");
    --end;
  }
  size_t p = u32(zip, end + 16);
  uint16_t count = u16(zip, end + 10);
  std::vector<uint8_t> npy;
  for (int k = 0; k < count; ++k) {
    if (u32(zip, p) != 0x02014b50)
      throw std::runtime_error("bad NPZ central directory");
    auto method = u16(zip, p + 10);
    size_t compressed = u32(zip, p + 20), size = u32(zip, p + 24), nl = u16(zip, p + 28),
           el = u16(zip, p + 30), cl = u16(zip, p + 32), local = u32(zip, p + 42);
    if (p + 46 + nl > zip.size())
      throw std::runtime_error("truncated zip filename");
    std::string name(reinterpret_cast<char *>(zip.data() + p + 46), nl);
    p += 46 + nl + el + cl;
    if (name != entry)
      continue;
    if (size > 512 * 1024 * 1024)
      throw std::runtime_error("motion exceeds 512 MiB limit");
    if (u32(zip, local) != 0x04034b50)
      throw std::runtime_error("bad zip local entry");
    size_t off = local + 30 + u16(zip, local + 26) + u16(zip, local + 28);
    if (off + compressed > zip.size())
      throw std::runtime_error("truncated zip payload");
    npy.resize(size);
    if (method == 0) {
      if (compressed != size)
        throw std::runtime_error("zip size mismatch");
      std::copy_n(zip.data() + off, size, npy.data());
    } else if (method == 8) {
      z_stream z{};
      z.next_in = zip.data() + off;
      z.avail_in = static_cast<uInt>(compressed);
      z.next_out = npy.data();
      z.avail_out = static_cast<uInt>(size);
      if (inflateInit2(&z, -MAX_WBITS) != Z_OK)
        throw std::runtime_error("inflate init failed");
      int result = inflate(&z, Z_FINISH);
      inflateEnd(&z);
      if (result != Z_STREAM_END || z.total_out != size)
        throw std::runtime_error("NPZ inflate failed");
    } else
      throw std::runtime_error("unsupported ZIP compression");
    if (crc32(0, npy.data(), static_cast<uInt>(npy.size())) != u32(zip, p - 46 - nl - el - cl + 16))
      throw std::runtime_error("NPZ CRC mismatch");
    break;
  }
  if (npy.size() < 12 || std::memcmp(npy.data(), "\x93NUMPY", 6) != 0)
    throw std::runtime_error("numeric qpos.npy required");
  size_t hs = npy[6] == 1 ? 10 : 12,
         hl = npy[6] == 1 ? size_t(npy[8] | (npy[9] << 8)) : u32(npy, 8);
  if (hs + hl > npy.size())
    throw std::runtime_error("bad NPY header");
  std::string header(reinterpret_cast<char *>(npy.data() + hs), hl);
  std::smatch m;
  if (!std::regex_search(header, m, std::regex("'descr':\\s*'([^']+)'")))
    throw std::runtime_error("NPY dtype missing");
  auto dtype = m[1].str();
  if (dtype != "<f4" && dtype != "<f8" && dtype != "|f4" && dtype != "|f8")
    throw std::runtime_error("qpos must be little-endian float32/64; pickle/object unsupported");
  const bool fortran = header.find("True") != std::string::npos;
  if (!std::regex_search(header, m,
                         std::regex("'shape':\\s*\\(\\s*([0-9]+)\\s*,\\s*([0-9]+)\\s*,?\\s*\\)")))
    throw std::runtime_error("qpos must be 2D");
  Array a;
  a.rows = std::stoull(m[1]);
  a.cols = std::stoull(m[2]);
  size_t bytes = dtype.back() == '4' ? 4 : 8;
  if (a.rows == 0 || a.cols == 0 || a.rows > npy.size() / bytes / a.cols ||
      hs + hl + a.rows * a.cols * bytes != npy.size())
    throw std::runtime_error("qpos shape/payload mismatch");
  a.values.resize(a.rows * a.cols);
  static_assert(std::endian::native == std::endian::little);
  for (size_t i = 0; i < a.values.size(); ++i) {
    const size_t source = fortran ? (i % a.cols) * a.rows + i / a.cols : i;
    if (bytes == 4)
      std::memcpy(&a.values[i], npy.data() + hs + hl + source * bytes, 4);
    else {
      double d;
      std::memcpy(&d, npy.data() + hs + hl + source * bytes, 8);
      a.values[i] = static_cast<float>(d);
    }
  }
  finite(a.values, "motion qpos");
  return a;
}
class FileMotion final : public MotionSource {
public:
  FileMotion(const std::filesystem::path &path, const RobotSpec &r, const std::vector<int> &offsets,
             double rate, bool fp16)
      : joints_(r.joints), bodies_(r.bodies), offsets_(offsets) {
    dt_ = 1 / rate;
    auto dir = std::filesystem::absolute(path).parent_path();
    while (!std::filesystem::exists(dir / "manifest.json")) {
      if (dir == dir.root_path())
        throw std::runtime_error("any4hdmi manifest.json required above motion");
      dir = dir.parent_path();
    }
    std::ifstream input(dir / "manifest.json");
    nlohmann::json manifest;
    input >> manifest;
    double source_dt = manifest.at("timestep").get<double>();
    if (!std::isfinite(source_dt) || source_dt <= 0)
      throw std::runtime_error("invalid motion timestep");
    auto names = manifest.at("qpos_names").get<Names>();
    auto a = load_qpos(path);
    if (names.size() != a.cols)
      throw std::runtime_error("qpos_names shape mismatch");
    RobotModel fk(r);
    // Accept the model's named freejoint as well as canonical root_* fields.
    // Only aliases for this robot's actual root are valid; never infer a layout.
    if (std::find(names.begin(), names.end(), "root_tx") == names.end()) {
      const char *root = mj_id2name(fk.model.get(), mjOBJ_JOINT, 0);
      if (root) {
        for (const auto *suffix : {"tx", "ty", "tz", "qw", "qx", "qy", "qz"}) {
          const auto column = index_of(names, std::string(root) + "_" + suffix);
          names[column] = std::string("root_") + suffix;
        }
      }
    }
    std::vector<size_t> columns;
    for (auto &n : joints_)
      columns.push_back(index_of(names, n));
    for (auto n : {"root_tx", "root_ty", "root_tz", "root_qw", "root_qx", "root_qy", "root_qz"})
      index_of(names, n);
    auto getpos = [&](size_t row) {
      return V3(a.values[row * a.cols + index_of(names, "root_tx")],
                a.values[row * a.cols + index_of(names, "root_ty")],
                a.values[row * a.cols + index_of(names, "root_tz")]);
    };
    auto getquat = [&](size_t row) {
      Quat q(a.values[row * a.cols + index_of(names, "root_qw")],
             a.values[row * a.cols + index_of(names, "root_qx")],
             a.values[row * a.cols + index_of(names, "root_qy")],
             a.values[row * a.cols + index_of(names, "root_qz")]);
      if (q.norm() < 1e-8)
        throw std::runtime_error("zero motion quaternion");
      return q.normalized();
    };
    size_t length = size_t(std::floor(double(a.rows - 1) * source_dt * rate + 1e-9)) + 1;
    frames_.reserve(length);
    // Match any4hdmi: differentiate at source fps, interpolate qpos/qvel,
    // then run FK with those velocities (mjData.cvel is the deployed contract).
    std::vector<Vec> source_velocity(a.rows, Vec(6 + joints_.size()));
    for (size_t row = 0; row + 1 < a.rows; ++row) {
      V3 linear = (getpos(row + 1) - getpos(row)) / source_dt;
      auto delta = (getquat(row).conjugate() * getquat(row + 1)).normalized();
      if (delta.w() < 0)
        delta.coeffs() *= -1;
      Eigen::AngleAxisd aa(delta);
      auto angular = (aa.axis() * aa.angle() / source_dt).eval();
      for (int j = 0; j < 3; ++j) {
        source_velocity[row][j] = float(linear[j]);
        source_velocity[row][3 + j] = float(angular[j]);
      }
      for (size_t j = 0; j < columns.size(); ++j)
        source_velocity[row][6 + j] = float((double(a.values[(row + 1) * a.cols + columns[j]]) -
                                             a.values[row * a.cols + columns[j]]) /
                                            source_dt);
    }
    if (a.rows > 1)
      source_velocity.back() = source_velocity[a.rows - 2];
    // Optional supplied velocities preserve the originating framework's
    // derivative convention.
    if (manifest.contains("joint_velocity_names")) {
      const auto velocity_names = manifest.at("joint_velocity_names").get<Names>();
      const auto velocity_key = manifest.value("joint_velocity_key", std::string("joint_vel"));
      const auto supplied = load_qpos(path, velocity_key + ".npy");
      if (supplied.rows != a.rows || supplied.cols != velocity_names.size())
        throw std::runtime_error("supplied joint velocity shape/name mismatch");
      for (size_t j = 0; j < joints_.size(); ++j) {
        const auto col = index_of(velocity_names, joints_[j]);
        for (size_t row = 0; row < a.rows; ++row)
          source_velocity[row][6 + j] = supplied.values[row * supplied.cols + col];
      }
    }

    for (size_t k = 0; k < length; ++k) {
      double t = double(k) / rate / source_dt;
      size_t lo = std::min(size_t(t), a.rows - 1), hi = std::min(lo + 1, a.rows - 1);
      double alpha = t - double(lo);
      MotionFrame f;
      for (auto col : columns)
        f.q.push_back(static_cast<float>((1 - alpha) * a.values[lo * a.cols + col] +
                                         alpha * a.values[hi * a.cols + col]));
      fk.set_pose(f.q, (1 - alpha) * getpos(lo) + alpha * getpos(hi),
                  getquat(lo).slerp(alpha, getquat(hi)));
      Vec velocity(source_velocity[lo].size());
      for (size_t j = 0; j < velocity.size(); ++j)
        velocity[j] = float((1 - alpha) * source_velocity[lo][j] + alpha * source_velocity[hi][j]);
      for (int j = 0; j < 6; ++j)
        fk.data->qvel[j] = velocity[j];
      for (size_t j = 0; j < joints_.size(); ++j) {
        f.dq.push_back(velocity[6 + j]);
        fk.data->qvel[fk.vadr[j]] = velocity[6 + j];
      }
      mj_forward(fk.model.get(), fk.data.get());
      for (int id : fk.bodies) {
        f.body_linear_velocity.emplace_back(fk.data->cvel[6 * id + 3], fk.data->cvel[6 * id + 4],
                                            fk.data->cvel[6 * id + 5]);
        f.body_angular_velocity.emplace_back(fk.data->cvel[6 * id], fk.data->cvel[6 * id + 1],
                                             fk.data->cvel[6 * id + 2]);
        f.body_position.emplace_back(fk.data->xpos[3 * id], fk.data->xpos[3 * id + 1],
                                     fk.data->xpos[3 * id + 2]);
        f.body_orientation.emplace_back(fk.data->xquat[4 * id], fk.data->xquat[4 * id + 1],
                                        fk.data->xquat[4 * id + 2], fk.data->xquat[4 * id + 3]);
      }
      frames_.push_back(std::move(f));
    }
    // Match any4hdmi's FP16 backing store before observations cast to float32.
    if (fp16)
      for (auto &f : frames_) {
        auto half = [](double x) { return float(Eigen::half(float(x))); };
        for (auto &x : f.q)
          x = half(x);
        for (auto &x : f.dq)
          x = half(x);
        for (auto &p : f.body_position)
          for (int i = 0; i < 3; ++i)
            p[i] = half(p[i]);
        for (auto &q : f.body_orientation)
          for (int i = 0; i < 4; ++i)
            q.coeffs()[i] = half(q.coeffs()[i]);
        for (auto *values : {&f.body_linear_velocity, &f.body_angular_velocity})
          for (auto &v : *values)
            for (int i = 0; i < 3; ++i)
              v[i] = half(v[i]);
      }
  }
  void reset() override { frame_ = 0; }
  MotionWindow sample(bool advance, bool paused) override {
    if (advance && !paused)
      frame_ = std::min(frame_ + 1, int64_t(frames_.size() - 1));
    MotionWindow w;
    w.joints = joints_;
    w.bodies = bodies_;
    w.offsets = offsets_;
    w.frame = frame_;
    w.last_frame = int64_t(frames_.size() - 1);
    w.dt = dt_;
    w.ended = frame_ == int64_t(frames_.size() - 1);
    for (int o : offsets_) {
      w.frames.push_back(frames_[size_t(
          std::clamp(frame_ + (paused ? 0 : o), int64_t(0), int64_t(frames_.size() - 1)))]);
      if (paused) {
        std::fill(w.frames.back().dq.begin(), w.frames.back().dq.end(), 0);
        for (auto *values :
             {&w.frames.back().body_linear_velocity, &w.frames.back().body_angular_velocity})
          for (auto &v : *values)
            v.setZero();
      }
    }
    return w;
  }
  const Names &joint_names() const override { return joints_; }
  const Names &body_names() const override { return bodies_; }

private:
  Names joints_, bodies_;
  std::vector<int> offsets_;
  std::vector<MotionFrame> frames_;
  int64_t frame_ = 0;
  double dt_ = 0.02;
};
} // namespace
std::unique_ptr<MotionSource> file_motion(const std::filesystem::path &path, const RobotSpec &r,
                                          const std::vector<int> &offsets, double rate, bool fp16) {
  return std::make_unique<FileMotion>(path, r, offsets, rate, fp16);
}
} // namespace boltrt
