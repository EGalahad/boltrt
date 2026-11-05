// SPDX-License-Identifier: Apache-2.0
#include "boltrt/utils/inference.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <onnxruntime_cxx_api.h>
namespace boltrt {
namespace {
struct Binding {
  std::string name, key;
  std::vector<int64_t> shape;
  ONNXTensorElementDataType type;
};
std::string key_name(const nlohmann::json &j) {
  if (j.is_string())
    return j.get<std::string>();
  if (j.is_array() && j.size() == 2)
    return j[0].get<std::string>() + "/" + j[1].get<std::string>();
  throw std::runtime_error("unsupported ONNX binding key");
}
std::string normalized(std::string n, bool input) {
  if (n.size() >= 5 && n.substr(n.size() - 5) == "_orig")
    n.resize(n.size() - 5);
  if (n.rfind("next_", 0) == 0)
    n = (input ? "" : "next/") + n.substr(5);
  return n;
}
class OnnxInference final : public Inference {
public:
  OnnxInference(const PolicyConfig &p, int threads)
      : env_(ORT_LOGGING_LEVEL_WARNING, "boltrt"),
        memory_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    options_.SetIntraOpNumThreads(threads);
    options_.SetInterOpNumThreads(1);
    options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    session_ = std::make_unique<Ort::Session>(env_, p.model.c_str(), options_);
    auto sidecar = p.model;
    sidecar.replace_extension(".json");
    nlohmann::json meta;
    if (p.yaml && p.yaml["onnx"]) {
      const auto cfg = p.yaml["onnx"];
      require_keys(cfg, {"in_keys", "out_keys"}, "policy.onnx");
      for (const auto *key : {"in_keys", "out_keys"}) {
        if (!cfg[key])
          continue;
        if (!cfg[key].IsSequence())
          throw std::runtime_error(std::string("policy.onnx.") + key + " must be a sequence");
        meta[key] = nlohmann::json::array();
        for (const auto &binding : cfg[key]) {
          if (binding.IsScalar())
            meta[key].push_back(binding.as<std::string>());
          else if (binding.IsSequence() && binding.size() == 2)
            meta[key].push_back({binding[0].as<std::string>(), binding[1].as<std::string>()});
          else
            throw std::runtime_error("invalid ONNX YAML binding");
        }
      }
    } else if (std::filesystem::exists(sidecar)) {
      // Compatibility for original external checkpoints. New exports embed this
      // metadata in policy.yaml and require only ONNX + YAML.
      std::ifstream f(sidecar);
      f >> meta;
    }
    Ort::AllocatorWithDefaultOptions alloc;
    for (bool input : {true, false}) {
      auto count = input ? session_->GetInputCount() : session_->GetOutputCount();
      auto &bindings = input ? inputs_ : outputs_;
      std::string mkey = input ? "in_keys" : "out_keys";
      if (meta.contains(mkey) &&
          (meta[mkey].size() > count || (input && meta[mkey].size() != count)))
        throw std::runtime_error("ONNX binding count mismatch");
      for (size_t i = 0; i < count; ++i) {
        auto name = input ? session_->GetInputNameAllocated(i, alloc)
                          : session_->GetOutputNameAllocated(i, alloc);
        auto info = input ? session_->GetInputTypeInfo(i) : session_->GetOutputTypeInfo(i);
        auto tensor = info.GetTensorTypeAndShapeInfo();
        Binding b{name.get(),
                  meta.contains(mkey) && i < meta[mkey].size() ? key_name(meta[mkey][i])
                                                               : normalized(name.get(), input),
                  tensor.GetShape(), tensor.GetElementType()};
        if (b.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
            b.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 &&
            b.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL)
          throw std::runtime_error("unsupported ONNX dtype: " + b.name);
        bindings.push_back(std::move(b));
      }
    }
    reset();
  }
  void reset() override {
    state_.clear();
    for (auto &b : inputs_)
      if (is_state(b.key)) {
        Tensor t;
        t.shape = b.shape;
        for (auto &d : t.shape) {
          if (d < 0)
            d = 1;
        }
        size_t n = 1;
        for (auto d : t.shape)
          n *= size_t(d);
        if (n > 100000000)
          throw std::runtime_error("recurrent state too large");
        if (b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
          t.data = Vec(n);
        else if (b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64)
          t.data = std::vector<int64_t>(n);
        else
          t.data = std::vector<uint8_t>(n);
        state_[b.key] = std::move(t);
      }
  }
  void validate(const TensorMap &values) const override {
    for (auto &b : inputs_) {
      auto t = lookup(values, b);
      (void)shape(*t, b);
    }
  }
  TensorMap run(const TensorMap &values) override {
    std::vector<Ort::Value> args;
    std::vector<const char *> in_names, out_names;
    for (auto &b : inputs_) {
      auto t = lookup(values, b);
      auto dims = shape(*t, b);
      args.push_back(std::visit(
          [&](auto &v) {
            void *ptr = const_cast<void *>(static_cast<const void *>(v.data()));
            return Ort::Value::CreateTensor(
                memory_, ptr, v.size() * sizeof(typename std::decay_t<decltype(v)>::value_type),
                dims.data(), dims.size(), b.type);
          },
          t->data));
      in_names.push_back(b.name.c_str());
    }
    for (auto &b : outputs_)
      out_names.push_back(b.name.c_str());
    auto output = session_->Run(Ort::RunOptions{nullptr}, in_names.data(), args.data(), args.size(),
                                out_names.data(), out_names.size());
    TensorMap result;
    for (size_t i = 0; i < output.size(); ++i) {
      auto &b = outputs_[i];
      auto info = output[i].GetTensorTypeAndShapeInfo();
      Tensor t;
      t.shape = info.GetShape();
      size_t n = info.GetElementCount();
      if (b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        auto v = output[i].GetTensorData<float>();
        t.data = Vec(v, v + n);
        finite(std::get<Vec>(t.data), "inference output " + b.name);
      } else if (b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
        auto v = output[i].GetTensorData<int64_t>();
        t.data = std::vector<int64_t>(v, v + n);
      } else {
        auto v = output[i].GetTensorData<bool>();
        std::vector<uint8_t> a;
        for (size_t k = 0; k < n; ++k)
          a.push_back(v[k] ? 1 : 0);
        t.data = std::move(a);
      }
      result[b.key] = t;
      if (b.key.rfind("next/", 0) == 0) {
        auto key = b.key.substr(5);
        state_[key] = t;
        state_[b.key] = std::move(t);
      }
    }
    return result;
  }

private:
  bool is_state(const std::string &key) const {
    for (auto &b : outputs_)
      if (b.key == "next/" + key || b.key == key && key.rfind("next/", 0) == 0)
        return true;
    return false;
  }
  const Tensor *lookup(const TensorMap &values, const Binding &b) const {
    auto it = values.find(b.key);
    if (it == values.end())
      it = values.find(b.name);
    if (it != values.end())
      return &it->second;
    auto s = state_.find(b.key);
    if (s != state_.end())
      return &s->second;
    throw std::runtime_error("missing ONNX input: " + b.name + " key=" + b.key);
  }
  std::vector<int64_t> shape(const Tensor &t, const Binding &b) const {
    t.validate();
    auto expected = b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT   ? DType::Float32
                    : b.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 ? DType::Int64
                                                                    : DType::Bool;
    if (t.dtype() != expected)
      throw std::runtime_error("ONNX dtype mismatch: " + b.name);
    auto dims = t.shape;
    if (dims.size() == b.shape.size() + 1 && dims[0] == 1)
      dims.erase(dims.begin());
    if (dims.size() != b.shape.size())
      throw std::runtime_error("ONNX rank mismatch: " + b.name);
    for (size_t i = 0; i < dims.size(); ++i)
      if (b.shape[i] >= 0 && b.shape[i] != dims[i])
        throw std::runtime_error("ONNX shape mismatch: " + b.name + " axis=" + std::to_string(i) +
                                 " got=" + std::to_string(dims[i]) +
                                 " expected=" + std::to_string(b.shape[i]));
    return dims;
  }
  Ort::Env env_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_;
  std::vector<Binding> inputs_, outputs_;
  TensorMap state_;
};
} // namespace
std::unique_ptr<Inference> onnx_inference(const PolicyConfig &p, int threads) {
  if (threads <= 0)
    throw std::runtime_error("ORT threads must be positive");
  return std::make_unique<OnnxInference>(p, threads);
}
} // namespace boltrt
