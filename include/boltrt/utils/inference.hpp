// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/config/config.hpp"
#include <memory>
namespace boltrt {
class Inference {
public:
  virtual ~Inference() = default;
  virtual TensorMap run(const TensorMap &) = 0;
  virtual void reset() = 0;
  virtual void validate(const TensorMap &) const = 0;
};
std::unique_ptr<Inference> onnx_inference(const PolicyConfig &, int threads = 1);
} // namespace boltrt
