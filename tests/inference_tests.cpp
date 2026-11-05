// SPDX-License-Identifier: Apache-2.0
#include "boltrt/utils/inference.hpp"
#include <fstream>
#include <iostream>
#include <unistd.h>
using namespace boltrt;
Tensor boolean(bool v) {
  Tensor t;
  t.shape = {1};
  t.data = std::vector<uint8_t>{uint8_t(v)};
  return t;
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("fixture path required");
    PolicyConfig p;
    p.model = argv[1];
    auto model = onnx_inference(p);
    Tensor integer;
    integer.shape = {1, 1};
    integer.data = std::vector<int64_t>{2};
    TensorMap inputs{{"x", Tensor::floats({1, 3})}, {"is_init", boolean(true)}, {"mode", integer}};
    model->validate(inputs);
    auto first = model->run(inputs);
    if (std::get<Vec>(first.at("action").data) != Vec{3, 5})
      throw std::runtime_error("typed input or reset binding failed");
    inputs["is_init"] = boolean(false);
    auto second = model->run(inputs);
    if (std::get<Vec>(second.at("action").data) != Vec{6, 10})
      throw std::runtime_error("next_h_orig state feedback failed");
    model->reset();
    auto third = model->run(inputs);
    if (std::get<Vec>(third.at("action").data) != Vec{3, 5})
      throw std::runtime_error("recurrent state reset failed");
    // Two-file checkpoint: explicit YAML bindings, no JSON sidecar.
    const auto dir =
        std::filesystem::temp_directory_path() / ("boltrt-two-file-" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    std::filesystem::copy_file(p.model, dir / "policy.onnx",
                               std::filesystem::copy_options::overwrite_existing);
    std::ofstream(dir / "policy.yaml") << "onnx:\n  in_keys: [features, h, is_init, mode]\n"
                                          "  out_keys: [action, [next, h]]\n";
    PolicyConfig two;
    two.model = dir / "policy.onnx";
    two.yaml = YAML::LoadFile((dir / "policy.yaml").string());
    auto embedded = onnx_inference(two);
    auto renamed = inputs;
    renamed["features"] = renamed.at("x");
    renamed.erase("x");
    embedded->validate(renamed);
    if (std::get<Vec>(embedded->run(renamed).at("action").data) != Vec{3, 5})
      throw std::runtime_error("embedded YAML bindings failed");
    // Explicit YAML must win over an obsolete JSON sidecar.
    std::ofstream(dir / "policy.json") << R"({"in_keys":["wrong"]})";
    embedded = onnx_inference(two);
    embedded->validate(renamed);
    two.yaml["onnx"]["in_keys"] = YAML::Load("[features]");
    bool rejected = false;
    try {
      auto invalid = onnx_inference(two);
    } catch (const std::exception &) {
      rejected = true;
    }
    if (!rejected)
      throw std::runtime_error("bad YAML binding count accepted");
    std::filesystem::remove_all(dir);
    inputs["mode"] = Tensor::floats({1});
    bool bad = false;
    try {
      model->validate(inputs);
    } catch (const std::exception &) {
      bad = true;
    }
    if (!bad)
      throw std::runtime_error("dtype mismatch accepted");
    std::cout << "typed/recurrent ONNX contracts passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
