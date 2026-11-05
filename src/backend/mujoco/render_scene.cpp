// SPDX-License-Identifier: Apache-2.0
// Render the current runtime scene without a visible window.
#include "boltrt/utils/mujoco/model.hpp"
#include "lodepng.h"
#include "scene.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
int main(int argc, char **argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("render_scene <robot.yaml> <state.jsonl> <output-dir>");
    auto robot = boltrt::load_robot(argv[1]);
    boltrt::RobotModel scene(robot, boltrt::configure_studio_scene);
    std::ifstream trace(argv[2]);
    std::string line;
    std::getline(trace, line);
    auto row = nlohmann::json::parse(line);
    auto q = row.at("q").get<boltrt::Vec>(), p = row.at("root_position").get<boltrt::Vec>(),
         o = row.at("orientation").get<boltrt::Vec>();
    scene.set_pose(q, boltrt::V3(p[0], p[1], p[2]), boltrt::Quat(o[0], o[1], o[2], o[3]));
    if (!glfwInit())
      throw std::runtime_error("GLFW initialization failed");
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    auto window = glfwCreateWindow(960, 720, "Floor material previews", nullptr, nullptr);
    if (!window)
      throw std::runtime_error("GLFW context failed");
    glfwMakeContextCurrent(window);
    constexpr int width = 960, height = 720;
    scene.model->vis.global.offwidth = width;
    scene.model->vis.global.offheight = height;
    scene.model->vis.quality.offsamples = 8;
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.type = mjCAMERA_TRACKING;
    camera.trackbodyid = scene.bodies.front();
    camera.distance = 2.7;
    camera.azimuth = 120;
    camera.elevation = -20;
    mjvOption options;
    mjv_defaultOption(&options);
    mjvScene visual;
    mjv_defaultScene(&visual);
    mjv_makeScene(scene.model.get(), &visual, 5000);
    mjrContext context;
    mjr_defaultContext(&context);
    mjr_makeContext(scene.model.get(), &context, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &context);
    std::filesystem::path output = argv[3];
    std::filesystem::create_directories(output);
    std::vector<unsigned char> rgb(width * height * 3), rgba(width * height * 4);
    mjv_updateScene(scene.model.get(), scene.data.get(), &options, nullptr, &camera, mjCAT_ALL,
                    &visual);
    visual.flags[mjRND_HAZE] = 0;
    mjr_render({0, 0, width, height}, &visual, &context);
    mjr_readPixels(rgb.data(), nullptr, {0, 0, width, height}, &context);
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x) {
        size_t from = 3 * ((height - 1 - y) * width + x), to = 4 * (y * width + x);
        for (int c = 0; c < 3; ++c)
          rgba[to + c] = rgb[from + c];
        rgba[to + 3] = 255;
      }
    auto file = output / "scene.png";
    auto error = lodepng_encode32_file(file.c_str(), rgba.data(), width, height);
    if (error)
      throw std::runtime_error(lodepng_error_text(error));
    std::cout << file << '\n';
    mjr_freeContext(&context);
    mjv_freeScene(&visual);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
