target_sources(boltrt_backend PRIVATE scene.cpp)
if(BOLTRT_VIEWER)
  find_path(GLFW_INCLUDE GLFW/glfw3.h HINTS "${PROJECT_SOURCE_DIR}/third_party/include" REQUIRED)
  find_library(GLFW_LIB NAMES glfw glfw3 libglfw.so.3 REQUIRED)
  find_package(OpenGL REQUIRED)
  add_library(mujoco_simulate STATIC
    "${MUJOCO_ROOT}/simulate/simulate.cc"
    "${MUJOCO_ROOT}/simulate/platform_ui_adapter.cc"
    "${MUJOCO_ROOT}/simulate/glfw_adapter.cc"
    "${MUJOCO_ROOT}/simulate/glfw_dispatch.cc"
    "${PROJECT_SOURCE_DIR}/third_party/lodepng/lodepng.cpp")
  target_include_directories(mujoco_simulate PUBLIC "${MUJOCO_ROOT}/simulate" "${MUJOCO_INCLUDE}" "${GLFW_INCLUDE}" PRIVATE "${PROJECT_SOURCE_DIR}/third_party/lodepng")
  target_link_libraries(mujoco_simulate PUBLIC "${MUJOCO_LIB}" "${GLFW_LIB}" OpenGL::GL ${CMAKE_DL_LIBS} Threads::Threads)
  target_sources(boltrt_backend PRIVATE viewer.cpp)
  target_compile_definitions(boltrt_backend PRIVATE BOLTRT_VIEWER)
  target_link_libraries(boltrt_backend PRIVATE mujoco_simulate)
  add_executable(render_scene EXCLUDE_FROM_ALL render_scene.cpp)
  target_include_directories(render_scene PRIVATE "${PROJECT_SOURCE_DIR}/third_party/lodepng" "${PROJECT_SOURCE_DIR}/third_party/include")
  target_link_libraries(render_scene PRIVATE boltrt_backend mujoco_simulate)
  set_target_properties(render_scene PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()
