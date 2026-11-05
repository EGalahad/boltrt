add_library(boltrt_track_motion motion/file.cpp motion/pose.cpp motion/zmq.cpp motion/smpl.cpp)
target_include_directories(boltrt_track_motion PRIVATE "${PROJECT_SOURCE_DIR}/third_party/include")
target_link_libraries(boltrt_track_motion PUBLIC boltrt_mujoco_util ZLIB::ZLIB "${ZMQ_LIB}")
add_library(boltrt_track_observation
  observation/registry.cpp observation/proprioception.cpp observation/mimic_lite.cpp
  observation/bfm_zero.cpp observation/grit.cpp observation/heft.cpp observation/holomotion.cpp
  observation/humanoid_gpt.cpp observation/scalebfm.cpp observation/sonic.cpp
  observation/teleopit.cpp observation/twist2.cpp)
target_link_libraries(boltrt_track_observation PUBLIC boltrt_mujoco_util)
add_library(boltrt_track loop.cpp register.cpp)
target_link_libraries(boltrt_track PUBLIC boltrt_core boltrt_track_motion boltrt_track_observation)
