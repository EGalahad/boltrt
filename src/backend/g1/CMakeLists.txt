find_path(UNITREE_INCLUDE unitree/robot/channel/channel_publisher.hpp HINTS "${UNITREE_SDK_ROOT}/include" REQUIRED NO_DEFAULT_PATH)
find_library(UNITREE_LIB unitree_sdk2 HINTS "${UNITREE_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}" REQUIRED NO_DEFAULT_PATH)
find_library(DDSC_LIB ddsc HINTS "${UNITREE_SDK_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}" REQUIRED NO_DEFAULT_PATH)
find_library(DDSCXX_LIB ddscxx HINTS "${UNITREE_SDK_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}" REQUIRED NO_DEFAULT_PATH)
add_library(boltrt_g1 backend.cpp)
# The vendor CycloneDDS headers use template destructors removed in C++20.
# Keep the SDK adapter in C++17; the runtime remains C++20.
set_target_properties(boltrt_g1 PROPERTIES CXX_STANDARD 17)
target_include_directories(boltrt_g1 PRIVATE "${UNITREE_INCLUDE}" "${UNITREE_SDK_ROOT}/thirdparty/include" "${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx")
target_link_libraries(boltrt_g1 PUBLIC boltrt_backend "${UNITREE_LIB}" "${DDSC_LIB}" "${DDSCXX_LIB}")
