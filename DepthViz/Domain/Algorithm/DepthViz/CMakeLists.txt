cmake_minimum_required(VERSION 3.14)
project(DV_SLAM VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Eigen (use local ThirdParty copy or system)
set(EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/Eigen")
if(EXISTS "${EIGEN_DIR}/Eigen/Core")
    message(STATUS "Using local Eigen from ${EIGEN_DIR}")
    include_directories(${EIGEN_DIR})
else()
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# PNG library (for ARKitScenes depth map loading)
find_package(PNG REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

# Core library (all .cpp sources except test/benchmark mains)
set(DV_SLAM_SOURCES
    src/DV_ESKF.cpp
    src/DV_LIOBackend.cpp
    src/DV_VIOManager.cpp
    src/DepthVizEngine.cpp
    src/DV_OfflineEvaluator.cpp
)

add_library(dv_slam STATIC ${DV_SLAM_SOURCES})
target_link_libraries(dv_slam PRIVATE PNG::PNG)

# Test runner
add_executable(dv_test_runner tests/DepthVizTestRunner.cpp)
target_link_libraries(dv_test_runner PRIVATE dv_slam)

# Benchmark CLI
add_executable(dv_benchmark tools/benchmark_main.cpp)
target_link_libraries(dv_benchmark PRIVATE dv_slam PNG::PNG)

# Install targets
install(TARGETS dv_test_runner dv_benchmark DESTINATION bin)
