cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)
project(BezierCurve)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories("${VCPKG_ROOT}/installed/x64-windows/include")

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES} TBB::tbb)
