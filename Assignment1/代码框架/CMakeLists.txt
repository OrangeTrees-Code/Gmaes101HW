cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)
project(Rasterizer)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

inClude_directories("D:/vcpkg/installed/x64-windows/include")

target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
