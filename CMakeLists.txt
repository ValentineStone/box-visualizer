cmake_minimum_required(VERSION 3.5)

project(box-visualizer VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
include_directories(${OpenMP_CXX_INCLUDE_DIRS})

set(PROJECT_SOURCES
  main.cpp
)

add_executable(box-visualizer
  ${PROJECT_SOURCES}
)

target_link_libraries(box-visualizer PRIVATE
  ${OpenCV_LIBS}
  ${OpenMP_CXX_LIBRARIES}
  realsense2
)

target_compile_options(box-visualizer PRIVATE ${OpenMP_CXX_FLAGS})