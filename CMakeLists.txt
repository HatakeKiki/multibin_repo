
cmake_minimum_required(VERSION 2.8)
project(multibin_repro) 
find_package(OpenCV REQUIRED)
add_compile_options(-O3)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  )
add_executable(
  ${PROJECT_NAME}
  main.cpp 
  )
target_link_libraries(
  ${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
)
