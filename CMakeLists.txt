cmake_minimum_required(VERSION 3.8)
project(path_planning)

add_library(rrt src/rrt.cpp)
target_compile_features(rrt PUBLIC cxx_std_11)

add_compile_options(-ggdb)

add_executable(main src/main.cpp)
target_link_libraries(main rrt)
