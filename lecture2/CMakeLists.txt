cmake_minimum_required(VERSION 3.16)

project(LECTURE_2)

include_directories(part_1/includes part_2/ex3)

add_executable(cmake_learn
    part_1/src/cmake_learn.cpp
    part_1/tools/hello.cpp
)

add_executable(ex1 part_2/ex1/example1.cpp)
add_executable(ex2 part_2/ex2/example2.cpp)
add_executable(ex3 part_2/ex3/car.cpp part_2/ex3/ex3.cpp)
find_package(OpenCV REQUIRED) 
