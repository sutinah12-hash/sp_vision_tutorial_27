cmake_minimum_required(VERSION 3.16)

project(modern_cpp_opencv_lesson LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)
find_package(Threads REQUIRED)

function(lesson_target target source)
  add_executable(${target} ${source})
  target_compile_options(
    ${target}
    PRIVATE
      $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
      $<$<CXX_COMPILER_ID:MSVC>:/W4>
  )
endfunction()

add_subdirectory(ex1 cmake/ex1)
add_subdirectory(ex2 cmake/ex2)
add_subdirectory(ex3 cmake/ex3)
add_subdirectory(ex4 cmake/ex4)
add_subdirectory(demos cmake/demos)
