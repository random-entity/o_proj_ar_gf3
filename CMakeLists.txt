cmake_minimum_required(VERSION 3.11)
project(som_linuxcppmoteus VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
include(FetchContent)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG main
)
FetchContent_MakeAvailable(moteus)

add_library(som_linuxcppmoteus INTERFACE)
target_compile_features(som_linuxcppmoteus INTERFACE cxx_std_17)
target_include_directories(som_linuxcppmoteus INTERFACE lib)
target_link_libraries(som_linuxcppmoteus INTERFACE moteus::cpp)
if(EXISTS "/opt/vc/include/bcm_host.h")
  message("IS RASPBERRY PI")
  FetchContent_Declare(
    pi3hat
    GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(pi3hat)
  add_definitions(-D__RASPBERRY_PI__)
  target_link_libraries(som_linuxcppmoteus INTERFACE pi3hat::pi3hat)
else()
  message("NOT RASPBERRY PI")
endif()

add_executable(simple example/simple.cpp)
target_link_libraries(simple som_linuxcppmoteus)
