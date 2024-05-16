cmake_minimum_required(VERSION 3.11)
project(som_linuxcppmoteus VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
include(FetchContent)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20240430
)
FetchContent_MakeAvailable(moteus)

add_library(som_linuxcppmoteus INTERFACE)
target_compile_features(som_linuxcppmoteus INTERFACE cxx_std_17)
target_include_directories(som_linuxcppmoteus INTERFACE lib)
target_link_libraries(som_linuxcppmoteus INTERFACE nlohmann_json moteus::cpp)

if(EXISTS "/opt/vc/include/bcm_host.h")
  message("This system is a Raspberry Pi")
  FetchContent_Declare(
    pi3hat
    GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
    GIT_TAG 0.1-20231216
  )
  FetchContent_MakeAvailable(pi3hat)
  add_definitions(-D__RASPBERRY_PI__)
  target_link_libraries(som_linuxcppmoteus INTERFACE pi3hat::pi3hat)
else()
  message("This system is NOT a Raspberry Pi")
endif()

add_executable(debug examples/debug/main.cpp)
target_link_libraries(debug som_linuxcppmoteus)
set_target_properties(debug PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../examples/debug/bin"
)

add_executable(vanilla examples/vanilla/main.cpp)
target_link_libraries(vanilla som_linuxcppmoteus)
set_target_properties(vanilla PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../examples/vanilla/bin"
)

add_executable(udp examples/udp/main.cpp)
target_link_libraries(udp som_linuxcppmoteus)
set_target_properties(udp PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../examples/udp/bin"
)
