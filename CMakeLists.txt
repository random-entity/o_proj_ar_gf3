cmake_minimum_required(VERSION 3.11)
project(gf3 VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(gf3 src/gf3/main.cpp)
target_compile_features(gf3 PUBLIC cxx_std_17)
target_include_directories(gf3 PUBLIC src)
set_target_properties(gf3 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../bin"
)

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
  GIT_TAG 0.1-20240617
)
FetchContent_MakeAvailable(moteus)

target_link_libraries(gf3 nlohmann_json moteus::cpp)

if(EXISTS "/usr/include/bcm_host.h")
  message("This system is a Raspberry Pi")
  FetchContent_Declare(
    pi3hat
    GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
    GIT_TAG 0.1-20231216
  )
  set(FETCHCONTENT_SOURCE_DIR_MOTEUS ${MOTEUS_SOURCE_DIR})
  FetchContent_MakeAvailable(pi3hat)
  add_definitions(-D__RASPBERRY_PI__)
  target_link_libraries(gf3 pi3hat::pi3hat)
else()
  message("This system is NOT a Raspberry Pi")
endif()
