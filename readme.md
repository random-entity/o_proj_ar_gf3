# A simple user-friendly C++ API to use mjbots moteus servo controller on Linux (including Raspberry Pi)

This repository is a library that provides an easy-to-use C++ API to use mjbots moteus servo controller on your Linux system.

## How to use

### Import library to your system

- You can use CMake `FetchContent` module to import the library.
    ```cmake
    include(FetchContent)

    FetchContent_Declare(
      som_linuxcppmoteus
      GIT_REPOSITORY https://github.com/seoul-open-media/linux-cpp-moteus.git
      GIT_TAG        main
    )
    FetchContent_MakeAvailable(som_linuxcppmoteus)

    add_executable(myproject myproject.cpp)
    target_link_libraries(myproject som_linuxcppmoteus)
    ```
- Or you can just (fork and) clone the git repository to modify content by yourself.

### Write your code

1. Include the library header file. Write `#include "som_linuxcppmoteus.hpp"` on the top of your code.
2. Import namespace `som` if you don't want to write `som::` everytime you use entities defined in the library. Write `using namespace som;` on the top of your code after the `#include`'s.
3. Initialize a servo system using the `ServoSystem` class constructor, where you want to use it (probably in `main`).
    ```c++
    ServoSystem servo_system{
      {{1, 1}, {2, 1}, {3, 1}} // a map that maps the IDs of the controllers to the bus numbers they are connected to
      "../config", // the path to the directory where your `slcm.positionmode.config.json` file is located, relative to the executable
      true // whether to enable external input
    };
    ```
    - The `slcm.positionmode.config.json` file will be used to configure the initial values for each command type. If the `ServoSytem` can't find it, it will use the default values.
4. Get IDs of the servos that are successfully initialized.
    ```c++
    const auto& ids = servo_system.GetIds();
    ```
5. Set the listening mode. `ListeningMode::INTERNAL` means that you want the servo system to listen to what you write in your code. `ListeningMode::EXTERNAL` means that the servo system will listen to external input that comes from outside of your program. By default, the servo system will take inputs from standard input in this mode. However, you can modify this behavior by creating a child class and overriding the `ExternalInputGetter` method with your desired logic.
6. In `INTERNAL` listening mode, you can now control your servos using the `Input`, `InputAll`, `Stop`, `StopAll`, `SetBasePositions`, `SetBasePositionsAll` methods.
    - `void Input(std::map<int, std::map<CommandType, double>>)`
        - This is the most basic input you can send to the servos. Just pass a map of `id -> command_type -> value` as the argument. For example if you want servo of ID `1` to move to position `1.0` with velocity `0.1`, and servo of ID `2` to move to position `2.0` with velocity `0.2`, you can write as follows:
            ```c++
            servo_system.Input(
              {1, {CommandType::POSITION, 1.0,
                   CommandType::VELOCITY, 0.1}},
              {2, {CommandType::POSITION, 2.0,
                   CommandType::VELOCITY, 0.2}}
            );
            ```
    - `void Input(std::map<CommandType, double>)`
        - You can use this method if you want to input the same command to all servos. Just pass a map of `command_type -> value` as the argument. For example if you want all servos to move to position `0.0` with velocity `1.0`, you can write as follows:
            ```c++
            servo_system.InputAll(
              {CommandType::POSITION, 0.0,
               CommandType::VELOCITY, 1.0}
            );
            ```
    - `void Stop(std::vector<int> ids)`
        - Use this command to stop some of the servos. Just pass a vector of the IDs of the servos you want to stop as the argument.
    - `void StopAll()`
        - Use this command to stop all the servos.
    - `void SetBasePositions(std::vector<int> ids)`
        - The value you input for the `CommandType::POSITION` is the relative position to each servo's base position, which is already set as the initial servo position at `ServoSystem` construction time. However, if you want to set the base positions to the current positions at desired time during runtime, you can use this method. Just pass a vector of the IDs of the servos you want to set base positions as the argument.
    - `void SetBasePositionsAll()`
        - Use this method to set base positions for all servos.
    - Be mindful that once a command is input to the servo system, the same command will be keep sent to the servos at 100Hz rate until you send another input and override the command values.