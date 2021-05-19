# TCS3200 on MSP432 Launchpad

# Build Instructions
Before running CMake, it is necessary to run the following commands:
```shell
export CC=path/to/arm-none-eabi-gcc
export CXX=path/to/arm-none-eabi-g++
```
This will set the path to the ARM cross compilers and set them to CMake cache. 

# Building on Linux
```shell
cd ./build/
cmake -DCMAKE_TOOLCHAIN_FILE=C:/Users/alan/Desktop/lab/Sensors/ColorSensor-TCS3200/MSP432/TCS3200-ReadRGB/arm-none-eabi-toolchain.cmake ..

make
```

# Building on Windows (calling make in git bash sometimes fails, so try VSCode terminal instead.)
```sh
cd build
cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=C:/Users/alan/Desktop/lab/Sensors/ColorSensor-TCS3200/MSP432/TCS3200-ReadRGB/arm-none-eabi-toolchain.cmake ..

make
```


