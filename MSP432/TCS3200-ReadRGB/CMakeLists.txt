cmake_minimum_required(VERSION 3.18.4)

project(final)
enable_language(C CXX)

if(CMAKE_CROSSCOMPILING) 
    message(STATUS "Crosscompiling enabled, using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
    add_subdirectory(src)
else()
    message(STATUS "No crosscompiling specified, compiling in /tests only.")
    add_subdirectory(tests)
endif()

set(CMAKE_BUILD_TYPE Debug)

set(COMMON_C_FLAGS "-O0 -g3 -ggdb -std=gnu11 \
-Wall -Werror -Wstrict-prototypes \
-Wimplicit -Wcast-align -Wpointer-arith 
-ffunction-sections -fdata-sections -MD -Wundef")

set(COMMON_CXX_FLAGS "-O0 -g3 -Wall -Werror -std=c++11 -ggdb \
-fno-exceptions -fno-rtti \
-fno-threadsafe-statics -ffunction-sections -fdata-sections")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
