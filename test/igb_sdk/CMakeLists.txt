cmake_minimum_required(VERSION 3.15)

project(igb_sdk_test VERSION 1.0)

#enable_language(ASM)

unset(CMAKE_C_COMPILER CACHE)
set(CMAKE_C_COMPILER "/usr/local/bin/g++-14")
unset(CMAKE_CXX_COMPILER CACHE)
set(CMAKE_CXX_COMPILER "/usr/local/bin/g++-14")
unset(CMAKE_AS_COMPILER CACHE)
set(CMAKE_AS_COMPILER "/usr/local/bin/g++-14")
unset(CMAKE_EXE_LINKER CACHE)
set(CMAKE_EXE_LINKER "/usr/local/bin/g++-14")

unset(CMAKE_C_FLAGS CACHE)
set(CMAKE_C_FLAGS "-Wall -Wl,--gc-sections,-Werror=return-type -fno-exceptions -MMD")
unset(CMAKE_CXX_FLAGS CACHE)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-use-cxa-atexit -fno-exceptions -fno-rtti")

unset(CMAKE_EXE_LINKER_FLAGS CACHE)
set(CMAKE_EXE_LINKER_FLAGS "-lc -lm --no-exceptions" CACHE STRING "" FORCE)

unset(CMAKE_C_LINK_EXECUTABLE CACHE)
set(CMAKE_C_LINK_EXECUTABLE "${CMAKE_EXE_LINKER} <LINK_FLAGS> <OBJECTS> <LINK_LIBRARIES> -o <TARGET>")
unset(CMAKE_CXX_LINK_EXECUTABLE CACHE)
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_EXE_LINKER} <LINK_FLAGS> <OBJECTS> <LINK_LIBRARIES> -o <TARGET>")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS_DEBUG "-O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_subdirectory(../../igb_sdk igb_sdk)
add_subdirectory(../lib/Catch2 Catch2)

add_executable(
  igb_sdk_test
  src/test_quantizer.cpp
  src/test_button_history.cpp
  src/test_syncable_mod_clock.cpp
)
target_link_libraries(igb_sdk_test PRIVATE igb_sdk Catch2::Catch2WithMain)

#target_include_directories(igb_sdk PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
