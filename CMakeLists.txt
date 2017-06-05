cmake_minimum_required(VERSION 2.8.9)
project(libsbp_example)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

add_executable(libsbp_example example.c)

find_package(PkgConfig)
pkg_check_modules(LIBSERIALPORT libserialport)
link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")
include_directories("/usr/local/include/")

add_library(callback callback.c)
target_link_libraries(callback sbp)
target_link_libraries(libsbp_example callback serialport sbp)
