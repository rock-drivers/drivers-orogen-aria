PROJECT(aria)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(ariaBase)

# Install additional Types for Aria
INSTALL(FILES AriaTypes.hpp
    DESTINATION include/aria)

include($ENV{ROCK_CMAKE_MACROS}/RockRuby.cmake)
add_subdirectory(ruby)
# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

