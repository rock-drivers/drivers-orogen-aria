PROJECT(aria)
cmake_minimum_required(VERSION 2.6)

add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(ariaBase)

# Install additional Types for Aria
INSTALL(FILES AriaTypes.hpp
    DESTINATION include/aria)

add_subdirectory(ruby)
add_subdirectory(powerswitch)
# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

