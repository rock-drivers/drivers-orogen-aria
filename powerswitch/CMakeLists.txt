rock_executable(seekurPower powerswitch.cpp)
TARGET_LINK_LIBRARIES(seekurPower ${aria_LIBRARIES})

INSTALL(TARGETS seekurPower DESTINATION bin)
