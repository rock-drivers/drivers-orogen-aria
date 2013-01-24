SET(CMAKE_MODULE_PATH "$ENV{ROCK_CMAKE_MACROS}")
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
include($ENV{ROCK_CMAKE_MACROS}/RockRuby.cmake)

rock_ruby_rice_extension(mr_control mr_control.cpp)

rock_find_pkgconfig(ARIA aria) 
target_link_libraries(mr_control ${ARIA_LIBRARIES})

