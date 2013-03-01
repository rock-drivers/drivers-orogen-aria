SET(CMAKE_MODULE_PATH "$ENV{ROCK_CMAKE_MACROS}")
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
include($ENV{ROCK_CMAKE_MACROS}/RockRuby.cmake)

rock_ruby_rice_extension(aria AriaInterface.cpp)

rock_find_pkgconfig(ARIA aria) 
target_link_libraries(aria ${ARIA_LIBRARIES})

