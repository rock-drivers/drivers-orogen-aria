find_package(Rock)
include(RockRuby)
rock_ruby_rice_extension(aria AriaInterface.cpp)

rock_find_pkgconfig(ARIA aria) 
target_link_libraries(aria ${ARIA_LIBRARIES})

