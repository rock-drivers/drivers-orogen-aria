# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(ariaTaskLib)
ADD_LIBRARY(${ARIA_TASKLIB_NAME} SHARED 
    ${ARIA_TASKLIB_SOURCES})

orogen_pkg_check_modules(odometry REQUIRED odometry)
include_directories(${odometry_INCLUDE_DIRS})

add_dependencies(${ARIA_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${ARIA_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${odometry_LIBRARIES}
    ${ARIA_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${ARIA_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${ARIA_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${ARIA_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${ARIA_TASKLIB_HEADERS}
    DESTINATION include/orocos/aria)
