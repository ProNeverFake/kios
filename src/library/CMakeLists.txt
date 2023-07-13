set(MODULE_NAME actions)

file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/action/*.cpp")
file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/action/*.hpp")

add_library(${MODULE_NAME}
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )

add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

find_package(example_interfaces REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)


# message(STATE "SOURCE DIR IS ${CMAKE_SOURCE_DIR}\n")

# message(STATE "CMAKE INSTALL PREFIX IS ${CMAKE_INSTALL_PREFIX}\n")


target_include_directories(${MODULE_NAME}
    PRIVATE
    src
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>

    )

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${MODULE_NAME} "${cpp_typesupport_target}")

ament_target_dependencies(${MODULE_NAME}
    "rclcpp"
    "rclcpp_action"
    "example_interfaces"
    "action_tutorials_interfaces"
    )


# target_link_libraries(${MODULE_NAME}
#     PRIVATE
#         rclcpp
#         rclcpp_action
#         example_interfaces
#     )


