######################### action library
set(MODULE_NAME action)

set(${MODULE_NAME}_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/action/bt_action_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/action/bt_action_server.cpp
)

set(${MODULE_NAME}_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/action/bt_action_client.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/action/bt_action_server.hpp
)

# file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/action/*.cpp")
# file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/action/*.hpp")

add_library(${MODULE_NAME}
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )

add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

find_package(example_interfaces REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(bt_mios_ros2_interface REQUIRED)

# message(STATE "SOURCE DIR IS ${CMAKE_SOURCE_DIR}\n")
# message(STATE "CMAKE INSTALL PREFIX IS ${CMAKE_INSTALL_PREFIX}\n")

target_include_directories(${MODULE_NAME}
    PRIVATE
    src
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    )


############## Self defined interface setting
# rosidl_get_typesupport_target(cpp_typesupport_target
#     ${PROJECT_NAME} rosidl_typesupport_cpp)

# target_link_libraries(${MODULE_NAME} "${cpp_typesupport_target}")

ament_target_dependencies(${MODULE_NAME}
    "rclcpp"
    "rclcpp_action"
    "example_interfaces"
    "action_tutorials_interfaces"
    "bt_mios_ros2_interface"
    )


# target_link_libraries(${MODULE_NAME}
#     PRIVATE
#         rclcpp
#         rclcpp_action
#         example_interfaces
#     )


################### nodes
# add_executable(bt_node src/node/bt_node.cpp)

# ament_target_dependencies(bt_node rclcpp std_msgs rclcpp_action example_interfaces)

# target_link_libraries(bt_node behaviortree_cpp)

# target_include_directories(bt_node
#     PRIVATE
#         $<BUILD_INTERFACE:${INCLUDE_DIR}/${PROJECT_NAME}>
#     ) 

# # add_executable(mios_node src/node/mios_node.cpp)

# # ament_target_dependencies(mios_node rclcpp std_msgs rclcpp_action example_interfaces)

# # target_include_directories(mios_node
# #     PRIVATE
# #         $<BUILD_INTERFACE:${INCLUDE_DIR}/${PROJECT_NAME}>
# #     )   
    

# install(TARGETS
#     # mios_node
#     bt_node
#     DESTINATION lib/${PROJECT_NAME}
#     )
