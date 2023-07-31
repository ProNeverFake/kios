if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_executable(bt_client bt_client.cpp)

# target_link_libraries(bt_client
#   ${PROJECT_NAME}::action 
# )

# ament_target_dependencies(bt_client
#   "rclcpp"
#   "rclcpp_action"
#   "example_interfaces"
#   "bt_mios_ros2_interface"
# )



# add_executable(bt_server bt_server.cpp)

# target_link_libraries(bt_server
#   ${PROJECT_NAME}::action 
# )

# ament_target_dependencies(bt_server
#   "rclcpp"
#   "rclcpp_action"
#   "example_interfaces"
#   "bt_mios_ros2_interface"
# )

add_executable(bt_ros2_node bt_ros2_node.cpp)


target_link_libraries(bt_ros2_node
    ${PROJECT_NAME}::behavior_tree
    ${PROJECT_NAME}::ws_client
)

ament_target_dependencies(bt_ros2_node
  rclcpp
  rcl_interfaces
  bt_mios_ros2_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    # bt_server
    # bt_client
    bt_ros2_node

    DESTINATION lib/${PROJECT_NAME}
    )

