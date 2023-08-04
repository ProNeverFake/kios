if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################################################### bt_ros2_node
###################### as template ###################################

# add_executable(bt_ros2_node bt_ros2_node.cpp)

# target_link_libraries(bt_ros2_node
#     ${PROJECT_NAME}::behavior_tree
#     ${PROJECT_NAME}::ws_client
# )

# ament_target_dependencies(bt_ros2_node
#   rclcpp
#   rcl_interfaces
#   bt_mios_ros2_interface
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ######################################################### test node

# add_executable(test_node_cpp test_node_cpp.cpp)

# target_link_libraries(test_node_cpp
#     ${PROJECT_NAME}::behavior_tree
#     ${PROJECT_NAME}::ws_client
# )

# ament_target_dependencies(test_node_cpp
#   rclcpp
#   rcl_interfaces
#   bt_mios_ros2_interface
#   example_interfaces
# )

######################################################### robot_state_subscription

add_executable(robot_state_subscription robot_state_subscription.cpp)

target_link_libraries(robot_state_subscription
    ${PROJECT_NAME}::ws_client
)

ament_target_dependencies(robot_state_subscription
  rclcpp
  rcl_interfaces
  bt_mios_ros2_interface
)

# ######################################################### tree_node
# add_executable(tree_node tree_node.cpp)

# target_link_libraries(tree_node
#     ${PROJECT_NAME}::behavior_tree
#     ${PROJECT_NAME}::ws_client
# )

# ament_target_dependencies(tree_node
#   rclcpp
#   rcl_interfaces
#   bt_mios_ros2_interface
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

######################################################### callback example node

add_executable(callback_example callback_example.cpp)

ament_target_dependencies(callback_example
  rclcpp
  rcl_interfaces
  example_interfaces
  std_srvs
)

######################################################### commander

add_executable(commander commander.cpp)

target_link_libraries(commander
    ${PROJECT_NAME}::ws_client
)

ament_target_dependencies(commander
  rclcpp
  rcl_interfaces
  example_interfaces
  std_srvs
)



install(TARGETS
    # bt_ros2_node
    # test_node_cpp
    # tree_node
    callback_example
    robot_state_subscription
    commander

    DESTINATION lib/${PROJECT_NAME}
    )

