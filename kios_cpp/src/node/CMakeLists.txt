if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


######################################################### robot_state_subscription

add_executable(robot_state_subscription robot_state_subscription.cpp)

target_link_libraries(robot_state_subscription
    ${PROJECT_NAME}::ws_client
)

ament_target_dependencies(robot_state_subscription
  rclcpp
  rcl_interfaces
  kios_interface
)

######################################################### tree_node
add_executable(tree_node tree_node.cpp)

target_link_libraries(tree_node
    ${PROJECT_NAME}::behavior_tree
)

ament_target_dependencies(tree_node
  rclcpp
  rcl_interfaces
  kios_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

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
  kios_interface
)



install(TARGETS
    # test_node_cpp
    tree_node
    robot_state_subscription
    commander

    DESTINATION lib/${PROJECT_NAME}
    )

