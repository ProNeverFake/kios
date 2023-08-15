if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
    ${PROJECT_NAME}::behavior_tree
    ${PROJECT_NAME}::kios_utils
)

ament_target_dependencies(commander
  rclcpp
  rcl_interfaces
  kios_interface
)

######################################################### tactician

add_executable(tactician tactician.cpp)

target_link_libraries(tactician
    ${PROJECT_NAME}::behavior_tree
    nlohmann_json::nlohmann_json
)

ament_target_dependencies(tactician
  rclcpp
  kios_interface
)

######################################################### messenger

add_executable(messenger messenger.cpp)

target_link_libraries(messenger
    ${PROJECT_NAME}::behavior_tree
    nlohmann_json::nlohmann_json
    ${PROJECT_NAME}::kios_utils
)

ament_target_dependencies(messenger
  rclcpp
  kios_interface
)



install(TARGETS
    # robot_state_subscription
    commander
    messenger
    tactician
    tree_node

    DESTINATION lib/${PROJECT_NAME}
    )

