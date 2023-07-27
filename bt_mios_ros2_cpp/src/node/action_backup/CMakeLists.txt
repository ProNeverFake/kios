# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(rclcpp REQUIRED)

add_executable(bt_ros2_node bt_ros2_node.cpp)

target_link_libraries(bt_ros2_node
  ${PROJECT_NAME}::behavior_tree
  behaviortree_cpp 
)

ament_target_dependencies(bt_ros2_node
  "rclcpp"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    bt_ros2_node

    DESTINATION lib/${PROJECT_NAME}
    )

