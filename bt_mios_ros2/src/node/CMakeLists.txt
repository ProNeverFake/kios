if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(bt_mios_ros2_interface REQUIRED)

add_executable(bt_client bt_client.cpp)

target_link_libraries(bt_client
  ${PROJECT_NAME}::action 
)

ament_target_dependencies(bt_client
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "bt_mios_ros2_interface"
)



add_executable(bt_server bt_server.cpp)

target_link_libraries(bt_server
  ${PROJECT_NAME}::action 
)

ament_target_dependencies(bt_server
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "bt_mios_ros2_interface"
)






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    bt_server
    bt_client

    DESTINATION lib/${PROJECT_NAME}
    )

