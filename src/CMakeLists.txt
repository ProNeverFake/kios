
add_subdirectory(library)
add_executable(bt_node bt_node.cpp)

ament_target_dependencies(bt_node rclcpp std_msgs rclcpp_action example_interfaces)

target_link_libraries(bt_node behaviortree_cpp)

target_include_directories(bt_node
    PRIVATE
        $<BUILD_INTERFACE:${INCLUDE_DIR}/${PROJECT_NAME}>
    )   

install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(bt_node bt_node.cpp)

# ament_target_dependencies(bt_node rclcpp std_msgs)
# target_link_libraries(bt_node behaviortree_cpp)

# install(TARGETS
#   bt_node
#   DESTINATION lib/${PROJECT_NAME}
# )

