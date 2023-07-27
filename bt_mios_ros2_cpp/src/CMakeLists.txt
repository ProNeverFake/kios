add_subdirectory(library)

add_subdirectory(node)

# set(NODE_NAME
#     bt_action_client
#     bt_action_server
#     bt_node
#     mios_node
# )

# find_package(example_interfaces REQUIRED)
# find_package(action_tutorials_interfaces REQUIRED)

# foreach(node_name ${NODE_NAME})
#   add_executable(${node_name} ${node_name}.cpp)
#   ament_target_dependencies(${node_name} rclcpp std_msgs rclcpp_action example_interfaces)
# endforeach()

# target_link_libraries(bt_node behaviortree_cpp)


# target_include_directories(bt_node
#     PRIVATE
#         $<BUILD_INTERFACE:${INCLUDE_DIR}/${PROJECT_NAME}>
#     )  

# install(TARGETS
#     bt_node
#     DESTINATION lib/${PROJECT_NAME}
#     )