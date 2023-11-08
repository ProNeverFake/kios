if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################################################### tree_node
add_library(tree_node SHARED tree_node.cpp)

target_include_directories(tree_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions( tree_node
  PRIVATE "BEHAVIOR_TREE_BUILDING_DLL"
)

target_link_libraries(tree_node
    ${PROJECT_NAME}::behavior_tree
    ${PROJECT_NAME}::kios_utils
    ${PROJECT_NAME}::kios_communication
)

ament_target_dependencies(tree_node
  rclcpp
  rclcpp_action
  rclcpp_components
  rcl_interfaces
  kios_interface
)

rclcpp_components_register_node(tree_node PLUGIN "kios::TreeNode" EXECUTABLE tree_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

######################################################### commander

add_library(commander SHARED commander.cpp)

target_include_directories(commander
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(commander
  PRIVATE "BEHAVIOR_TREE_BUILDING_DLL"
)

target_link_libraries(commander
    ${PROJECT_NAME}::behavior_tree
    ${PROJECT_NAME}::kios_utils
    ${PROJECT_NAME}::kios_communication
)

ament_target_dependencies(tree_node
  rclcpp
  rclcpp_action
  rclcpp_components
  rcl_interfaces
  kios_interface
)

rclcpp_components_register_node(commander PLUGIN "kios::Commander" EXECUTABLE commander)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

######################################################### tactician

add_executable(tactician tactician.cpp)

target_link_libraries(tactician
    ${PROJECT_NAME}::behavior_tree
    nlohmann_json::nlohmann_json
    ${PROJECT_NAME}::kios_utils
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

######################################################### test_node

add_executable(test_node test_node.cpp)

target_link_libraries(test_node
    ${PROJECT_NAME}::behavior_tree
    nlohmann_json::nlohmann_json
    ${PROJECT_NAME}::kios_utils
)

ament_target_dependencies(test_node
  rclcpp
  kios_interface
)


######################################################### messenger

add_executable(mongo_reader mongo_reader.cpp)

target_link_libraries(mongo_reader
    ${PROJECT_NAME}::behavior_tree
    nlohmann_json::nlohmann_json
    ${PROJECT_NAME}::kios_utils
    ${PROJECT_NAME}::kios_communication
)

ament_target_dependencies(mongo_reader
  rclcpp
  kios_interface
)


# install(TARGETS
#     commander
#     messenger
#     tactician
#     tree_node
#     mongo_reader
#     test_node

#     DESTINATION lib/${PROJECT_NAME}
#     )

install(TARGETS
    commander
    messenger
    tactician
    tree_node
    mongo_reader
    test_node

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )