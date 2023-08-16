cmake_minimum_required(VERSION 3.8)
project(kios_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

################################### kios_cli_srv
set(MODULE_NAME kios_cli_srv)

file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/srv/kios_cli/*.srv")

set(${MODULE_NAME}_FILE_NAMES)

foreach(file ${${MODULE_NAME}_FILES})
    get_filename_component(file_name ${file} NAME)
    list(APPEND ${MODULE_NAME}_FILE_NAMES "srv/kios_cli/${file_name}")
endforeach()

################################### kios_srv
set(MODULE_NAME kios_srv)

file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/srv/kios/*.srv")

set(${MODULE_NAME}_FILE_NAMES)

foreach(file ${${MODULE_NAME}_FILES})
    get_filename_component(file_name ${file} NAME)
    list(APPEND ${MODULE_NAME}_FILE_NAMES "srv/kios/${file_name}")
endforeach()

# message("The ${MODULE_NAME}_FILE_NAMES is ${${MODULE_NAME}_FILE_NAMES}\n")

# ################################### mios_service
# set(MODULE_NAME mios_srv)

# file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/srv/mios/*.srv")
# # message("the result of glob: ${${MODULE_NAME}_FILES}\n")

# set(${MODULE_NAME}_FILE_NAMES)

# foreach(file ${${MODULE_NAME}_FILES})
#     get_filename_component(file_name ${file} NAME)
#     list(APPEND ${MODULE_NAME}_FILE_NAMES "srv/mios/${file_name}")
# endforeach()

# message("The ${MODULE_NAME}_FILE_NAMES is ${${MODULE_NAME}_FILE_NAMES}\n")

################################### kios_msg
set(MODULE_NAME kios_msg)

file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/kios/*.msg")

set(${MODULE_NAME}_FILE_NAMES)

foreach(file ${${MODULE_NAME}_FILES})
    get_filename_component(file_name ${file} NAME)
    list(APPEND ${MODULE_NAME}_FILE_NAMES "msg/kios/${file_name}")
endforeach()

# message("The ${MODULE_NAME}_FILE_NAMES is ${${MODULE_NAME}_FILE_NAMES}\n")

################################### mios_msg
set(MODULE_NAME mios_msg)

file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/mios/*.msg")

set(${MODULE_NAME}_FILE_NAMES)

foreach(file ${${MODULE_NAME}_FILES})
    get_filename_component(file_name ${file} NAME)
    list(APPEND ${MODULE_NAME}_FILE_NAMES "msg/mios/${file_name}")
endforeach()

# message("The ${MODULE_NAME}_FILE_NAMES is ${${MODULE_NAME}_FILE_NAMES}\n")

############################ action
set(MODULE_NAME action)

file(GLOB ${MODULE_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/action/*.action")
# message("the result of glob: ${${MODULE_NAME}_FILES}\n")

set(${MODULE_NAME}_FILE_NAMES)

foreach(file ${${MODULE_NAME}_FILES})
    get_filename_component(file_name ${file} NAME)
    list(APPEND ${MODULE_NAME}_FILE_NAMES "action/${file_name}")
endforeach()

message("The ${MODULE_NAME}_FILE_NAMES is ${${MODULE_NAME}_FILE_NAMES}\n")


################################ generate the interface
rosidl_generate_interfaces(${PROJECT_NAME}
      ${kios_cli_srv_FILE_NAMES}
      ${kios_srv_FILE_NAMES}      
      ${mios_srv_FILE_NAMES}
      ${action_FILE_NAMES}
      ${mios_msg_FILE_NAMES}
      ${kios_msg_FILE_NAMES}
    )

ament_package()
