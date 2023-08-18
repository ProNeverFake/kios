cmake_minimum_required(VERSION 3.8)
project(kios_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################## conan setting
# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.16.1/conan.cmake"
                  "${CMAKE_BINARY_DIR}/conan.cmake")
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(CONANFILE conanfile.txt  
                BASIC_SETUP CMAKE_TARGETS
                BUILD missing)


# set CPP standard as 17 according to package behavior_tree library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fmt")

# # set variable (TO BE REMOVED IN THE FUTURE)
# set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)


# # add static library directory (prefix) to the library finding path
# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/static-lib-dir")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(kios_interface REQUIRED)
find_package(spdlog REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(websocketpp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net)

find_package(fmt REQUIRED)

# rosidl dependencies
find_package(rosidl_default_generators REQUIRED)

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


# ########################## fetch libs
# ! DISCARDED
# include(FetchContent)

# FetchContent_Declare(
#     spdlog
#     GIT_REPOSITORY https://github.com/gabime/spdlog.git
#     GIT_TAG v1.12.0  # Adjust the version as needed
# )

# FetchContent_MakeAvailable(spdlog)

add_subdirectory(lib/BehaviorTree.CPP)
add_subdirectory(src)

################## Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
