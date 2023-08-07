
######################################################### ws_client library
set(MODULE_NAME ws_client)

set(${MODULE_NAME}_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ws_client/ws_client.cpp
)

set(${MODULE_NAME}_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ws_client/ws_client.hpp
)

# file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/action/*.cpp")
# file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/action/*.hpp")

add_library(${MODULE_NAME} STATIC
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )

add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

# pthread issues
set(THREADS_PREFER_PTHREAD_FLAG ON)

target_include_directories(${MODULE_NAME}
    PRIVATE
        src
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    )

# Attention: target_link_libraries() should not apply any keyword
# since no keyword is applied in ament_target_dependencies()
target_link_libraries(${MODULE_NAME}
    Threads::Threads
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
)

ament_target_dependencies(${MODULE_NAME}
    "rclcpp"
    "rclcpp_action"
    )
#########################################################################


################################################### behavior_tree library
set(MODULE_NAME behavior_tree)


file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/action_node/*.cpp")
list(APPEND ${MODULE_NAME}_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/node_list.cpp
)

file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/behavior_tree/action_node/*.hpp")
list(APPEND ${MODULE_NAME}_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/behavior_tree/node_list.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/behavior_tree/tree_map.hpp
)

add_library(${MODULE_NAME} STATIC 
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )

add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

target_include_directories(${MODULE_NAME}
    PRIVATE
        src
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    )

target_link_libraries(${MODULE_NAME} 
    PUBLIC
        fmt::fmt
        spdlog
    INTERFACE
        behaviortree_cpp
    )

########## copy the xml tree file into the corresponding dir
# file(GLOB FILENAMES "${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/xml/*.xml")

# foreach(FILENAME ${FILENAMES})
#     message(STATUS "FILENAME: ${FILENAME}\n")
#     message(STATUS "copied to: ${CMAKE_CURRENT_BINARY_DIR}/xml/\n")
#     configure_file(${FILENAME} ${CMAKE_CURRENT_BINARY_DIR}/xml/ COPYONLY) 

# endforeach(FILENAME ${FILENAMES})
##########

######################################################################

# install(TARGETS
#     # mios_node
#     bt_node
#     DESTINATION lib/${PROJECT_NAME}
#     )
