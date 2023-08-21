
######################################################### kios_communication library
set(MODULE_NAME kios_communication)

file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/kios_communication/*.cpp")
file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/kios_communication/*.hpp")

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
    spdlog::spdlog
    Threads::Threads
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
    Poco::Net
)

#########################################################################


################################################### behavior_tree library
set(MODULE_NAME behavior_tree)


file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/action_node/*.cpp")
list(APPEND ${MODULE_NAME}_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/tree_root.cpp
)

file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/behavior_tree/action_node/*.hpp")
list(APPEND ${MODULE_NAME}_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/behavior_tree/tree_root.hpp
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
        ${PROJECT_NAME}::kios_utils
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

######################################################### kios_utils library
set(MODULE_NAME kios_utils)

file(GLOB ${MODULE_NAME}_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/kios_utils/*.cpp")

file(GLOB ${MODULE_NAME}_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/kios_utils/*.hpp")

# DISCARDED
# set(${MODULE_NAME}_SRC
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/kios_utils/kios_utils.cpp
# )

# set(${MODULE_NAME}_HEADER_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/kios_utils/kios_utils.hpp
# )

add_library(${MODULE_NAME} STATIC
    ${${MODULE_NAME}_SRC}
    ${${MODULE_NAME}_HEADER_FILES}
    )

target_include_directories(${MODULE_NAME}
PRIVATE
    src
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

add_library(${PROJECT_NAME}::${MODULE_NAME} ALIAS ${MODULE_NAME})

######################################################################

# install(TARGETS
#     # mios_node
#     bt_node
#     DESTINATION lib/${PROJECT_NAME}
#     )
