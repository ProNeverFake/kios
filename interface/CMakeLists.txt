find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/CommandTest.action"
    DEPENDENCIES 
        action_msgs
)