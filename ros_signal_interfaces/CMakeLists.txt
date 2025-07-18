cmake_minimum_required(VERSION 3.8)
project(ros_signal_interfaces)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces("${PROJECT_NAME}"
        # sync policy states
        "msg/EpsilonTimeState.msg"
        "msg/PriorityState.msg"
        # monitor states
        "msg/MonitorState.msg"
        "msg/RateMonitorState.msg"
        "msg/SignalMonitorState.msg"
        "msg/TimeoutMonitorState.msg"
        DEPENDENCIES
        "std_msgs"
        "builtin_interfaces"
)

ament_export_dependencies("rosidl_default_runtime")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
