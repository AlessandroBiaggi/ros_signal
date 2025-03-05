cmake_minimum_required(VERSION 3.8)
project(ros_signal CXX)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ros_signal_interfaces REQUIRED)

add_library(
        "${PROJECT_NAME}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/latch_hold_timer_base.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/rate_monitor.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/rate_monitor_base.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/rate_monitor_options.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/signal_monitor_options.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/timeout_monitor.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/timeout_monitor_base.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/timeout_monitor_options.cpp"
)
target_include_directories(
        "${PROJECT_NAME}" PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(
        "${PROJECT_NAME}"
        "rclcpp"
        "builtin_interfaces"
        "std_msgs"
        "ros_signal_interfaces"
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)

    ament_lint_auto_find_test_dependencies()
    ament_add_gmock("${PROJECT_NAME}_test"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/main.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/entry.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/rate_monitor_base.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/signal.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/sync_policy_epsilon_time.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/sync_policy_priority.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/traits.cpp"
    )
    target_link_libraries("${PROJECT_NAME}_test" "${PROJECT_NAME}")
endif ()

install(
        DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        DESTINATION "include/${PROJECT_NAME}"
)

install(
        TARGETS "${PROJECT_NAME}"
        EXPORT "export_${PROJECT_NAME}"
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets("export_${PROJECT_NAME}" HAS_LIBRARY_TARGET)
ament_export_dependencies("ros_signal_interfaces")


ament_package()
