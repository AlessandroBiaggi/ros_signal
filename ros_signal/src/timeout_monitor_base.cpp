#include "ros_signal/timeout_monitor_base.hpp"


ros_signal::TimeoutMonitorBase::TimeoutMonitorBase(const TimeoutMonitorBaseOptions &options)
    : timeout_monitor_options_(options),
      error_(false),
      last_signal_time_(0, 0, RCL_ROS_TIME) {
}

void ros_signal::TimeoutMonitorBase::signal(const rclcpp::Time &stamp) {
    last_signal_time_ = stamp;
}

void ros_signal::TimeoutMonitorBase::timeout(const rclcpp::Time &stamp) {
    latch(stamp);
}

void ros_signal::TimeoutMonitorBase::latch(const rclcpp::Time &stamp) {
    error_ = true;
    latch_handler(stamp);
}

void ros_signal::TimeoutMonitorBase::unlatch(const rclcpp::Time &stamp) {
    error_ = false;
    unlatch_handler(stamp);
}
