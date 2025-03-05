#include <stdexcept>
#include <utility>

#include "ros_signal/timeout_monitor_options.hpp"


ros_signal::TimeoutMonitorBaseOptions::TimeoutMonitorBaseOptions() = default;

ros_signal::TimeoutMonitorBaseOptions::~TimeoutMonitorBaseOptions() = default;

const rclcpp::Duration &ros_signal::TimeoutMonitorBaseOptions::timeout() const {
    return timeout_;
}

ros_signal::TimeoutMonitorBaseOptions &ros_signal::TimeoutMonitorBaseOptions::timeout(const rclcpp::Duration &timeout) {
    timeout_ = timeout;
    return *this;
}

const ros_signal::TimeoutMonitorBaseOptions &ros_signal::TimeoutMonitorBaseOptions::validate() const {
    if (timeout_ < rclcpp::Duration::from_seconds(0)) {
        throw std::invalid_argument("timeout must be non-negative");
    }

    return *this;
}

ros_signal::TimeoutMonitorOptions::TimeoutMonitorOptions() = default;

ros_signal::TimeoutMonitorOptions::TimeoutMonitorOptions(const TimeoutMonitorBaseOptions &base_options)
    : TimeoutMonitorBaseOptions(base_options) {
}

ros_signal::TimeoutMonitorOptions::TimeoutMonitorOptions(TimeoutMonitorBaseOptions &&base_options)
    : TimeoutMonitorBaseOptions(std::forward<TimeoutMonitorBaseOptions>(base_options)) {
}

ros_signal::TimeoutMonitorOptions::~TimeoutMonitorOptions() = default;

const rclcpp::Duration &ros_signal::TimeoutMonitorOptions::latch_hold_period() const {
    return latch_hold_period_;
}

ros_signal::TimeoutMonitorOptions &ros_signal::TimeoutMonitorOptions::latch_hold_period(const rclcpp::Duration &latch_hold_period) {
    latch_hold_period_ = latch_hold_period;
    return *this;
}

const rclcpp::CallbackGroup::SharedPtr &ros_signal::TimeoutMonitorOptions::callback_group() const {
    return callback_group_;
}

ros_signal::TimeoutMonitorOptions &ros_signal::TimeoutMonitorOptions::callback_group(rclcpp::CallbackGroup::SharedPtr callback_group) {
    callback_group_ = std::move(callback_group);
    return *this;
}

const ros_signal::TimeoutMonitorOptions &ros_signal::TimeoutMonitorOptions::validate() const {
    (void) ros_signal::TimeoutMonitorBaseOptions::validate();

    if (latch_hold_period_ < rclcpp::Duration::from_seconds(0)) {
        throw std::invalid_argument("latch_hold_period must be non-negative");
    }

    return *this;
}
