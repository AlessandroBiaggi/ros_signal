#include <stdexcept>
#include <utility>

#include "ros_signal/signal_monitor_options.hpp"


ros_signal::SignalMonitorBaseOptions::SignalMonitorBaseOptions() = default;

ros_signal::SignalMonitorBaseOptions::~SignalMonitorBaseOptions() = default;

const ros_signal::SignalMonitorBaseOptions &ros_signal::SignalMonitorBaseOptions::validate() const {
    return *this;
}

ros_signal::SignalMonitorOptions::SignalMonitorOptions() = default;

ros_signal::SignalMonitorOptions::SignalMonitorOptions(const ros_signal::SignalMonitorBaseOptions &base_options)
    : ros_signal::SignalMonitorBaseOptions(base_options) {
}

ros_signal::SignalMonitorOptions::SignalMonitorOptions(ros_signal::SignalMonitorBaseOptions &&base_options)
    : ros_signal::SignalMonitorBaseOptions(std::forward<ros_signal::SignalMonitorBaseOptions>(base_options)) {
}

ros_signal::SignalMonitorOptions::~SignalMonitorOptions() = default;

const rclcpp::Duration &ros_signal::SignalMonitorOptions::latch_hold_period() const {
    return latch_hold_period_;
}

ros_signal::SignalMonitorOptions &ros_signal::SignalMonitorOptions::latch_hold_period(
    const rclcpp::Duration &latch_hold_period) {
    latch_hold_period_ = latch_hold_period;
    return *this;
}

const rclcpp::CallbackGroup::SharedPtr &ros_signal::SignalMonitorOptions::callback_group() const {
    return callback_group_;
}

ros_signal::SignalMonitorOptions &ros_signal::SignalMonitorOptions::callback_group(
    rclcpp::CallbackGroup::SharedPtr callback_group) {
    callback_group_ = std::move(callback_group);
    return *this;
}

const ros_signal::SignalMonitorOptions &ros_signal::SignalMonitorOptions::validate() const {
    (void) ros_signal::SignalMonitorBaseOptions::validate();

    if (latch_hold_period_ < rclcpp::Duration::from_seconds(0)) {
        throw std::invalid_argument("latch_hold_period must be non-negative");
    }

    return *this;
}
