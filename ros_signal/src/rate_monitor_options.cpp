#include <stdexcept>
#include <utility>

#include "ros_signal/rate_monitor_options.hpp"


ros_signal::RateMonitorBaseOptions::RateMonitorBaseOptions() = default;

ros_signal::RateMonitorBaseOptions::~RateMonitorBaseOptions() = default;

const rclcpp::Duration &ros_signal::RateMonitorBaseOptions::window() const {
    return window_;
}

ros_signal::RateMonitorBaseOptions &ros_signal::RateMonitorBaseOptions::window(const rclcpp::Duration &window) {
    window_ = window;
    return *this;
}

float ros_signal::RateMonitorBaseOptions::f_nominal() const {
    return f_nominal_;
}

ros_signal::RateMonitorBaseOptions &ros_signal::RateMonitorBaseOptions::f_nominal(const float f_nominal) {
    f_nominal_ = f_nominal;
    return *this;
}

float ros_signal::RateMonitorBaseOptions::f_min() const {
    return f_min_;
}

ros_signal::RateMonitorBaseOptions &ros_signal::RateMonitorBaseOptions::f_min(const float f_min) {
    f_min_ = f_min;
    return *this;
}

float ros_signal::RateMonitorBaseOptions::f_max() const {
    return f_max_;
}

ros_signal::RateMonitorBaseOptions &ros_signal::RateMonitorBaseOptions::f_max(const float f_max) {
    f_max_ = f_max;
    return *this;
}


const ros_signal::RateMonitorBaseOptions &ros_signal::RateMonitorBaseOptions::validate() const {
    if (window_.seconds() <= 0) {
        throw std::invalid_argument("window must be positive");
    }
    if (f_min_ < 0) {
        throw std::invalid_argument("f_min must be non-negative");
    }
    if (f_nominal_ < f_min_) {
        throw std::invalid_argument("f_nominal must be greater than or equal to f_min");
    }
    if (f_max_ < f_nominal_) {
        throw std::invalid_argument("f_max must be greater than or equal to f_nominal");
    }

    return *this;
}

ros_signal::RateMonitorOptions::RateMonitorOptions() = default;

ros_signal::RateMonitorOptions::RateMonitorOptions(const ros_signal::RateMonitorBaseOptions &base_options)
    : ros_signal::RateMonitorBaseOptions(base_options) {
}

ros_signal::RateMonitorOptions::RateMonitorOptions(ros_signal::RateMonitorBaseOptions &&base_options)
    : ros_signal::RateMonitorBaseOptions(std::forward<ros_signal::RateMonitorBaseOptions>(base_options)) {
}

ros_signal::RateMonitorOptions::~RateMonitorOptions() = default;

const rclcpp::Duration &ros_signal::RateMonitorOptions::latch_hold_period() const {
    return latch_hold_period_;
}

ros_signal::RateMonitorOptions &
ros_signal::RateMonitorOptions::latch_hold_period(const rclcpp::Duration &latch_hold_period) {
    latch_hold_period_ = latch_hold_period;
    return *this;
}

const rclcpp::CallbackGroup::SharedPtr &ros_signal::RateMonitorOptions::callback_group() const {
    return callback_group_;
}

ros_signal::RateMonitorOptions &
ros_signal::RateMonitorOptions::callback_group(rclcpp::CallbackGroup::SharedPtr callback_group) {
    callback_group_ = std::move(callback_group);
    return *this;
}

const ros_signal::RateMonitorOptions &ros_signal::RateMonitorOptions::validate() const {
    (void) ros_signal::RateMonitorBaseOptions::validate();

    if (latch_hold_period_.seconds() < 0) {
        throw std::invalid_argument("latch_hold_period must be non-negative");
    }

    if (callback_group_ != nullptr) {
        if (callback_group_->type() != rclcpp::CallbackGroupType::MutuallyExclusive) {
            throw std::invalid_argument("callback_group must be of type MutuallyExclusive");
        }
    }

    return *this;
}
