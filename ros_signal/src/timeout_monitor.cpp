#include "ros_signal/timeout_monitor.hpp"

#include <rclcpp/create_timer.hpp>


ros_signal::TimeoutMonitor::TimeoutMonitor(
        rclcpp::Node *const node,
        rclcpp::Clock::SharedPtr clock,
        const TimeoutMonitorOptions &options,
        std::function<void(const rclcpp::Time &)> state_change_cb
) : TimeoutMonitorBase(options.validate()),
    LatchHoldTimerBase(node, clock, options.latch_hold_period(), options.callback_group()),
    clk_(std::move(clock)),
    state_change_cb_(std::move(state_change_cb)) {
    timeout_timer_ = rclcpp::create_timer(
            node, clk_, options.timeout(),
            std::bind(&ros_signal::TimeoutMonitor::timeout_timer_cb, this),
            options.callback_group());
}

void ros_signal::TimeoutMonitor::signal(const rclcpp::Time &stamp) {
    timeout_timer_->reset();
    TimeoutMonitorBase::signal(stamp);
    if (error() && can_unlatch()) {
        unlatch(stamp);
    }
}

void ros_signal::TimeoutMonitor::timeout_timer_cb() {
    const auto stamp = clk_->now();
    TimeoutMonitorBase::timeout(stamp);
}
