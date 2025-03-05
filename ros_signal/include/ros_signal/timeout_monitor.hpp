#ifndef ROS_SIGNAL_TIMEOUT_MONITOR_HPP
#define ROS_SIGNAL_TIMEOUT_MONITOR_HPP

#include <rclcpp/clock.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include "ros_signal/detail/latch_hold_timer_base.hpp"
#include "ros_signal/timeout_monitor_base.hpp"
#include "ros_signal/timeout_monitor_options.hpp"


namespace ros_signal {
    class TimeoutMonitor : public TimeoutMonitorBase, protected detail::LatchHoldTimerBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(TimeoutMonitor)

        TimeoutMonitor(
                rclcpp::Node *node,
                rclcpp::Clock::SharedPtr clock,
                const TimeoutMonitorOptions &options,
                std::function<void(const rclcpp::Time &)> state_change_cb);

        void signal(const rclcpp::Time &stamp) override;

    private:
        void latch_handler(const rclcpp::Time &stamp) override {
            set_cannot_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        void unlatch_handler(const rclcpp::Time &stamp) override {
            set_can_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        void timeout_timer_cb();

        rclcpp::Clock::SharedPtr clk_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        std::function<void(const rclcpp::Time &)> state_change_cb_;
    };
}// namespace ros_signal

#endif// ROS_SIGNAL_TIMEOUT_MONITOR_HPP
