#ifndef ROS_SIGNAL_DETAIL_LATCH_HOLD_TIMER_BASE_HPP
#define ROS_SIGNAL_DETAIL_LATCH_HOLD_TIMER_BASE_HPP

#include <rclcpp/callback_group.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>


namespace ros_signal::detail {
    class LatchHoldTimerBase {
    public:
        LatchHoldTimerBase(
            rclcpp::Node *node,
            rclcpp::Clock::SharedPtr clock,
            const rclcpp::Duration &latch_hold_period,
            rclcpp::CallbackGroup::SharedPtr callback_group
        );

        virtual ~LatchHoldTimerBase() = default;

        void set_cannot_unlatch();

        void set_can_unlatch();

        void can_unlatch(bool);

        [[nodiscard]] bool can_unlatch() const;

    private:
        virtual void latch_hold_timer_cb();

        bool can_unlatch_;
        rclcpp::TimerBase::SharedPtr latch_hold_timer_;
    };
} // namespace ros_signal::detail

#endif // ROS_SIGNAL_DETAIL_LATCH_HOLD_TIMER_BASE_HPP
