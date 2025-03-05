#include "ros_signal/detail/latch_hold_timer_base.hpp"

#include <rclcpp/create_timer.hpp>


ros_signal::detail::LatchHoldTimerBase::LatchHoldTimerBase(
        rclcpp::Node *const node,
        rclcpp::Clock::SharedPtr clock,
        const rclcpp::Duration &latch_hold_period,
        rclcpp::CallbackGroup::SharedPtr callback_group
) : can_unlatch_(false) {
    latch_hold_timer_ = rclcpp::create_timer(
        node, std::move(clock),
        latch_hold_period.to_chrono<std::chrono::duration<double>>(),
        std::bind(&LatchHoldTimerBase::latch_hold_timer_cb, this),
        std::move(callback_group)
    );
    set_can_unlatch();
}

void ros_signal::detail::LatchHoldTimerBase::set_cannot_unlatch() {
    can_unlatch_ = false;
    latch_hold_timer_->reset();
}

void ros_signal::detail::LatchHoldTimerBase::set_can_unlatch() {
    can_unlatch_ = true;
    latch_hold_timer_->cancel();
}

void ros_signal::detail::LatchHoldTimerBase::can_unlatch(const bool can_unlatch) {
    if (can_unlatch) {
        set_can_unlatch();
    } else {
        set_cannot_unlatch();
    }
}

bool ros_signal::detail::LatchHoldTimerBase::can_unlatch() const {
    return can_unlatch_;
}

void ros_signal::detail::LatchHoldTimerBase::latch_hold_timer_cb() {
    set_can_unlatch();
}
