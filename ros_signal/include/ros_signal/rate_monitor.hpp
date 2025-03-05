#ifndef ROS_SIGNAL_RATE_MONITOR_HPP
#define ROS_SIGNAL_RATE_MONITOR_HPP

#include <functional>
#include <memory>

#include <rclcpp/clock.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include "ros_signal/detail/latch_hold_timer_base.hpp"
#include "ros_signal/rate_monitor_base.hpp"
#include "ros_signal/rate_monitor_options.hpp"


namespace ros_signal {
    template<typename AllocT = std::allocator<void> >
    class RateMonitor : public RateMonitorBase<AllocT>, protected detail::LatchHoldTimerBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(RateMonitor)

        using typename RateMonitorBase<AllocT>::allocator_type;

        RateMonitor(
                rclcpp::Node *node,
                rclcpp::Clock::SharedPtr clock,
                const RateMonitorOptions &options,
                std::function<void(const rclcpp::Time &)> state_change_cb,
                const allocator_type &alloc = allocator_type()
        );

        ~RateMonitor() override = default;

        bool signal(const rclcpp::Time &stamp) override;

    private:
        void latch_handler(const rclcpp::Time &stamp) override {
            set_cannot_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        void unlatch_handler(const rclcpp::Time &stamp) override {
            set_can_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        void f_nominal_timer_cb();

        rclcpp::TimerBase::SharedPtr f_nominal_timer_;
        std::function<void(const rclcpp::Time &)> state_change_cb_;

        rclcpp::Clock::SharedPtr clk_;
    };

    template<typename AllocT>
    RateMonitor<AllocT>::RateMonitor(
            rclcpp::Node *node,
            rclcpp::Clock::SharedPtr clock,
            const RateMonitorOptions &options,
            std::function<void(const rclcpp::Time &)> state_change_cb,
            const allocator_type &alloc
    ) : RateMonitorBase<AllocT>(options, alloc),
        LatchHoldTimerBase(
                node,
                clock,
                options.latch_hold_period(),
                options.callback_group()
        ),
        state_change_cb_(std::move(state_change_cb)),
        clk_(std::move(clock)) {
        f_nominal_timer_ = rclcpp::create_timer(
            node, clk_,
            std::chrono::duration<double>(1. / options.f_nominal()),
            std::bind(&RateMonitor::f_nominal_timer_cb, this),
            options.callback_group()
        );
    }

    template<typename AllocT>
    bool RateMonitor<AllocT>::signal(const rclcpp::Time &stamp) {
        if (f_nominal_timer_->is_canceled()) {
            f_nominal_timer_->reset();
        }
        const auto err = RateMonitorBase<AllocT>::error();
        const auto ok = RateMonitorBase<AllocT>::signal(stamp);
        if (err && ok && can_unlatch()) {
            RateMonitorBase<AllocT>::unlatch(stamp);
        }
        return ok;
    }

    template<typename AllocT>
    void RateMonitor<AllocT>::f_nominal_timer_cb() {
        RateMonitorBase<AllocT>::timer(clk_->now());
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_RATE_MONITOR_HPP
