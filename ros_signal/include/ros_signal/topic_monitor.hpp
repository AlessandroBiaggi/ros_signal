#ifndef ROS_SIGNAL_TOPIC_MONITOR_HPP
#define ROS_SIGNAL_TOPIC_MONITOR_HPP

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>

#include "ros_signal/rate_monitor.hpp"
#include "ros_signal/signal_monitor.hpp"
#include "ros_signal/timeout_monitor.hpp"

#include "ros_signal/rate_monitor_options.hpp"
#include "ros_signal/signal_monitor_options.hpp"
#include "ros_signal/timeout_monitor_options.hpp"

#include "ros_signal/traits.hpp"

#include <ros_signal_interfaces/msg/monitor_state.hpp>


namespace ros_signal {
    template<typename MsgT, typename AllocT = std::allocator<void>>
    class TopicMonitor {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(TopicMonitor)

        using value_type = typename SignalMonitorBase<MsgT, AllocT>::value_type;

        template<typename A = std::allocator<void>>
        using state_message_type = ros_signal_interfaces::msg::MonitorState_<A>;

        template<typename... CheckTs>
        TopicMonitor(
            rclcpp::Node *node,
            rclcpp::Clock::SharedPtr clock,
            const rclcpp::Duration &latch_hold_period,
            const rclcpp::Duration &window,
            float f_nominal,
            float f_min,
            float f_max,
            const rclcpp::Duration &timeout,
            std::function<void(const rclcpp::Time &)> state_change_cb,
            rclcpp::CallbackGroup::SharedPtr cg,
            CheckTs... checks
        );

        void signal(const rclcpp::Time &stamp, const value_type &msg);

        void signal(const value_type &msg) {
            signal(ros_signal::get_stamp(msg), msg);
        }

        [[nodiscard]] bool error() const;

        [[nodiscard]] bool rate_error() const {
            return rate_monitor_->error();
        }

        [[nodiscard]] bool signal_error() const {
            return signal_monitor_->error();
        }

        template<size_t Idx>
        [[nodiscard]] bool signal_error() const {
            return signal_monitor_->template error<Idx>();
        }

        [[nodiscard]] bool signal_error(const size_t idx) const {
            return signal_monitor_->error(idx);
        }

        [[nodiscard]] bool timeout_error() const {
            return timeout_monitor_->error();
        }

        void unlatch(const rclcpp::Time &stamp) {
            rate_monitor_->unlatch(stamp);
            signal_monitor_->unlatch(stamp);
            timeout_monitor_->unlatch(stamp);
        }

        template<typename A>
        void to_state_message(state_message_type<A> &msg, const rclcpp::Time &) const;
    private:
        typename RateMonitor<AllocT>::SharedPtr rate_monitor_;
        typename SignalMonitor<MsgT, AllocT>::SharedPtr signal_monitor_;
        TimeoutMonitor::SharedPtr timeout_monitor_;
    };

    template<typename MsgT, typename AllocT>
    template<typename... CheckTs>
    TopicMonitor<MsgT, AllocT>::TopicMonitor(
        rclcpp::Node *const node,
        rclcpp::Clock::SharedPtr clock,
        const rclcpp::Duration &latch_hold_period,
        const rclcpp::Duration &window,
        const float f_nominal,
        const float f_min,
        const float f_max,
        const rclcpp::Duration &timeout,
        std::function<void(const rclcpp::Time &)> state_change_cb,
        rclcpp::CallbackGroup::SharedPtr cg,
        CheckTs... checks
    ) {
        auto rate_monitor_options = RateMonitorOptions();
        rate_monitor_options
                .latch_hold_period(latch_hold_period)
                .callback_group(cg)
                .window(window)
                .f_nominal(f_nominal)
                .f_min(f_min)
                .f_max(f_max);
        rate_monitor_ = RateMonitor<AllocT>::make_shared(
            node, clock,
            rate_monitor_options,
            state_change_cb
        );

        auto signal_monitor_options = SignalMonitorOptions();
        signal_monitor_options
                .latch_hold_period(latch_hold_period)
                .callback_group(cg);
        signal_monitor_ = SignalMonitor<MsgT, AllocT>::make_shared(
            node, clock,
            signal_monitor_options,
            state_change_cb,
            std::move(checks)...
        );

        auto timeout_monitor_options = TimeoutMonitorOptions();
        timeout_monitor_options
                .latch_hold_period(latch_hold_period)
                .callback_group(cg)
                .timeout(timeout);
        timeout_monitor_ = TimeoutMonitor::make_shared(
            node, clock,
            timeout_monitor_options,
            state_change_cb
        );
}

    template<typename MsgT, typename AllocT>
    void TopicMonitor<MsgT, AllocT>::signal(const rclcpp::Time &stamp, const value_type &msg) {
        rate_monitor_->signal(stamp);
        signal_monitor_->signal(stamp, msg);
        timeout_monitor_->signal(stamp);
    }

    template<typename MsgT, typename AllocT>
    bool TopicMonitor<MsgT, AllocT>::error() const {
        return rate_error() || signal_error() || timeout_error();
    }

    template<typename MsgT, typename AllocT>
    template<typename A>
    void TopicMonitor<MsgT, AllocT>::to_state_message(state_message_type<A> &msg, const rclcpp::Time &stamp) const {
        msg.error = error();
        rate_monitor_->to_state_message(msg.rate);
        signal_monitor_->to_state_message(msg.signal);
        timeout_monitor_->to_state_message(msg.timeout, stamp);
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_TOPIC_MONITOR_HPP
