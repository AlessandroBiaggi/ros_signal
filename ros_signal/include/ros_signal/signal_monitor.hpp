#ifndef ROS_SIGNAL_SIGNAL_MONITOR_HPP
#define ROS_SIGNAL_SIGNAL_MONITOR_HPP

#include "signal_monitor_base.hpp"


#include <functional>
#include <memory>
#include <type_traits>

#include <rclcpp/clock.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include "ros_signal/detail/latch_hold_timer_base.hpp"
#include "ros_signal/signal_monitor_base.hpp"
#include "ros_signal/signal_monitor_options.hpp"

#include <oneapi/tbb/partitioner.h>


namespace ros_signal {
    template<typename MsgT, typename AllocT = std::allocator<void>>
    class SignalMonitor : public SignalMonitorBase<MsgT, AllocT>, protected detail::LatchHoldTimerBase {
    public:
        using typename SignalMonitorBase<MsgT, AllocT>::value_type;
        using typename SignalMonitorBase<MsgT, AllocT>::allocator_type;

        RCLCPP_SMART_PTR_DEFINITIONS(SignalMonitor)

        template<typename... CheckTs>
        SignalMonitor(
            rclcpp::Node *const node,
            rclcpp::Clock::SharedPtr clock,
            const SignalMonitorOptions &options,
            std::function<void(const rclcpp::Time &)> state_change_cb,
            CheckTs... checks
        ) : SignalMonitor(
            allocator_type(),
            node,
            std::move(clock),
            options,
            std::move(state_change_cb),
            std::move(checks)...
        ) {}

        template<typename... CheckTs>
        SignalMonitor(
            const allocator_type &alloc,
            rclcpp::Node *node,
            rclcpp::Clock::SharedPtr clock,
            const SignalMonitorOptions &options,
            std::function<void(const rclcpp::Time &)> state_change_cb,
            CheckTs... checks
        );

        SignalMonitor(const SignalMonitor &) = delete;

        SignalMonitor(SignalMonitor &&) = default;

        SignalMonitor &operator=(const SignalMonitor &) = delete;

        SignalMonitor &operator=(SignalMonitor &&) = default;

        ~SignalMonitor() override = default;

        bool signal(const rclcpp::Time &stamp, const value_type &msg) override;

    private:
        void latch_handler(const rclcpp::Time &stamp) override {
            set_cannot_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        void unlatch_handler(const rclcpp::Time &stamp) override {
            set_can_unlatch();
            state_change_cb_(std::cref(stamp));
        }

        // check_tuple_type checks_;
        std::function<void(const rclcpp::Time &)> state_change_cb_;
    };

    template<typename MsgT, typename AllocT>
    template<typename... CheckTs>
    SignalMonitor<MsgT, AllocT>::SignalMonitor(
            const allocator_type &alloc,
            rclcpp::Node *const node,
            rclcpp::Clock::SharedPtr clock,
            const SignalMonitorOptions &options,
            std::function<void(const rclcpp::Time &)> state_change_cb,
            CheckTs... checks
    ) : SignalMonitorBase<MsgT, AllocT>(alloc, options.validate(), std::move(checks)...),
        LatchHoldTimerBase(node, std::move(clock), options.latch_hold_period(), options.callback_group()),
        state_change_cb_(std::move(state_change_cb)) {
        static_assert((std::is_invocable_r_v<bool, std::decay_t<CheckTs>, const value_type &> && ...),
                      "CheckTs must be invocable with const std::decay_t<MsgT> & and return bool");
    }

    template<typename MsgT, typename AllocT>
    bool SignalMonitor<MsgT, AllocT>::signal(const rclcpp::Time &stamp, const value_type &msg) {
        const auto err = SignalMonitorBase<MsgT, AllocT>::error();
        const auto ok = SignalMonitorBase<MsgT, AllocT>::signal(stamp, msg);
        if (err && ok && can_unlatch()) {
            SignalMonitorBase<MsgT, AllocT>::unlatch(stamp);
        }
        return ok;
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_SIGNAL_MONITOR_HPP
