#ifndef ROS_SIGNAL_TIMEOUT_MONITOR_BASE_HPP
#define ROS_SIGNAL_TIMEOUT_MONITOR_BASE_HPP

#include <memory>

#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include <ros_signal_interfaces/msg/timeout_monitor_state.hpp>

#include "ros_signal/timeout_monitor_options.hpp"


namespace ros_signal {
    class TimeoutMonitorBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(TimeoutMonitorBase)

        template<typename A = std::allocator<void>>
        using state_message_type = ros_signal_interfaces::msg::TimeoutMonitorState_<A>;

        explicit TimeoutMonitorBase(const ros_signal::TimeoutMonitorBaseOptions &options = ros_signal::TimeoutMonitorBaseOptions());

        TimeoutMonitorBase(const TimeoutMonitorBase &) = delete;

        TimeoutMonitorBase(TimeoutMonitorBase &&) = default;

        TimeoutMonitorBase &operator=(const TimeoutMonitorBase &) = delete;

        TimeoutMonitorBase &operator=(TimeoutMonitorBase &&) = default;

        virtual ~TimeoutMonitorBase() = default;

        virtual void signal(const rclcpp::Time &stamp);

        virtual void timeout(const rclcpp::Time &stamp);

        void latch(const rclcpp::Time &stamp);

        void unlatch(const rclcpp::Time &stamp);

        [[nodiscard]] bool error() const {
            return error_;
        }

        template<typename A>
        void to_state_message(state_message_type<A> &, const rclcpp::Time &now) const noexcept;

    protected:
        ros_signal::TimeoutMonitorBaseOptions timeout_monitor_options_;

    private:
        virtual void latch_handler(const rclcpp::Time &) {
        }

        virtual void unlatch_handler(const rclcpp::Time &) {
        }

        bool error_;
        rclcpp::Time last_signal_time_;
    };

    template<typename A>
    void TimeoutMonitorBase::to_state_message(state_message_type<A> &msg, const rclcpp::Time &now) const noexcept {
        msg.elapsed = now - last_signal_time_;
        msg.timeout = timeout_monitor_options_.timeout();

        msg.error = error_;
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_TIMEOUT_MONITOR_BASE_HPP
