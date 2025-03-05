#ifndef ROS_SIGNAL_SIGNAL_MONITOR_OPTIONS_HPP
#define ROS_SIGNAL_SIGNAL_MONITOR_OPTIONS_HPP

#include <cstddef>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>


namespace ros_signal {
    class SignalMonitorBaseOptions {
    public:
        explicit SignalMonitorBaseOptions();

        virtual ~SignalMonitorBaseOptions();

        virtual const SignalMonitorBaseOptions &validate() const;
    };

    class SignalMonitorOptions : public SignalMonitorBaseOptions {
    public:
        explicit SignalMonitorOptions();

        explicit SignalMonitorOptions(const SignalMonitorBaseOptions &base_options);

        explicit SignalMonitorOptions(SignalMonitorBaseOptions &&base_options);

        ~SignalMonitorOptions() override;

        const rclcpp::Duration &latch_hold_period() const;

        SignalMonitorOptions &latch_hold_period(const rclcpp::Duration &latch_hold_period);

        const rclcpp::CallbackGroup::SharedPtr &callback_group() const;

        SignalMonitorOptions &callback_group(rclcpp::CallbackGroup::SharedPtr callback_group);

        const SignalMonitorOptions &validate() const override;

    private:
        rclcpp::Duration latch_hold_period_{0, 0};
        rclcpp::CallbackGroup::SharedPtr callback_group_;
    };
} // namespace ros_signal

#endif // ROS_SIGNAL_SIGNAL_MONITOR_OPTIONS_HPP
