#ifndef ROS_SIGNAL_TIMEOUT_MONITOR_OPTIONS_HPP
#define ROS_SIGNAL_TIMEOUT_MONITOR_OPTIONS_HPP

#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>


namespace ros_signal {
    class TimeoutMonitorBaseOptions {
    public:
        explicit TimeoutMonitorBaseOptions();

        virtual ~TimeoutMonitorBaseOptions();

        const rclcpp::Duration &timeout() const;

        TimeoutMonitorBaseOptions &timeout(const rclcpp::Duration &timeout);

        virtual const TimeoutMonitorBaseOptions &validate() const;

    private:
        rclcpp::Duration timeout_{0, 0};
    };

    class TimeoutMonitorOptions : public TimeoutMonitorBaseOptions {
    public:
        explicit TimeoutMonitorOptions();

        explicit TimeoutMonitorOptions(const TimeoutMonitorBaseOptions &base_options);

        explicit TimeoutMonitorOptions(TimeoutMonitorBaseOptions &&base_options);

        ~TimeoutMonitorOptions() override;

        const rclcpp::Duration &latch_hold_period() const;

        TimeoutMonitorOptions &latch_hold_period(const rclcpp::Duration &latch_hold_period);

        const rclcpp::CallbackGroup::SharedPtr &callback_group() const;

        TimeoutMonitorOptions &callback_group(rclcpp::CallbackGroup::SharedPtr callback_group);

        const TimeoutMonitorOptions &validate() const override;
    private:
        rclcpp::Duration latch_hold_period_{0, 0};
        rclcpp::CallbackGroup::SharedPtr callback_group_;
    };
} // namespace ros_signal

#endif // ROS_SIGNAL_TIMEOUT_MONITOR_OPTIONS_HPP
