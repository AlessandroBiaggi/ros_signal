#ifndef ROS_SIGNAL_RATE_MONITOR_OPTIONS_HPP
#define ROS_SIGNAL_RATE_MONITOR_OPTIONS_HPP

#include <rclcpp/callback_group.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>

namespace ros_signal {
    class RateMonitorBaseOptions {
    public:
        explicit RateMonitorBaseOptions();

        virtual ~RateMonitorBaseOptions();

        const rclcpp::Duration &window() const;

        RateMonitorBaseOptions &window(const rclcpp::Duration &window);

        float f_nominal() const;

        RateMonitorBaseOptions &f_nominal(float f_nominal);

        float f_min() const;

        RateMonitorBaseOptions &f_min(float f_min);

        float f_max() const;

        RateMonitorBaseOptions &f_max(float f_max);

        virtual const ros_signal::RateMonitorBaseOptions &validate() const;

    private:
        rclcpp::Duration window_{0, 0};
        float f_nominal_ = 0.0;
        float f_min_ = 0.0;
        float f_max_ = 0.0;
    };

    class RateMonitorOptions : public RateMonitorBaseOptions {
    public:
        explicit RateMonitorOptions();

        explicit RateMonitorOptions(const ros_signal::RateMonitorBaseOptions &base_options);

        explicit RateMonitorOptions(ros_signal::RateMonitorBaseOptions &&base_options);

        ~RateMonitorOptions() override;

        const rclcpp::Duration &latch_hold_period() const;

        RateMonitorOptions &latch_hold_period(const rclcpp::Duration &latch_hold_period);

        const rclcpp::CallbackGroup::SharedPtr &callback_group() const;

        RateMonitorOptions &callback_group(rclcpp::CallbackGroup::SharedPtr callback_group);

        const ros_signal::RateMonitorOptions &validate() const override;

    private:
        rclcpp::Duration latch_hold_period_{0, 0};
        rclcpp::CallbackGroup::SharedPtr callback_group_;
    };
} // namespace ros_signal

#endif // ROS_SIGNAL_RATE_MONITOR_OPTIONS_HPP
