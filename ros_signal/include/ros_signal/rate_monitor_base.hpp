#ifndef ROS_SIGNAL_RATE_MONITOR_BASE_HPP
#define ROS_SIGNAL_RATE_MONITOR_BASE_HPP

#include "rate_calculator.hpp"


#include <cmath>
#include <cstddef>

#include <deque>
#include <memory>

#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include <ros_signal_interfaces/msg/rate_monitor_state.hpp>

#include "ros_signal/rate_calculator.hpp"
#include "ros_signal/rate_monitor_options.hpp"


namespace ros_signal {
    template<typename AllocT = std::allocator<void>>
    class RateMonitorBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(RateMonitorBase)

        using allocator_type = AllocT;

        template<typename A = std::allocator<void> >
        using state_message_type = ros_signal_interfaces::msg::RateMonitorState_<A>;

        explicit RateMonitorBase(
            const ros_signal::RateMonitorBaseOptions &options,
            const allocator_type &alloc = allocator_type()
        );

        RateMonitorBase(const RateMonitorBase &) = delete;

        RateMonitorBase(RateMonitorBase &&) = default;

        RateMonitorBase &operator=(const RateMonitorBase &) = delete;

        RateMonitorBase &operator=(RateMonitorBase &&) = default;

        virtual ~RateMonitorBase() = default;

        virtual bool signal(const rclcpp::Time &stamp);

        virtual bool timer(const rclcpp::Time &stamp);

        RateMonitorBase &window(const rclcpp::Duration &window) {
            rate_monitor_options_.window(window);
            calc_.window = window;
            return *this;
        }

        [[nodiscard]] rclcpp::Duration window() const {
            return rate_monitor_options_.window();
        }

        RateMonitorBase &f_min(const float f_min) {
            rate_monitor_options_.f_min(f_min);
            return *this;
        }

        [[nodiscard]] float f_min() const {
            return rate_monitor_options_.f_min();
        }

        RateMonitorBase &f_max(const float f_max) {
            rate_monitor_options_.f_max(f_max);
            return *this;
        }

        [[nodiscard]] float f_max() const {
            return rate_monitor_options_.f_max();
        }

        [[nodiscard]] float f_signal() const {
            return calc_.f();
        }

        [[nodiscard]] bool check() const;

        [[nodiscard]] bool error() const {
            return error_;
        }

        void latch(const rclcpp::Time &stamp);

        void unlatch(const rclcpp::Time &stamp);

        template<typename A>
        void to_state_message(state_message_type<A> &) const noexcept;

    protected:
        ros_signal::RateMonitorBaseOptions rate_monitor_options_;

    private:
        bool event_handler(const rclcpp::Time &stamp);

        virtual void latch_handler(const rclcpp::Time &) {
        }

        virtual void unlatch_handler(const rclcpp::Time &) {
        }

        bool error_;
        RateCalculator<AllocT> calc_;
    };

    template<typename AllocT>
    RateMonitorBase<AllocT>::RateMonitorBase(
        const ros_signal::RateMonitorBaseOptions &options,
        const allocator_type &alloc
    ) : rate_monitor_options_(options.validate()),
        error_(true),
        calc_(options.window(), alloc) {
    }

    template<typename AllocT>
    bool RateMonitorBase<AllocT>::signal(const rclcpp::Time &stamp) {
        calc_.signal(stamp);
        return event_handler(stamp);
    }

    template<typename AllocT>
    bool RateMonitorBase<AllocT>::timer(const rclcpp::Time &stamp) {
        calc_.timer(stamp);
        return event_handler(stamp);
    }

    template<typename AllocT>
    bool RateMonitorBase<AllocT>::check() const {
        const auto f = f_signal();
        return rate_monitor_options_.f_min() <= f && f <= rate_monitor_options_.f_max();
    }

    template<typename AllocT>
    void RateMonitorBase<AllocT>::latch(const rclcpp::Time &stamp) {
        error_ = true;
        latch_handler(stamp);
    }

    template<typename AllocT>
    void RateMonitorBase<AllocT>::unlatch(const rclcpp::Time &stamp) {
        error_ = false;
        unlatch_handler(stamp);
    }

    template<typename AllocT>
    template<typename A>
    void RateMonitorBase<AllocT>::to_state_message(state_message_type<A> &msg) const noexcept {
        msg.window = rate_monitor_options_.window();

        msg.times.assign(calc_.times.cbegin(), calc_.times.cend());

        msg.f_min = rate_monitor_options_.f_min();
        msg.f_max = rate_monitor_options_.f_max();
        msg.f_nominal = rate_monitor_options_.f_nominal();
        msg.f_signal = f_signal();

        msg.error = error_;
    }

    template<typename AllocT>
    bool RateMonitorBase<AllocT>::event_handler(const rclcpp::Time &stamp) {
        const bool ok = check();
        if (!(ok || error_)) {
            latch(stamp);
        }
        return ok;
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_RATE_MONITOR_BASE_HPP
