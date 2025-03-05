#ifndef ROS_SIGNAL_RATE_CALCULATOR_HPP
#define ROS_SIGNAL_RATE_CALCULATOR_HPP

#include <deque>
#include <memory>

#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>


namespace ros_signal {
    template<typename AllocT = std::allocator<void>>
    struct RateCalculator {
        RCLCPP_SMART_PTR_DEFINITIONS(RateCalculator)

        using allocator_type = AllocT;

        explicit RateCalculator(
                const rclcpp::Duration &window,
                const allocator_type &alloc = allocator_type()
        );

        ~RateCalculator() = default;

        void signal(const rclcpp::Time &stamp);

        void timer(const rclcpp::Time &stamp);

        [[nodiscard]] float f() const;

        allocator_type get_allocator() const {
            return times.get_allocator();
        }

        rclcpp::Duration window;

        using TimeAllocator = typename std::allocator_traits<AllocT>::template rebind_alloc<rclcpp::Time>;
        std::deque<rclcpp::Time, TimeAllocator> times;
    };

    template<typename AllocT>
    RateCalculator<AllocT>::RateCalculator(
            const rclcpp::Duration &window,
            const allocator_type &alloc
    ) : window(window),
        times(alloc) {
    }

    template<typename AllocT>
    void RateCalculator<AllocT>::signal(const rclcpp::Time &stamp) {
        times.erase(
            times.begin(),
            std::upper_bound(
                times.begin(),
                times.end(),
                stamp - window
            )
        );
        times.push_back(stamp);
    }

    template<typename AllocT>
    void RateCalculator<AllocT>::timer(const rclcpp::Time &stamp) {
        times.erase(
            times.begin(),
            std::lower_bound(
                times.begin(),
                times.end(),
                stamp - window
            )
        );
    }

    template<typename AllocT>
    float RateCalculator<AllocT>::f() const {
        return static_cast<float>(times.size()) / window.seconds();
    }
}// namespace ros_signal

#endif//ROS_SIGNAL_RATE_CALCULATOR_HPP
