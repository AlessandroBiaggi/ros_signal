#ifndef ROS_SIGNAL_DETAIL_TRAITS_HPP
#define ROS_SIGNAL_DETAIL_TRAITS_HPP

#include <type_traits>

#include <std_msgs/msg/header.hpp>

#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>


namespace ros_signal::detail {
    template<typename T>
    struct is_header {
    private:
        using test_type = std::remove_cv_t<T>;

        template<typename AllocT>
        static std::true_type test(const std_msgs::msg::Header_<AllocT> *);

        static std::false_type test(...);

    public:
        static constexpr bool value = decltype(test(std::declval<const test_type *>()))::value;
    };

    template<typename T>
    static constexpr bool is_header_v = is_header<T>::value;

    template<typename T>
    struct is_time {
    private:
        using test_type = std::remove_cv_t<T>;

        static std::true_type test(const rclcpp::Time *);

        template<typename AllocT>
        static std::true_type test(const builtin_interfaces::msg::Time_<AllocT> *);

        static std::false_type test(...);

    public:
        static constexpr bool value = decltype(test(std::declval<const test_type *>()))::value;
    };

    template<typename T>
    static constexpr bool is_time_v = is_time<T>::value;
} // namespace ros_signal::detail

#endif // ROS_SIGNAL_DETAIL_TRAITS_HPP
