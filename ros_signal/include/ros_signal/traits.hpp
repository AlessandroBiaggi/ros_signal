#ifndef ROS_SIGNAL_MESSAGE_HPP
#define ROS_SIGNAL_MESSAGE_HPP

#include <cstddef>

#include <type_traits>

#include <rclcpp/loaned_message.hpp>

#include "ros_signal/detail/traits.hpp"


namespace ros_signal {
    template<typename T>
    struct is_nullable : std::bool_constant<
                std::is_constructible_v<std::decay_t<T>, std::nullptr_t>
                && std::is_assignable_v<std::decay_t<T>, std::nullptr_t>> {
    };

    template<typename T>
    inline constexpr bool is_nullable_v = is_nullable<T>::value;

    template<typename, typename = void>
    struct header {
    };

    template<typename T>
    const auto &get_header(const T &v) {
        return header<T>::get(v);
    }

    template<typename T>
    auto &get_header(T &v) {
        return header<T>::get(v);
    }

    template<typename, typename = void>
    struct has_header : std::false_type {
    };

    template<typename T>
    struct has_header<T, std::enable_if_t<
                detail::is_header_v<typename header<T>::type>
                && std::is_same_v<decltype(header<T>::get(std::declval<const T &>())), const typename header<T>::type &>
                && std::is_same_v<decltype(header<T>::get(std::declval<T &>())), typename header<T>::type &>
            > > : std::true_type {
    };

    template<typename T>
    inline constexpr bool has_header_v = has_header<T>::value;

    template<typename T>
    struct header<T, std::enable_if_t<detail::is_header_v<decltype(std::declval<T>().header)> > > {
        using value_type = std::decay_t<T>;
        using type = decltype(std::declval<value_type>().header);

        static const type &get(const value_type &msg) {
            return msg.header;
        }

        static type &get(value_type &msg) {
            return msg.header;
        }
    };

    template<typename T>
    struct header<T, std::enable_if_t<has_header_v<typename std::pointer_traits<T>::element_type> > > {
        using value_type = std::decay_t<T>;
        using element_type = typename std::pointer_traits<T>::element_type;

        using type = typename header<element_type>::type;

        static const type &get(const T &msg) {
            return header<element_type>::get(*msg);
        }

        static type &get(T &msg) {
            return header<element_type>::get(*msg);
        }
    };

    template<typename T, typename AllocT>
    struct header<rclcpp::LoanedMessage<T, AllocT>, std::enable_if_t<has_header_v<T> > > {
        using type = typename header<T>::type;

        static const type &get(const rclcpp::LoanedMessage<T, AllocT> &msg) {
            return header<T>::get(msg.get());
        }

        static type &get(rclcpp::LoanedMessage<T, AllocT> &msg) {
            return header<T>::get(msg.get());
        }
    };

    template<typename AllocT>
    struct header<std_msgs::msg::Header_<AllocT> > {
        using type = std_msgs::msg::Header_<AllocT>;

        static const type &get(const type &msg) {
            return msg;
        }

        static type &get(type &msg) {
            return msg;
        }
    };

    template<typename, typename = void>
    struct stamp {
    };

    template<typename T>
    const auto &get_stamp(const T &v) {
        return stamp<T>::get(v);
    }

    template<typename T>
    auto &get_stamp(T &v) {
        return stamp<T>::get(v);
    }

    template<typename, typename = void>
    struct has_stamp : std::false_type {
    };

    template<typename T>
    struct has_stamp<T, std::enable_if_t<
                detail::is_time_v<typename stamp<T>::type>
                && std::is_same_v<decltype(stamp<T>::get(std::declval<const T &>())), const typename stamp<T>::type &>
                && std::is_same_v<decltype(stamp<T>::get(std::declval<T &>())), typename stamp<T>::type &>
            > > : std::true_type {
    };

    template<typename T>
    inline constexpr bool has_stamp_v = has_stamp<T>::value;

    template<typename T>
    struct stamp<T, std::enable_if_t<detail::is_time_v<decltype(std::declval<T>().stamp)> > > {
        using value_type = std::decay_t<T>;
        using type = decltype(std::declval<value_type>().stamp);

        static const type &get(const value_type &msg) {
            return msg.stamp;
        }

        static type &get(value_type &msg) {
            return msg.stamp;
        }
    };

    template<typename T>
    struct stamp<T, std::enable_if_t<has_header_v<T> > > {
        using value_type = std::decay_t<T>;
        using type = decltype(header<value_type>::get(std::declval<value_type>()).stamp);

        static const type &get(const value_type &msg) {
            return header<T>::get(msg).stamp;
        }

        static type &get(value_type &msg) {
            return header<T>::get(msg).stamp;
        }
    };

    template<typename T>
    struct stamp<T, std::enable_if_t<
                not has_header_v<T> and has_stamp_v<typename std::pointer_traits<T>::element_type>> > {
        using value_type = std::decay_t<T>;
        using element_type = typename std::pointer_traits<T>::element_type;

        using type = typename stamp<element_type>::type;

        static const type &get(const value_type &msg) {
            return stamp<element_type>::get(*msg);
        }

        static type &get(value_type &msg) {
            return stamp<element_type>::get(*msg);
        }
    };

    template<typename T, typename AllocT>
    struct stamp<rclcpp::LoanedMessage<T, AllocT>, std::enable_if_t<not has_header_v<T> and has_stamp_v<T>> > {
        using type = typename stamp<T>::type;

        static const type &get(const rclcpp::LoanedMessage<T, AllocT> &msg) {
            return stamp<T>::get(msg.get());
        }

        static type &get(rclcpp::LoanedMessage<T, AllocT> &msg) {
            return stamp<T>::get(msg.get());
        }
    };

    template<typename AllocT>
    struct stamp<builtin_interfaces::msg::Time_<AllocT> > {
        using type = builtin_interfaces::msg::Time_<AllocT>;

        static const type &get(const type &msg) {
            return msg;
        }

        static type &get(type &msg) {
            return msg;
        }
    };
} // namespace ros_signal

#endif //ROS_SIGNAL_MESSAGE_HPP
