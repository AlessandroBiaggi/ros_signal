#ifndef ROS_SIGNAL_ENTRY_HPP
#define ROS_SIGNAL_ENTRY_HPP

#include <rclcpp/time.hpp>

#include "ros_signal/detail/entry_storage.hpp"
#include "ros_signal/traits.hpp"


namespace ros_signal {
    template<typename ValueT, typename = void>
    class Entry final : public detail::EntryTraits<ValueT, Entry<ValueT>>,
                        public detail::EntryStampStorage<ValueT, Entry<ValueT>>,
                        public detail::EntryValueStorage<ValueT, Entry<ValueT>> {
    public:
        using typename detail::EntryStampStorage<ValueT, Entry>::time_type;
        using typename detail::EntryValueStorage<ValueT, Entry>::value_type;

        Entry(const time_type &stamp, const value_type &value)
            : detail::EntryStampStorage<ValueT, Entry>(stamp),
              detail::EntryValueStorage<ValueT, Entry>(value) {
        }

        Entry(const time_type &stamp, value_type &&value)
            : detail::EntryStampStorage<ValueT, Entry>(stamp),
              detail::EntryValueStorage<ValueT, Entry>(value) {
        }

        Entry(const value_type &value)
            : detail::EntryStampStorage<ValueT, Entry>(),
              detail::EntryValueStorage<ValueT, Entry>(value) {
        }

        Entry(value_type &&value)
            : detail::EntryStampStorage<ValueT, Entry>(),
              detail::EntryValueStorage<ValueT, Entry>(std::forward<value_type>(value)) {
        }

        Entry(const Entry &) = default;

        Entry(Entry &&) = default;

        ~Entry() = default;

        Entry &operator=(const Entry &) = default;

        Entry &operator=(Entry &&) = default;

        bool operator==(const Entry &rhs) const {
            return detail::EntryStampStorage<ValueT, Entry>::stamp() == rhs.stamp() &&
                   detail::EntryValueStorage<ValueT, Entry>::value() == rhs.value();
        }

        bool operator!=(const Entry &rhs) const {
            return !(*this == rhs);
        }
    };

    template<typename ValuePtrT>
    class Entry<ValuePtrT, std::enable_if_t<ros_signal::is_nullable_v<ValuePtrT>>> final
        : public detail::EntryTraits<ValuePtrT, Entry<ValuePtrT>>,
          public detail::EntryStampStorage<ValuePtrT, Entry<ValuePtrT>>,
          public detail::EntryValueStorage<ValuePtrT, Entry<ValuePtrT>> {
    public:
        using typename detail::EntryStampStorage<ValuePtrT, Entry>::time_type;
        using typename detail::EntryValueStorage<ValuePtrT, Entry>::value_ptr_type;
        using typename detail::EntryValueStorage<ValuePtrT, Entry>::value_type;

        Entry(const time_type &stamp, const value_ptr_type &value)
            : detail::EntryStampStorage<ValuePtrT, Entry>(stamp),
              detail::EntryValueStorage<ValuePtrT, Entry>(value) {
        }

        Entry(const time_type &stamp, value_ptr_type &&value)
            : detail::EntryStampStorage<ValuePtrT, Entry>(stamp),
              detail::EntryValueStorage<ValuePtrT, Entry>(value) {
        }

        Entry(const value_ptr_type &value)
            : detail::EntryStampStorage<ValuePtrT, Entry>(),
              detail::EntryValueStorage<ValuePtrT, Entry>(value) {
        }

        Entry(value_ptr_type &&value)
            : detail::EntryStampStorage<ValuePtrT, Entry>(),
              detail::EntryValueStorage<ValuePtrT, Entry>(std::forward<value_ptr_type>(value)) {
        }

        Entry(const Entry &) = default;

        Entry(Entry &&) = default;

        ~Entry() = default;

        Entry &operator=(const Entry &) = default;

        Entry &operator=(Entry &&) = default;

        bool operator==(const Entry &rhs) const {
            return detail::EntryStampStorage<ValuePtrT, Entry>::stamp() == rhs.stamp() &&
                   detail::EntryValueStorage<ValuePtrT, Entry>::value() == rhs.value();
        }

        bool operator!=(const Entry &rhs) const {
            return !(*this == rhs);
        }
    };

    template<typename ValueT>
    struct LessEntryStamp {
        bool operator()(const Entry<ValueT> &lhs, const Entry<ValueT> &rhs) const {
            return rclcpp::Time(lhs.stamp()) < rclcpp::Time(rhs.stamp());
        }

        bool operator()(const Entry<ValueT> &lhs, const rclcpp::Time &rhs) const {
            return rclcpp::Time(lhs.stamp()) < rhs;
        }

        bool operator()(const rclcpp::Time &lhs, const Entry<ValueT> &rhs) const {
            return lhs < rclcpp::Time(rhs.stamp());
        }
    };

    template<typename ValueT, typename = std::enable_if_t<not std::is_same_v<Entry<ValueT>, typename Entry<ValueT>::nullable_type>>>
    auto get_header(const Entry<ValueT> &e) {
        return ros_signal::get_header(e.value());
    }

    template<typename ValueT, typename = std::enable_if_t<not std::is_same_v<Entry<ValueT>, typename Entry<ValueT>::nullable_type>>>
    auto get_header(Entry<ValueT> &e) {
        return ros_signal::get_header(e.value());
    }

    template<typename ValueT, typename = std::enable_if_t<not std::is_same_v<Entry<ValueT>, typename Entry<ValueT>::nullable_type>>>
    auto get_stamp(const Entry<ValueT> &e) -> const typename Entry<ValueT>::time_type & {
        return e.stamp();
    }

    template<typename ValueT, typename = std::enable_if_t<not std::is_same_v<Entry<ValueT>, typename Entry<ValueT>::nullable_type>>>
    auto get_stamp(Entry<ValueT> &e) -> typename Entry<ValueT>::time_type & {
        return e.stamp();
    }

    template<typename ValueT>
    auto get_header(const typename Entry<ValueT>::nullable_type &e) -> std::optional<std_msgs::msg::Header> {
        if (e.has_value()) {
            return ros_signal::get_header(e.value());
        }
        return std::nullopt;
    }

    template<typename ValueT>
    auto get_stamp(const typename Entry<ValueT>::nullable_type &e) -> std::optional<typename Entry<ValueT>::time_type> {
        if (e.has_value()) {
            return ros_signal::get_stamp(e.value());
        }
        return std::nullopt;
    }
}// namespace ros_signal

#endif// ROS_SIGNAL_ENTRY_HPP
