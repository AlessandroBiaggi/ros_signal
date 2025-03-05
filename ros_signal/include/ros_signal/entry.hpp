#ifndef ROS_SIGNAL_ENTRY_HPP
#define ROS_SIGNAL_ENTRY_HPP

#include <rclcpp/time.hpp>

#include "ros_signal/traits.hpp"
#include "ros_signal/detail/entry_storage.hpp"


namespace ros_signal {
    template<typename MsgT>
    class Entry final : public detail::EntryTraits<MsgT, Entry<MsgT> >,
                        public detail::EntryStampStorage<MsgT, Entry<MsgT> >,
                        public detail::EntryMessageStorage<MsgT, Entry<MsgT> > {
    public:
        using typename detail::EntryStampStorage<MsgT, Entry>::time_type;
        using typename detail::EntryMessageStorage<MsgT, Entry>::message_type;

        explicit Entry(const time_type &stamp, const message_type &message = message_type())
            : detail::EntryStampStorage<MsgT, Entry>(stamp),
              detail::EntryMessageStorage<MsgT, Entry>(message) {
        }

        Entry(const time_type &stamp, message_type &&message)
            : detail::EntryStampStorage<MsgT, Entry>(stamp),
              detail::EntryMessageStorage<MsgT, Entry>(message) {
        }

        Entry(const message_type &message = message_type())
            : detail::EntryStampStorage<MsgT, Entry>(),
              detail::EntryMessageStorage<MsgT, Entry>(message) {
        }

        Entry(message_type &&message)
            : detail::EntryStampStorage<MsgT, Entry>(),
              detail::EntryMessageStorage<MsgT, Entry>(std::forward<message_type>(message)) {
        }

        Entry(const Entry &) = default;

        Entry(Entry &&) = default;

        ~Entry() = default;

        Entry &operator=(const Entry &) = default;

        Entry &operator=(Entry &&) = default;

        bool operator==(const Entry &rhs) const {
            return detail::EntryStampStorage<MsgT, Entry>::stamp() == rhs.stamp() &&
                   detail::EntryMessageStorage<MsgT, Entry>::message() == rhs.message();
        }

        bool operator!=(const Entry &rhs) const {
            return !(*this == rhs);
        }
    };

    template<typename MsgT>
    struct LessEntryStamp {
        bool operator()(const Entry<MsgT> &lhs, const Entry<MsgT> &rhs) const {
            return rclcpp::Time(lhs.stamp()) < rclcpp::Time(rhs.stamp());
        }

        bool operator()(const Entry<MsgT> &lhs, const rclcpp::Time &rhs) const {
            return rclcpp::Time(lhs.stamp()) < rhs;
        }

        bool operator()(const rclcpp::Time &lhs, const Entry<MsgT> &rhs) const {
            return lhs < rclcpp::Time(rhs.stamp());
        }
    };

    template<typename MsgT>
    const auto &get_header(const Entry<MsgT> &e) {
        return ros_signal::get_header(e.message());
    }

    template<typename MsgT>
    auto &get_header(Entry<MsgT> &e) {
        return ros_signal::get_header(e.message());
    }

    template<typename MsgT>
    const auto &get_stamp(const Entry<MsgT> &e) {
        return e.stamp();
    }

    template<typename MsgT>
    auto &get_stamp(Entry<MsgT> &e) {
        return e.stamp();
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_ENTRY_HPP
