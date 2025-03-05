#ifndef ROS_SIGNAL_TEST_TEST_TYPES_HPP
#define ROS_SIGNAL_TEST_TEST_TYPES_HPP

struct Message {
    int data = 0;

    Message() = default;

    Message(const Message &) = default;

    Message(Message &&) = default;

    Message &operator=(const Message &) = default;

    Message &operator=(Message &&) = default;

    bool operator==(const Message &rhs) const {
        return data == rhs.data;
    }
};

struct HeaderMessage {
    std_msgs::msg::Header header;
    int data = 0;

    HeaderMessage() = default;

    HeaderMessage(const HeaderMessage &) = default;

    HeaderMessage(HeaderMessage &&) = default;

    HeaderMessage &operator=(const HeaderMessage &) = default;

    HeaderMessage &operator=(HeaderMessage &&) = default;

    bool operator==(const HeaderMessage &rhs) const {
        return header == rhs.header && data == rhs.data;
    }
};

struct StampMessage {
    builtin_interfaces::msg::Time stamp;
    int data = 0;

    StampMessage() = default;

    StampMessage(const StampMessage &) = default;

    StampMessage(StampMessage &&) = default;

    StampMessage &operator=(const StampMessage &) = default;

    StampMessage &operator=(StampMessage &&) = default;

    bool operator==(const StampMessage &rhs) const {
        return stamp == rhs.stamp && data == rhs.data;
    }
};

using MessageEntry = ros_signal::Entry<Message>;
using HeaderMessageEntry = ros_signal::Entry<HeaderMessage>;
using StampMessageEntry = ros_signal::Entry<StampMessage>;

using MessageSharedPtrEntry = ros_signal::Entry<std::shared_ptr<Message> >;
using HeaderMessageSharedPtrEntry = ros_signal::Entry<std::shared_ptr<HeaderMessage> >;
using StampMessageSharedPtrEntry = ros_signal::Entry<std::shared_ptr<StampMessage> >;

#endif // ROS_SIGNAL_TEST_TEST_TYPES_HPP
