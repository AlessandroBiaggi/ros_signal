#ifndef ROS_SIGNAL_TEST_TEST_TYPES_HPP
#define ROS_SIGNAL_TEST_TEST_TYPES_HPP

struct Value {
    int data = 0;

    Value() = default;

    Value(const Value &) = default;

    Value(Value &&) = default;

    Value &operator=(const Value &) = default;

    Value &operator=(Value &&) = default;

    bool operator==(const Value &rhs) const {
        return data == rhs.data;
    }
};

struct HeaderValue {
    std_msgs::msg::Header header;
    int data = 0;

    HeaderValue() = default;

    HeaderValue(const HeaderValue &) = default;

    HeaderValue(HeaderValue &&) = default;

    HeaderValue &operator=(const HeaderValue &) = default;

    HeaderValue &operator=(HeaderValue &&) = default;

    bool operator==(const HeaderValue &rhs) const {
        return header == rhs.header && data == rhs.data;
    }
};

struct StampValue {
    builtin_interfaces::msg::Time stamp;
    int data = 0;

    StampValue() = default;

    StampValue(const StampValue &) = default;

    StampValue(StampValue &&) = default;

    StampValue &operator=(const StampValue &) = default;

    StampValue &operator=(StampValue &&) = default;

    bool operator==(const StampValue &rhs) const {
        return stamp == rhs.stamp && data == rhs.data;
    }
};

using ValueEntry = ros_signal::Entry<Value>;
using HeaderValueEntry = ros_signal::Entry<HeaderValue>;
using StampValueEntry = ros_signal::Entry<StampValue>;

using ValueSharedPtrEntry = ros_signal::Entry<std::shared_ptr<Value>>;
using HeaderValueSharedPtrEntry = ros_signal::Entry<std::shared_ptr<HeaderValue>>;
using StampValueSharedPtrEntry = ros_signal::Entry<std::shared_ptr<StampValue>>;

#endif// ROS_SIGNAL_TEST_TEST_TYPES_HPP
