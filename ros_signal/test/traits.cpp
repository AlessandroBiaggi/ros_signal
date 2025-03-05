#include <gtest/gtest.h>

#include <ros_signal/traits.hpp>

static_assert(ros_signal::detail::is_header_v<std_msgs::msg::Header>);


struct Empty {};

struct HasHeader {
    std_msgs::msg::Header header;
};

struct HasStamp {
    builtin_interfaces::msg::Time stamp;
};

static_assert(not ros_signal::has_header_v<Empty>);
static_assert(ros_signal::has_header_v<std_msgs::msg::Header>);

TEST(value, header) {
    HasHeader t;
    const auto &header = ros_signal::get_header(t);
    EXPECT_EQ(&t.header, &header);
}

static_assert(not ros_signal::has_stamp_v<Empty>);
static_assert(ros_signal::has_stamp_v<HasStamp>);
static_assert(ros_signal::has_stamp_v<HasHeader>);

TEST(value, stamp) {
    HasStamp t;
    const auto &stamp = ros_signal::get_stamp(t);
    EXPECT_EQ(&t.stamp, &stamp);
}

TEST(value, header_stamp) {
    HasHeader t;
    const auto &stamp = ros_signal::get_stamp(t);
    EXPECT_EQ(&t.header.stamp, &stamp);
}
