#include <gtest/gtest.h>

#include <std_msgs/msg/header.hpp>

#include <ros_signal/entry.hpp>

#include "test_types.hpp"


TEST(entry, nullable) {
    ::testing::StaticAssertTypeEq<ValueEntry::nullable_type, std::optional<ValueEntry> >();
    ::testing::StaticAssertTypeEq<HeaderValueEntry::nullable_type, std::optional<HeaderValueEntry> >();
    ::testing::StaticAssertTypeEq<StampValueEntry::nullable_type, std::optional<StampValueEntry> >();
}

TEST(entry, value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = Value{3};
    ValueEntry t{stamp, msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->data, msg.data);
}

TEST(entry, null_value) {
    ValueEntry::nullable_type t = ValueEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = Value{3};
    t.emplace(stamp, msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->value().data, msg.data);
}

TEST(entry, header_value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = HeaderValue{std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")};
    HeaderValueEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg.header);
}

TEST(entry, null_header_value) {
    HeaderValueEntry::nullable_type t = HeaderValueEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = HeaderValue{std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")};
    t.emplace(msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t.value()->header, msg.header);
}

TEST(entry, stamped_value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = StampValue{stamp};
    StampValueEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg.stamp);
}

TEST(entry, null_stamped_value) {
    StampValueEntry::nullable_type t = StampValueEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = StampValue{stamp};
    t.emplace(msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->value().stamp, msg.stamp);
}

TEST(entry_ptr, nullable) {
    ::testing::StaticAssertTypeEq<ValueSharedPtrEntry::nullable_type, std::optional<ValueSharedPtrEntry> >();
    ::testing::StaticAssertTypeEq<HeaderValueSharedPtrEntry::nullable_type, HeaderValueSharedPtrEntry>();
    ::testing::StaticAssertTypeEq<StampValueSharedPtrEntry::nullable_type, StampValueSharedPtrEntry>();
}

TEST(entry_ptr, value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<Value>(Value{3});
    ValueSharedPtrEntry t{stamp, msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->data, msg->data);
}

TEST(entry_ptr, null_value) {
    ValueSharedPtrEntry::nullable_type t = ValueSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<Value>(Value{3});
    t.emplace(stamp, msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->value().data, msg->data);
}

TEST(entry_ptr, header_value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<HeaderValue>(HeaderValue{
        std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")
    });
    HeaderValueSharedPtrEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg->header);
}

TEST(entry_ptr, null_header_value) {
    HeaderValueSharedPtrEntry::nullable_type t = HeaderValueSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<HeaderValue>(HeaderValue{
        std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")
    });
    t.emplace_ptr(msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg->header);
}

TEST(entry_ptr, stamped_value) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<StampValue>(StampValue{stamp});
    StampValueSharedPtrEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg->stamp);
}

TEST(entry_ptr, null_stamped_value) {
    StampValueSharedPtrEntry::nullable_type t = StampValueSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<StampValue>(StampValue{stamp});
    t.emplace_ptr(msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg->stamp);
}
