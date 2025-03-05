#include <gtest/gtest.h>

#include <std_msgs/msg/header.hpp>

#include <ros_signal/entry.hpp>

#include "test_types.hpp"


TEST(entry, nullable) {
    ::testing::StaticAssertTypeEq<MessageEntry::nullable_type, std::optional<MessageEntry> >();
    ::testing::StaticAssertTypeEq<HeaderMessageEntry::nullable_type, std::optional<HeaderMessageEntry> >();
    ::testing::StaticAssertTypeEq<StampMessageEntry::nullable_type, std::optional<StampMessageEntry> >();
}

TEST(entry, message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = Message{3};
    MessageEntry t{stamp, msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->data, msg.data);
}

TEST(entry, null_message) {
    MessageEntry::nullable_type t = MessageEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = Message{3};
    t.emplace(stamp, msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->message().data, msg.data);
}

TEST(entry, header_message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = HeaderMessage{std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")};
    HeaderMessageEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg.header);
}

TEST(entry, null_header_message) {
    HeaderMessageEntry::nullable_type t = HeaderMessageEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = HeaderMessage{std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")};
    t.emplace(msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t.value()->header, msg.header);
}

TEST(entry, stamped_message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = StampMessage{stamp};
    StampMessageEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg.stamp);
}

TEST(entry, null_stamped_message) {
    StampMessageEntry::nullable_type t = StampMessageEntry::null();
    EXPECT_EQ(t, std::nullopt);

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = StampMessage{stamp};
    t.emplace(msg);
    EXPECT_NE(t, std::nullopt);
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->message().stamp, msg.stamp);
}

TEST(entry_ptr, nullable) {
    ::testing::StaticAssertTypeEq<MessageSharedPtrEntry::nullable_type, std::optional<MessageSharedPtrEntry> >();
    ::testing::StaticAssertTypeEq<HeaderMessageSharedPtrEntry::nullable_type, HeaderMessageSharedPtrEntry>();
    ::testing::StaticAssertTypeEq<StampMessageSharedPtrEntry::nullable_type, StampMessageSharedPtrEntry>();
}

TEST(entry_ptr, message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<Message>(Message{3});
    MessageSharedPtrEntry t{stamp, msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->data, msg->data);
}

TEST(entry_ptr, null_message) {
    MessageSharedPtrEntry::nullable_type t = MessageSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<Message>(Message{3});
    t.emplace(stamp, msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t->stamp(), stamp);
    EXPECT_EQ(t->message().data, msg->data);
}

TEST(entry_ptr, header_message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<HeaderMessage>(HeaderMessage{
        std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")
    });
    HeaderMessageSharedPtrEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg->header);
}

TEST(entry_ptr, null_header_message) {
    HeaderMessageSharedPtrEntry::nullable_type t = HeaderMessageSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<HeaderMessage>(HeaderMessage{
        std_msgs::msg::Header().set__stamp(stamp).set__frame_id("id")
    });
    t.emplace(msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->header, msg->header);
}

TEST(entry_ptr, stamped_message) {
    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<StampMessage>(StampMessage{stamp});
    StampMessageSharedPtrEntry t{msg};

    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg->stamp);
}

TEST(entry_ptr, null_stamped_message) {
    StampMessageSharedPtrEntry::nullable_type t = StampMessageSharedPtrEntry::null();
    EXPECT_FALSE(t.has_value());

    const auto stamp = rclcpp::Time(1, 2);
    const auto msg = std::make_shared<StampMessage>(StampMessage{stamp});
    t.emplace(msg);
    EXPECT_TRUE(t.has_value());
    EXPECT_EQ(t.stamp(), stamp);
    EXPECT_EQ(t->stamp, msg->stamp);
}
