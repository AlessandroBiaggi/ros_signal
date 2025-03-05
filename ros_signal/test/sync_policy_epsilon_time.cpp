#include <gmock/gmock.h>

#include <ros_signal/sync_policy/epsilon_time.hpp>

#include "sync_policy_test.hpp"
#include "test_types.hpp"


using namespace ros_signal;

template<typename... MsgTs>
using SyncPolicyEpsilonTimeTest = SyncPolicyTest<sync_policy::EpsilonTime<MsgTs...> >;

using SyncPolicyEpsilonTimeTest1 = SyncPolicyEpsilonTimeTest<int>;
TEST_F(SyncPolicyEpsilonTimeTest1, epsilon_time) {
    EXPECT_EQ(sync_policy_.epsilon(), period_);

    const rclcpp::Duration epsilon_d = rclcpp::Duration::from_seconds(5.0);
    sync_policy_.epsilon(epsilon_d);
    EXPECT_EQ(sync_policy_.epsilon(), epsilon_d);

    const double epsilon_s = 10.0;
    sync_policy_.epsilon(epsilon_s);
    EXPECT_EQ(sync_policy_.epsilon(), rclcpp::Duration::from_seconds(epsilon_s));
}

/* TODO
TEST_F(SyncPolicyEpsilonTimeTest1, push) {
    const auto entry = ros_signal::Entry<int>(rclcpp::Time(1), 42);
    sync_policy_.push<0>(storage_, entry);

    EXPECT_FALSE(std::get<0>(storage_.entry_queues_).empty());
    EXPECT_EQ(std::get<0>(storage_.entry_queues_).back(), entry);
}

TEST_F(SyncPolicyEpsilonTimeTest1, emplace) {
    const auto stamp = rclcpp::Time(1, 0);
    const int value = 42;

    sync_policy_.emplace<0>(storage_, stamp, value);

    EXPECT_FALSE(std::get<0>(storage_.entry_queues_).empty());
    EXPECT_EQ(std::get<0>(storage_.entry_queues_).back(), ros_signal::Entry<int>(stamp, value));
}
 */

TEST_F(SyncPolicyEpsilonTimeTest1, count) {
    EXPECT_EQ(storage_.count(), 0);
    EXPECT_EQ(storage_.count<0>(), 0);
    sync_policy_.emplace<0>(storage_, rclcpp::Time(1), 42);
    EXPECT_EQ(storage_.count(), 1);
    EXPECT_EQ(storage_.count<0>(), 1);
}

TEST_F(SyncPolicyEpsilonTimeTest1, empty) {
    EXPECT_TRUE(storage_.empty());
    EXPECT_TRUE(storage_.empty<0>());
    sync_policy_.emplace<0>(storage_, rclcpp::Time(1), 42);
    EXPECT_FALSE(storage_.empty());
    EXPECT_FALSE(storage_.empty<0>());
}

TEST_F(SyncPolicyEpsilonTimeTest1, try_sync) {
    const auto stamp = rclcpp::Time(1, 0);
    const int value = 42;

    sync_policy_.emplace<0>(storage_, stamp, value);

    const auto out = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*std::get<0>(*out), ros_signal::Entry<int>(stamp, value));
}

using SyncPolicyEpsilonTimeTest2 = SyncPolicyEpsilonTimeTest<int, int>;
TEST_F(SyncPolicyEpsilonTimeTest2, try_sync) {
    const auto stamp = rclcpp::Time(1, 0);
    const auto entry1 = ros_signal::Entry<int>(stamp, 1);
    const auto entry2 = ros_signal::Entry<int>(stamp + period_, 2);

    sync_policy_.push<0>(storage_, entry1);
    sync_policy_.push<1>(storage_, entry2);

    const auto out = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*std::get<0>(*out), entry1);
    EXPECT_EQ(*std::get<1>(*out), entry2);

    EXPECT_TRUE(storage_.empty<0>());
    EXPECT_TRUE(storage_.empty<1>());
}

using SyncPolicyEpsilonTimeTest2 = SyncPolicyEpsilonTimeTest<int, int>;
TEST_F(SyncPolicyEpsilonTimeTest2, try_sync_fail) {
    const auto stamp = rclcpp::Time(1, 0);
    const auto entry1 = ros_signal::Entry<int>(stamp, 42);
    const auto entry2 = ros_signal::Entry<int>(stamp + period_ * 1.5, 42);

    sync_policy_.push<0>(storage_, entry1);
    sync_policy_.push<1>(storage_, entry2);

    const auto out = sync_policy_.try_sync(storage_);
    ASSERT_FALSE(out.has_value());
}

TEST_F(SyncPolicyEpsilonTimeTest2, non_trivial_sync) {
    const auto stamp = rclcpp::Time(1, 0);
    const auto entry1_1 = ros_signal::Entry<int>(stamp, 11);
    const auto entry1_2 = ros_signal::Entry<int>(stamp + period_ * .25, 12);
    const auto entry2_1 = ros_signal::Entry<int>(stamp + period_ * .25, 21);
    const auto entry2_2 = ros_signal::Entry<int>(stamp + period_ * .5, 22);

    sync_policy_.push<0>(storage_, entry1_1);
    sync_policy_.push<0>(storage_, entry1_2);
    sync_policy_.push<1>(storage_, entry2_1);
    sync_policy_.push<1>(storage_, entry2_2);

    const auto out = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*std::get<0>(*out), entry1_2);
    EXPECT_EQ(*std::get<1>(*out), entry2_1);
}

using SyncPolicyEpsilonTimeTest3 = SyncPolicyEpsilonTimeTest<int, int, int>;
TEST_F(SyncPolicyEpsilonTimeTest3, disable_and_enable) {
    const auto stamp = rclcpp::Time(1, 0);
    const auto entry1_1 = ros_signal::Entry<int>(stamp, 11);
    const auto entry1_2 = ros_signal::Entry<int>(stamp + period_ * .5, 12);
    const auto entry2_1 = ros_signal::Entry<int>(stamp + period_ * .1, 21);
    const auto entry2_2 = ros_signal::Entry<int>(stamp + period_ * .6, 22);
    const auto entry3_1 = ros_signal::Entry<int>(stamp + period_ * .2, 31);
    const auto entry3_2 = ros_signal::Entry<int>(stamp + period_ * 1.5, 32);

    sync_policy_.push<0>(storage_, entry1_1);
    sync_policy_.push<0>(storage_, entry1_2);
    sync_policy_.push<1>(storage_, entry2_1);
    sync_policy_.push<1>(storage_, entry2_2);
    sync_policy_.push<2>(storage_, entry3_1);
    sync_policy_.push<2>(storage_, entry3_2);

    const auto out1 = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out1.has_value());
    EXPECT_EQ(*std::get<0>(*out1), entry1_1);
    EXPECT_EQ(*std::get<1>(*out1), entry2_1);
    EXPECT_EQ(*std::get<2>(*out1), entry3_1);

    storage_.enabled<2>(false);

    const auto out2 = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out2.has_value());
    EXPECT_EQ(*std::get<0>(*out2), entry1_2);
    EXPECT_EQ(*std::get<1>(*out2), entry2_2);
    EXPECT_EQ(std::get<2>(*out2), ros_signal::Entry<int>::null());

    const auto entry1_3 = ros_signal::Entry<int>(stamp + period_ * 1.2, 12);
    const auto entry2_3 = ros_signal::Entry<int>(stamp + period_ * 1.4, 21);
    sync_policy_.push<0>(storage_, entry1_3);
    sync_policy_.push<1>(storage_, entry2_3);

    storage_.enabled<2>(true);

    const auto out3 = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out3.has_value());
    EXPECT_EQ(*std::get<0>(*out3), entry1_3);
    EXPECT_EQ(*std::get<1>(*out3), entry2_3);
    EXPECT_EQ(*std::get<2>(*out3), entry3_2);
}

using SyncPolicyEpsilonTimeTest4 = SyncPolicyEpsilonTimeTest<int, Value, HeaderValue, StampValue>;
TEST_F(SyncPolicyEpsilonTimeTest4, mixed_entries_1) {
    const auto stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
    auto entry1 = ros_signal::Entry<int>(stamp, 1);
    auto entry2 = ValueEntry(stamp, Value());
    auto entry3 = HeaderValueEntry(HeaderValue());
    auto entry4 = StampValueEntry(StampValue());

    entry2->data = 2;

    entry3->header.set__stamp(stamp).set__frame_id("id");
    entry3->data = 3;

    entry4->stamp = stamp;
    entry4->data = 4;

    sync_policy_.push<0>(storage_, entry1);
    sync_policy_.push<1>(storage_, entry2);
    sync_policy_.push<2>(storage_, entry3);
    sync_policy_.push<3>(storage_, entry4);

    const auto out = sync_policy_.try_sync(storage_);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*std::get<0>(*out), entry1);
    EXPECT_EQ(*std::get<1>(*out), entry2);
    EXPECT_EQ(*std::get<2>(*out), entry3);
}

using SyncPolicyEpsilonTimeTest5 = SyncPolicyEpsilonTimeTest<
    int,
    std::shared_ptr<int>,
    Value,
    std::shared_ptr<Value>,
    HeaderValue,
    std::shared_ptr<HeaderValue>,
    StampValue,
    std::shared_ptr<StampValue>
>;
TEST_F(SyncPolicyEpsilonTimeTest5, mixed_entries_2) {
    const auto stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
    auto entry1 = ros_signal::Entry<int>(stamp, 1);
    auto entry2 = ros_signal::Entry<std::shared_ptr<int> >(stamp, std::make_shared<int>(2));
    auto entry3 = ValueEntry(stamp, Value());
    auto entry4 = ValueSharedPtrEntry(stamp, std::make_shared<Value>());
    auto entry5 = HeaderValueEntry(HeaderValue());
    auto entry6 = HeaderValueSharedPtrEntry(std::make_shared<HeaderValue>());
    auto entry7 = StampValueEntry(StampValue());
    auto entry8 = StampValueSharedPtrEntry(std::make_shared<StampValue>());

    entry3->data = 3;
    entry4->data = 4;
    entry5->header.set__stamp(stamp).set__frame_id("id");
    entry5->data = 5;
    entry6->header.set__stamp(stamp).set__frame_id("id");
    entry6->data = 6;
    entry7->stamp = stamp;
    entry7->data = 7;
    entry8->stamp = stamp;
    entry8->data = 8;

    sync_policy_.push<0>(storage_, entry1);
    sync_policy_.push<1>(storage_, entry2);
    sync_policy_.push<2>(storage_, entry3);
    sync_policy_.push<3>(storage_, entry4);
    sync_policy_.push<4>(storage_, entry5);
    sync_policy_.push<5>(storage_, entry6);
    sync_policy_.push<6>(storage_, entry7);
    sync_policy_.push<7>(storage_, entry8);

    const auto out = sync_policy_.try_sync(storage_);

    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*std::get<0>(*out), entry1);
    EXPECT_EQ(*std::get<1>(*out), entry2);
    EXPECT_EQ(*std::get<2>(*out), entry3);
    EXPECT_EQ(std::get<3>(*out), entry4);
    EXPECT_EQ(*std::get<4>(*out), entry5);
    EXPECT_EQ(std::get<5>(*out), entry6);
    EXPECT_EQ(*std::get<6>(*out), entry7);
    EXPECT_EQ(std::get<7>(*out), entry8);
}
