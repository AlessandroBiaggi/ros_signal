#ifndef ROS_SIGNAL_TEST_SYNC_POLICY_TEST_HPP
#define ROS_SIGNAL_TEST_SYNC_POLICY_TEST_HPP

#include <memory>
#include <type_traits>

#include <gtest/gtest.h>


template<typename SyncPolicyT>
class SyncPolicyTest : public ::testing::Test {
protected:
    using sync_policy_type = std::decay_t<SyncPolicyT>;
    using storage_type = typename sync_policy_type::template storage_type<std::allocator<void>>;

    void SetUp() override {
        ::testing::Test::SetUp();
        sync_policy_.clear(storage_, rclcpp::Time::max());
    }

    rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1);
    sync_policy_type sync_policy_{period_};
    storage_type storage_;
};

#endif // ROS_SIGNAL_TEST_SYNC_POLICY_TEST_HPP
