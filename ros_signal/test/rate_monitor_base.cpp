#include <gtest/gtest.h>

#include <chrono>

#include <ros_signal/rate_monitor_base.hpp>


using namespace std::chrono_literals;


class RateMonitorBaseTest : public ::testing::Test {
protected:
    static constexpr auto window = 50ms;
    static constexpr auto f_nominal = 100.0;

    static constexpr auto f_min = f_nominal * 0.75;
    static constexpr auto f_max = f_nominal * 1.25;

    rclcpp::Time now{0, 0};
    ros_signal::RateMonitorBase<>::SharedPtr monitor = nullptr;

    const rclcpp::Duration period_nominal = rclcpp::Duration::from_seconds(1.0 / f_nominal);

    void SetUp() override {
        now = rclcpp::Time(0, 0);
        monitor = ros_signal::RateMonitorBase<>::make_shared(
            ros_signal::RateMonitorOptions()
            .window(window)
            .f_nominal(f_nominal)
            .f_min(f_min)
            .f_max(f_max)
        );

        EXPECT_FALSE(monitor->check());
        EXPECT_TRUE(monitor->error());

        const auto n_nominal = static_cast<std::size_t>(std::floor(rclcpp::Duration(window).seconds() * f_nominal));

        for (std::size_t i = 0; i < n_nominal; ++i) {
            monitor->signal(now);
            monitor->timer(now);

            now += period_nominal;
        }

        monitor->unlatch(now);
        EXPECT_FALSE(monitor->error());
    }
};

TEST_F(RateMonitorBaseTest, ok_at_f_nominal) {
    for (std::size_t i = 0; i < 1'000; ++i) {
        monitor->signal(now);
        monitor->timer(now);

        EXPECT_TRUE(monitor->check());
        EXPECT_FALSE(monitor->error());

        now += period_nominal;
    }
}

TEST_F(RateMonitorBaseTest, ok_at_f_min) {
    // TODO
}

TEST_F(RateMonitorBaseTest, ok_at_f_max) {
    // TODO
}

TEST_F(RateMonitorBaseTest, under_f_min) {
    // TODO
}

TEST_F(RateMonitorBaseTest, over_f_max) {
    // TODO
}
