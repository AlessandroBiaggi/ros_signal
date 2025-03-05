#include <gtest/gtest.h>

#include <ros_signal/signal.hpp>


struct Msg1 {
    int i1 = 1, i2 = 2;
    bool b1 = true, b2 = false;
    int a1[3] = {1, 2, 3};

    [[nodiscard]] int m1() const { return i1 + i2; }
    [[nodiscard]] int m2() const { return i1 - i2; }

    struct Msg2 {
        long l1 = 3;

        [[nodiscard]] const long &m3() const { return l1; }
    } msg2;
};

const auto msg1_i1 = ros_signal::field<&Msg1::i1>();
const auto msg1_i2 = ros_signal::field<&Msg1::i2>();
const auto msg1_b1 = ros_signal::field<&Msg1::b1>();
const auto msg1_b2 = ros_signal::field<&Msg1::b2>();
const auto msg1_a1 = ros_signal::field<&Msg1::a1>();
const auto msg1_msg2 = ros_signal::field<&Msg1::msg2>();
const auto msg1_msg2_l1 = msg1_msg2.member_field<&Msg1::Msg2::l1>();
const auto msg1_msg2_m3 = msg1_msg2.member_method<&Msg1::Msg2::m3>();

const auto msg1_m1 = ros_signal::method<&Msg1::m1>();
const auto msg1_m2 = ros_signal::method<&Msg1::m2>();

TEST(ros_signal, signal_field) {
    Msg1 t;
    EXPECT_EQ(msg1_i1(t), t.i1);
    EXPECT_EQ(msg1_i2(t), t.i2);
}

TEST(ros_signal, signal_method) {
    Msg1 t;
    EXPECT_EQ(msg1_m1(t), t.m1());
    EXPECT_EQ(msg1_m2(t), t.m2());
}

TEST(ros_signal, signal_cast) {
    Msg1 t;
    const auto y = msg1_i1.cast<long>();
    const auto x = msg1_i1.cast<long>()(t);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(x)>, long>);
    EXPECT_EQ(x, static_cast<long>(t.i1));
}

TEST(ros_signal, signal_member_field) {
    Msg1 t;
    EXPECT_EQ(msg1_msg2_l1(t), t.msg2.l1);
}

TEST(ros_signal, signal_member_method) {
    Msg1 t;
    EXPECT_EQ(msg1_msg2_m3(t), t.msg2.m3());
}

TEST(ros_signal, signal_compose) {
    Msg1 t;
    EXPECT_EQ(ros_signal::identity<Msg1>()
              .compose(ros_signal::field<&Msg1::msg2>())
              .compose(ros_signal::method<&Msg1::Msg2::m3>())(t),
              t.msg2.m3());
}

TEST(ros_signal, signal_unary_plus) {
    Msg1 t;
    EXPECT_EQ((+msg1_i1)(t), +t.i1);
}

TEST(ros_signal, signal_unary_minus) {
    Msg1 t;
    EXPECT_EQ((-msg1_i1)(t), -t.i1);
}

TEST(ros_signal, signal_operator_plus) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 + msg1_m1)(t), t.i1 + t.m1());

    int v = 1;
    EXPECT_EQ((msg1_i1 + v)(t), t.i1 + v);
    EXPECT_EQ((v + msg1_i2)(t), v + t.i2);

    EXPECT_EQ((msg1_i2 + 2)(t), t.i2 + 2);
    EXPECT_EQ((1 + msg1_m1)(t), 1 + t.m1());
}

TEST(ros_signal, signal_operator_minus) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 - msg1_m1)(t), t.i1 - t.m1());

    int v = 1;
    EXPECT_EQ((msg1_i1 - v)(t), t.i1 - v);
    EXPECT_EQ((v - msg1_i2)(t), v - t.i2);

    EXPECT_EQ((msg1_i2 - 2)(t), t.i2 - 2);
    EXPECT_EQ((1 - msg1_m1)(t), 1 - t.m1());
}

TEST(ros_signal, signal_operator_multiplies) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 * msg1_m1)(t), t.i1 * t.m1());

    int v = 1;
    EXPECT_EQ((msg1_i1 * v)(t), t.i1 * v);
    EXPECT_EQ((v * msg1_i2)(t), v * t.i2);

    EXPECT_EQ((msg1_i2 * 2)(t), t.i2 * 2);
    EXPECT_EQ((1 * msg1_m1)(t), 1 * t.m1());
}

TEST(ros_signal, signal_operator_divides) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 / msg1_m1)(t), t.i1 / t.m1());

    int v = 1;
    EXPECT_EQ((msg1_i1 / v)(t), t.i1 / v);
    EXPECT_EQ((v / msg1_i2)(t), v / t.i2);

    EXPECT_EQ((msg1_i2 / 2)(t), t.i2 / 2);
    EXPECT_EQ((1 / msg1_m1)(t), 1 / t.m1());
}

TEST(ros_signal, signal_operator_modulus) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 % msg1_m1)(t), t.i1 % t.m1());

    int v = 1;
    EXPECT_EQ((msg1_i1 % v)(t), t.i1 % v);
    EXPECT_EQ((v % msg1_i2)(t), v % t.i2);

    EXPECT_EQ((msg1_i2 % 2)(t), t.i2 % 2);
    EXPECT_EQ((1 % msg1_m1)(t), 1 % t.m1());
}

TEST(ros_signal, signal_operator_bit_not) {
    Msg1 t;
    EXPECT_EQ((~msg1_i1)(t), ~t.i1);
}

TEST(ros_signal, signal_operator_bit_and) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 & msg1_i2)(t), t.i1 & t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 & v)(t), t.i1 & v);
    EXPECT_EQ((v & msg1_i2)(t), v & t.i2);

    EXPECT_EQ((msg1_i2 & 2)(t), t.i2 & 2);
    EXPECT_EQ((1 & msg1_i1)(t), 1 & t.i1);
}

TEST(ros_signal, signal_operator_bit_or) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 | msg1_i2)(t), t.i1 | t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 | v)(t), t.i1 | v);
    EXPECT_EQ((v | msg1_i2)(t), v | t.i2);

    EXPECT_EQ((msg1_i2 | 2)(t), t.i2 | 2);
    EXPECT_EQ((1 | msg1_i1)(t), 1 | t.i1);
}

TEST(ros_signal, signal_operator_bit_xor) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 ^ msg1_i2)(t), t.i1 ^ t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 ^ v)(t), t.i1 ^ v);
    EXPECT_EQ((v ^ msg1_i2)(t), v ^ t.i2);

    EXPECT_EQ((msg1_i2 ^ 2)(t), t.i2 ^ 2);
    EXPECT_EQ((1 ^ msg1_i1)(t), 1 ^ t.i1);
}

TEST(ros_signal, signal_operator_left_shift) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 << msg1_i2)(t), t.i1 << t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 << v)(t), t.i1 << v);
    EXPECT_EQ((v << msg1_i2)(t), v << t.i2);

    EXPECT_EQ((msg1_i2 << 2)(t), t.i2 << 2);
    EXPECT_EQ((1 << msg1_i1)(t), 1 << t.i1);
}

TEST(ros_signal, signal_operator_right_shift) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 >> msg1_i2)(t), t.i1 >> t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 >> v)(t), t.i1 >> v);
    EXPECT_EQ((v >> msg1_i2)(t), v >> t.i2);

    EXPECT_EQ((msg1_i2 >> 2)(t), t.i2 >> 2);
    EXPECT_EQ((1 >> msg1_i1)(t), 1 >> t.i1);
}

TEST(ros_signal, signal_operator_logical_not) {
    Msg1 t;
    EXPECT_EQ((!msg1_b1)(t), !t.b1);
}

TEST(ros_signal, signal_operator_logical_and) {
    Msg1 t;
    EXPECT_EQ((msg1_b1 && msg1_b2)(t), t.b1 && t.b2);

    const bool v = true;
    EXPECT_EQ((msg1_b1 && v)(t), t.b1 && v);
    EXPECT_EQ((v && msg1_b2)(t), v && t.b2);

    EXPECT_EQ((msg1_b2 && false)(t), t.b2 && false);
    EXPECT_EQ((true && msg1_b1)(t), true && t.b1);
}

TEST(ros_signal, signal_operator_logical_or) {
    Msg1 t;
    EXPECT_EQ((msg1_b1 || msg1_b2)(t), t.b1 || t.b2);

    const bool v = true;
    EXPECT_EQ((msg1_b1 || v)(t), t.b1 || v);
    EXPECT_EQ((v || msg1_b2)(t), v || t.b2);

    EXPECT_EQ((msg1_b2 || false)(t), t.b2 || false);
    EXPECT_EQ((true || msg1_b1)(t), true || t.b1);
}

TEST(ros_signal, signal_operator_equal) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 == msg1_i2)(t), t.i1 == t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 == v)(t), t.i1 == v);
    EXPECT_EQ((v == msg1_i2)(t), v == t.i2);

    EXPECT_EQ((msg1_i2 == 2)(t), t.i2 == 2);
    EXPECT_EQ((1 == msg1_i1)(t), 1 == t.i1);
}

TEST(ros_signal, signal_operator_not_equal) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 != msg1_i2)(t), t.i1 != t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 != v)(t), t.i1 != v);
    EXPECT_EQ((v != msg1_i2)(t), v != t.i2);

    EXPECT_EQ((msg1_i2 != 2)(t), t.i2 != 2);
    EXPECT_EQ((1 != msg1_i1)(t), 1 != t.i1);
}

TEST(ros_signal, signal_operator_less) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 < msg1_i2)(t), t.i1 < t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 < v)(t), t.i1 < v);
    EXPECT_EQ((v < msg1_i2)(t), v < t.i2);

    EXPECT_EQ((msg1_i2 < 2)(t), t.i2 < 2);
    EXPECT_EQ((1 < msg1_i1)(t), 1 < t.i1);
}

TEST(ros_signal, signal_operator_greater) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 > msg1_i2)(t), t.i1 > t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 > v)(t), t.i1 > v);
    EXPECT_EQ((v > msg1_i2)(t), v > t.i2);

    EXPECT_EQ((msg1_i2 > 2)(t), t.i2 > 2);
    EXPECT_EQ((1 > msg1_i1)(t), 1 > t.i1);
}

TEST(ros_signal, signal_operator_less_equal) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 <= msg1_i2)(t), t.i1 <= t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 <= v)(t), t.i1 <= v);
    EXPECT_EQ((v <= msg1_i2)(t), v <= t.i2);

    EXPECT_EQ((msg1_i2 <= 2)(t), t.i2 <= 2);
    EXPECT_EQ((1 <= msg1_i1)(t), 1 <= t.i1);
}

TEST(ros_signal, signal_operator_greater_equal) {
    Msg1 t;
    EXPECT_EQ((msg1_i1 >= msg1_i2)(t), t.i1 >= t.i2);

    const int v = 1;
    EXPECT_EQ((msg1_i1 >= v)(t), t.i1 >= v);
    EXPECT_EQ((v >= msg1_i2)(t), v >= t.i2);

    EXPECT_EQ((msg1_i2 >= 2)(t), t.i2 >= 2);
    EXPECT_EQ((1 >= msg1_i1)(t), 1 >= t.i1);
}

TEST(ros_signal, signal_operator_subscript) {
    Msg1 t;
    EXPECT_EQ((msg1_a1[msg1_i1])(t), t.a1[t.i1]);

    const int v = 1;
    EXPECT_EQ((msg1_a1[v])(t), t.a1[v]);

    EXPECT_EQ((msg1_a1[0])(t), t.a1[0]);
}
