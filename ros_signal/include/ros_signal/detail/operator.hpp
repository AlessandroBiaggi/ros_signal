#ifndef ROS_SIGNAL_DETAIL_OPERATOR_HPP
#define ROS_SIGNAL_DETAIL_OPERATOR_HPP

#include <deque>
#include <functional>
#include <type_traits>


namespace ros_signal::detail {

    template<typename From, typename To, std::enable_if_t<std::is_convertible_v<From, To>, int> = 0>
    struct cast {
        [[nodiscard]] constexpr To operator()(const From &from) const {
            return static_cast<To>(from);
        }
    };

    struct unary_plus {
        template<typename T>
        [[nodiscard]] constexpr auto operator()(const T &arg) const {
            return +arg;
        }
    };

    struct unary_minus {
        template<typename T>
        [[nodiscard]] constexpr auto operator()(const T &arg) const {
            return -arg;
        }
    };

    using plus = std::plus<void>;
    using minus = std::minus<void>;
    using multiplies = std::multiplies<void>;
    using divides = std::divides<void>;
    using modulus = std::modulus<void>;

    using bit_not = std::bit_not<void>;
    using bit_and = std::bit_and<void>;
    using bit_or = std::bit_or<void>;
    using bit_xor = std::bit_xor<void>;

    struct bit_shift_left {
        template<typename T, typename U>
        [[nodiscard]] constexpr auto operator()(const T &lhs, const U &rhs) const {
            return lhs << rhs;
        }
    };

    struct bit_shift_right {
        template<typename T, typename U>
        [[nodiscard]] constexpr auto operator()(const T &lhs, const U &rhs) const {
            return lhs >> rhs;
        }
    };

    using logical_not = std::logical_not<void>;
    using logical_and = std::logical_and<void>;
    using logical_or = std::logical_or<void>;

    using equal_to = std::equal_to<void>;
    using not_equal_to = std::not_equal_to<void>;
    using less = std::less<void>;
    using greater = std::greater<void>;
    using less_equal = std::less_equal<void>;
    using greater_equal = std::greater_equal<void>;

    struct subscript {
        template<typename T, typename U>
        [[nodiscard]] constexpr const auto &operator()(const T &lhs, const U &rhs) const {
            return lhs[rhs];
        }
    };

    struct indirection {
        template<typename T>
        [[nodiscard]] constexpr auto operator()(const T &arg) const {
            return *arg;
        }
    };

    template<typename T, size_t N>
    struct moving_average_1 {
        using value_type = std::decay_t<T>;
        static_assert(N > 1);

        [[nodiscard]] constexpr auto operator()(const T &arg) const {
            if (size == N) {
                const auto prev = window[index];
                window[index] = arg;
                if constexpr (std::is_floating_point_v<value_type>) {
                    value += (arg - prev) / size;
                } else {
                    value = (value * size - prev + arg) / size;
                }
                index = (index + 1) % N;
            } else if (size > 0) {
                window[index] = arg;
                if constexpr (std::is_floating_point_v<value_type>) {
                    constexpr auto N_f = static_cast<value_type>(N);
                    value = value * (N_f / (N_f + 1.)) + arg / (N_f + 1.);
                } else {
                    value = (value * size + arg) / (size + 1);
                }
                ++index;
                ++size;
            } else {
                window[0] = value = arg;
                index = size = 1;
            }
            return value;
        }

    private:
        mutable value_type window[N];
        mutable value_type value;
        mutable size_t index = 0;
        mutable size_t size = 0;
    };

    template<typename T>
    struct moving_average_1<T, 1> {
        using value_type = std::decay_t<T>;
        [[nodiscard]] constexpr auto operator()(const T &arg) const { return arg; }
    };

    template<typename T, typename AllocT = std::allocator<void>>
    struct moving_average_2 {
        using value_type = std::decay_t<T>;
        using allocator_type = typename std::allocator_traits<AllocT>::template rebind_alloc<value_type>;

        moving_average_2(const size_t size, const AllocT &alloc)
            : size(size), window(alloc) {}

        [[nodiscard]] constexpr auto operator()(const T &arg) const {
            if (window.size() == size) {
                const auto prev = window.front();
                window.pop_front();
                window.push_back(arg);

                if constexpr (std::is_floating_point_v<value_type>) {
                    value += (arg - prev) / size;
                } else {
                    value = (value * size - prev + arg) / size;
                }
            } else if (window.size() > 0) {
                if constexpr (std::is_floating_point_v<value_type>) {
                    const auto N_f = static_cast<T>(window.size());
                    value = value * (N_f / (N_f + 1.)) + arg / (N_f + 1.);
                } else {
                    value = (value * size + arg) / (size + 1);
                }
                window.push_back(arg);
            }
            return value;
        }

    private:
        const size_t size;
        mutable std::deque<value_type, allocator_type> window;
        mutable value_type value;
    };
} // namespace ros_signal::detail

#endif // ROS_SIGNAL_DETAIL_OPERATOR_HPP
