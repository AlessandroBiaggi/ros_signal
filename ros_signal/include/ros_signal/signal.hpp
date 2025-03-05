#ifndef ROS_SIGNAL_SIGNAL_HPP
#define ROS_SIGNAL_SIGNAL_HPP

#include <tuple>
#include <type_traits>

#include "ros_signal/detail/operator.hpp"


namespace ros_signal {
    template<typename, typename, typename = void>
    struct signal;

    template<typename S>
    struct is_signal {
    private:
        template<typename T, typename U>
        static std::true_type test(const signal<T, U> *);

        static std::false_type test(...);

    public:
        static constexpr bool value = decltype(test(std::declval<const std::remove_reference_t<S> *>()))::value;
    };

    template<typename T>
    constexpr bool is_signal_v = is_signal<T>::value;

    template<typename T>
    struct identity final : signal<const T &, T, identity<T> > {
        using typename signal<const T &, T>::argument_type;
        using typename signal<const T &, T>::result_type;

        constexpr identity() = default;

        ~identity() override = default;

        [[nodiscard]] constexpr result_type operator()(const argument_type &arg) const override {
            return arg;
        }
    };

    template<typename T, typename U>
    struct constant final : signal<const T &, U, constant<T, U> > {
        using typename signal<const T &, U>::argument_type;
        using typename signal<const T &, U>::result_type;

        using type = std::decay_t<T>;
        const type v;

        explicit constexpr constant(const type &v) : v(v) {
        }

        explicit constexpr constant(type &&v) : v(std::forward<type>(v)) {
        }

        ~constant() override = default;

        [[nodiscard]] result_type operator()(const argument_type &) const override {
            return v;
        }
    };

    template<typename T, typename U>
    struct reference final : signal<const T &, U, reference<T, U> > {
        using typename signal<const T &, U>::argument_type;
        using typename signal<const T &, U>::result_type;

        using type = std::decay_t<T>;
        const type &v;

        constexpr explicit reference(const type &v) : v(v) {
        }

        explicit reference(type &&) = delete;

        ~reference() override = default;

        [[nodiscard]] constexpr result_type operator()(const argument_type &) const override {
            return v;
        }
    };

    template<auto>
    struct field {
    };

    template<typename T, typename U, T U::*FieldPtr>
    struct field<FieldPtr> final : signal<const T &, U, field<FieldPtr> > {
        using typename signal<const T &, U>::argument_type;
        using typename signal<const T &, U>::result_type;

        constexpr field() = default;

        ~field() override = default;

        [[nodiscard]] constexpr result_type operator()(const argument_type &arg) const override {
            return arg.*FieldPtr;
        }
    };

    template<auto>
    struct method {
    };

    template<typename T, typename U, T (U::*MethodPtr)() const>
    struct method<MethodPtr> final : signal<T, U, method<MethodPtr> > {
        using typename signal<T, U>::argument_type;
        using typename signal<T, U>::result_type;

        constexpr method() = default;

        ~method() override = default;

        [[nodiscard]] constexpr result_type operator()(const argument_type &arg) const override {
            return (arg.*MethodPtr)();
        }
    };

    template<typename T, typename U, typename O, typename... A>
    struct signal_operator final : signal<T, U, signal_operator<T, U, O, A...> > {
        using typename signal<T, U>::argument_type;
        using typename signal<T, U>::result_type;

        using operator_type = O;

        operator_type o;
        std::tuple<A...> a;

        explicit constexpr signal_operator(const operator_type &o, const A &... a) : o(o), a(a...) {
        }

        ~signal_operator() override = default;

        [[nodiscard]] constexpr result_type operator()(const argument_type &arg) const override {
            return o(std::get<A>(a)(arg)...);
        }
    };

    template<typename T, typename U>
    struct signal<T, U, void> {
        using argument_type = std::remove_cv_t<std::remove_reference_t<U> >;
        using result_type = std::conditional_t<std::is_array_v<T> or std::is_function_v<T>, std::decay_t<T>, T>;

        constexpr signal() = default;

        virtual ~signal() = default;

        [[nodiscard]] virtual result_type operator()(const argument_type &) const = 0;
    };

    template<typename T, typename U, typename D>
    struct signal : signal<T, U> {
        using typename signal<T, U>::argument_type;
        using typename signal<T, U>::result_type;

        constexpr signal() { static_assert(std::is_base_of_v<signal, D>); }

        ~signal() override = default;

        template<typename S>
        [[nodiscard]] constexpr auto cast() const {
            using V = typename signal<S, U>::result_type;
            return signal_operator<V, U, detail::cast<T, V>, D>(
                detail::cast<T, V>(), *dynamic_cast<const D *>(this));
        }

        template<auto MemberFieldPtr>
        [[nodiscard]] constexpr auto member_field() const {
            using field = field<MemberFieldPtr>;
            using V = typename field::result_type;
            return signal_operator<V, U, field, D>(
                field(), *dynamic_cast<const D *>(this));
        }

        template<auto MemberMethodPtr>
        [[nodiscard]] constexpr auto member_method() const {
            using method = method<MemberMethodPtr>;
            using V = typename method::result_type;
            return signal_operator<V, U, method, D>(
                method(), *dynamic_cast<const D *>(this));
        }

        template<typename R>
        [[nodiscard]] constexpr auto compose(const R &inner) const {
            return signal_operator<typename R::result_type, U, R, D>(
                inner, *dynamic_cast<const D *>(this));
        }

        template<size_t N>
        [[nodiscard]] constexpr auto moving_average() const {
            return signal_operator<T, U, detail::moving_average_1<T, N>, D>(
                detail::moving_average_1<T, N>(), *dynamic_cast<const D *>(this));
        }

        template<typename AllocT = std::allocator<T>>
        [[nodiscard]] constexpr auto moving_average(const size_t N, const AllocT &alloc = AllocT()) const {
            return signal_operator<T, U, detail::moving_average_2<T, AllocT>, D>(
                detail::moving_average_2<T, AllocT>(N, alloc), *dynamic_cast<const D *>(this));
        }

        [[nodiscard]] constexpr auto operator+() const {
            using V = decltype(+std::declval<T>());
            return signal_operator<V, U, detail::unary_plus, D>(
                detail::unary_plus(), *dynamic_cast<const D *>(this));
        }

        [[nodiscard]] constexpr auto operator-() const {
            using V = decltype(-std::declval<T>());
            return signal_operator<V, U, detail::unary_minus, D>(
                detail::unary_minus(), *dynamic_cast<const D *>(this));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator+(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() + std::declval<S>());
            return signal_operator<V, U, detail::plus, D, R>(
                detail::plus(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator-(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() - std::declval<S>());
            return signal_operator<V, U, detail::minus, D, R>(
                detail::minus(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator*(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() * std::declval<S>());
            return signal_operator<V, U, detail::multiplies, D, R>(
                detail::multiplies(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator/(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() / std::declval<S>());
            return signal_operator<V, U, detail::divides, D, R>(
                detail::divides(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator%(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() % std::declval<S>());
            return signal_operator<V, U, detail::modulus, D, R>(
                detail::modulus(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        [[nodiscard]] constexpr auto operator~() const {
            using V = decltype(~std::declval<T>());
            return signal_operator<V, U, detail::bit_not, D>(
                detail::bit_not(), *dynamic_cast<const D *>(this));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator&(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() & std::declval<S>());
            return signal_operator<V, U, detail::bit_and, D, R>(
                detail::bit_and(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator|(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() | std::declval<S>());
            return signal_operator<V, U, detail::bit_or, D, R>(
                detail::bit_or(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator^(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() ^ std::declval<S>());
            return signal_operator<V, U, detail::bit_xor, D, R>(
                detail::bit_xor(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator<<(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() << std::declval<S>());
            return signal_operator<V, U, detail::bit_shift_left, D, R>(
                detail::bit_shift_left(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator>>(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() >> std::declval<S>());
            return signal_operator<V, U, detail::bit_shift_right, D, R>(
                detail::bit_shift_right(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        [[nodiscard]] constexpr auto operator!() const {
            using V = decltype(!std::declval<T>());
            return signal_operator<V, U, detail::logical_not, D>(
                detail::logical_not(), *dynamic_cast<const D *>(this));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator&&(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() && std::declval<S>());
            return signal_operator<V, U, detail::logical_and, D, R>(
                detail::logical_and(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator||(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() || std::declval<S>());
            return signal_operator<V, U, detail::logical_or, D, R>(
                detail::logical_or(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator==(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() == std::declval<S>());
            return signal_operator<V, U, detail::equal_to, D, R>(
                detail::equal_to(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator!=(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() != std::declval<S>());
            return signal_operator<V, U, detail::not_equal_to, D, R>(
                detail::not_equal_to(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator<(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() < std::declval<S>());
            return signal_operator<V, U, detail::less, D, R>(
                detail::less(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator>(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() > std::declval<S>());
            return signal_operator<V, U, detail::greater, D, R>(
                detail::greater(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator<=(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() <= std::declval<S>());
            return signal_operator<V, U, detail::less_equal, D, R>(
                detail::less_equal(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator>=(const signal<S, U, R> &rhs) const {
            using V = decltype(std::declval<T>() >= std::declval<S>());
            return signal_operator<V, U, detail::greater_equal, D, R>(
                detail::greater_equal(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&rhs));
        }

        template<typename S, typename R>
        [[nodiscard]] constexpr auto operator[](const signal<S, U, R> &i) const {
            using V = decltype(std::declval<T>()[std::declval<S>()]);
            return signal_operator<const V &, U, detail::subscript, D, R>(
                detail::subscript(), *dynamic_cast<const D *>(this), *dynamic_cast<const R *>(&i));
        }

        template<typename S, std::enable_if_t<not std::is_rvalue_reference_v<S>, int>  = 0,
            typename = decltype(std::declval<T>()[std::declval<S>()])>
        [[nodiscard]] constexpr auto operator[](const S &i) const {
            return operator[](reference<S, U>(i));
        }

        template<typename S, std::enable_if_t<not std::is_lvalue_reference_v<S>, int>  = 0,
            typename = decltype(std::declval<T>()[std::declval<S>()])>
        [[nodiscard]] constexpr auto operator[](S &&i) const {
            return operator[](constant<S, U>(i));
        }

        [[nodiscard]] constexpr auto operator*() const {
            using V = decltype(*std::declval<T>());
            return signal_operator<V, U, detail::indirection, D>(
                detail::indirection(), *dynamic_cast<const D *>(this));
        }
    };
} // namespace ros_signal

#define ROS_SIGNAL_BINARY_OPERATOR_IMPL(op) \
template<typename T, typename U, typename D, typename S, \
        std::enable_if_t<not ros_signal::is_signal_v<S> and not std::is_rvalue_reference_v<S>, int> = 0> \
[[nodiscard]] constexpr auto operator op(const ros_signal::signal<T, U, D> &lhs, const S &rhs) { \
    return lhs.operator op(ros_signal::reference<S, U>(rhs)); \
} \
template<typename T, typename U, typename D, typename S, \
        std::enable_if_t<not ros_signal::is_signal_v<S> and not std::is_rvalue_reference_v<S>, int> = 0> \
[[nodiscard]] constexpr auto operator op(const S &lhs, const ros_signal::signal<T, U, D> &rhs) { \
    return ros_signal::reference<S, U>(lhs).operator op(rhs); \
} \
template<typename T, typename U, typename D, typename S, \
        std::enable_if_t<not ros_signal::is_signal_v<S> and not std::is_lvalue_reference_v<S>, int> = 0> \
[[nodiscard]] constexpr auto operator op(const ros_signal::signal<T, U, D> &lhs, S &&rhs) { \
    return lhs.operator op(ros_signal::constant<S, U>(std::forward<S>(rhs))); \
} \
template<typename T, typename U, typename D, typename S, \
        std::enable_if_t<not ros_signal::is_signal_v<S> and not std::is_lvalue_reference_v<S>, int> = 0> \
[[nodiscard]] constexpr auto operator op(S &&lhs, const ros_signal::signal<T, U, D> &rhs) { \
    return ros_signal::constant<S, U>(std::forward<S>(lhs)).operator op(rhs); \
}

ROS_SIGNAL_BINARY_OPERATOR_IMPL(+)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(-)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(*)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(/)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(%)

ROS_SIGNAL_BINARY_OPERATOR_IMPL(&)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(|)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(^)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(<<)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(>>)

ROS_SIGNAL_BINARY_OPERATOR_IMPL(&&)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(||)

ROS_SIGNAL_BINARY_OPERATOR_IMPL(==)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(!=)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(<)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(>)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(<=)
ROS_SIGNAL_BINARY_OPERATOR_IMPL(>=)

#undef ROS_SIGNAL_BINARY_OPERATOR_IMPL

#endif // ROS_SIGNAL_SIGNAL_HPP
