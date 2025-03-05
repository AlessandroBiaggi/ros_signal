#ifndef ROS_SIGNAL_DETAIL_CHECK_IMPL_HPP
#define ROS_SIGNAL_DETAIL_CHECK_IMPL_HPP

#include <cstddef>

#include <functional>
#include <iterator>
#include <tuple>
#include <type_traits>


namespace ros_signal::detail {
    template<typename MsgT, typename IterT>
    class CheckImplBase {
    public:
        virtual ~CheckImplBase() = default;

        virtual bool operator()(const MsgT &, IterT) = 0;

        virtual bool operator()(const MsgT &, std::nullptr_t) = 0;
    };

    template<typename MsgT, typename IterT, typename... CheckTs>
    class CheckImpl : public CheckImplBase<MsgT, IterT> {
    public:
        static_assert((std::is_invocable_r_v<bool, std::decay_t<CheckTs>, const MsgT &> && ...),
                      "CheckTs must be invocable with const std::decay_t<MsgT> & and return bool");

        static constexpr auto check_count = sizeof...(CheckTs);
        using check_tuple_type = std::tuple<std::decay_t<CheckTs>...>;

        explicit CheckImpl(CheckTs... args) : checks_(std::move(args)...) {
        }

        ~CheckImpl() override = default;

        bool operator()(const MsgT &msg, IterT it) override {
            return check_impl<0, IterT>(msg, it);
        }

        bool operator()(const MsgT &msg, std::nullptr_t) override {
            return check_impl<0, std::nullptr_t>(msg, nullptr);
        }

    private:
        check_tuple_type checks_;

        template<std::size_t Idx, typename IterU>
        bool check_impl(const MsgT &msg, IterU it);
    };

    template<typename MsgT, typename IterT, typename... CheckTs>
    template<std::size_t Idx, typename IterU>
    bool CheckImpl<MsgT, IterT, CheckTs...>::check_impl(const MsgT &msg, IterU it) {
        static_assert(std::is_same_v<IterU, IterT> || std::is_same_v<IterU, std::nullptr_t>);
        static_assert(0 <= Idx && Idx <= check_count);

        if constexpr (Idx < check_count) {
            const auto i_ok = std::invoke(std::get<Idx>(checks_), msg);
            const auto n_ok = [&] {
                if constexpr (std::is_same_v<IterT, IterU>) {
                    *it = !i_ok;
                    return check_impl<Idx + 1>(msg, std::next(it));
                } else {
                    return check_impl<Idx + 1>(msg, nullptr);
                }
            }();

            return i_ok && n_ok;
        } else {
            return true;
        }
    }
}// namespace ros_signal::detail

#endif//ROS_SIGNAL_DETAIL_CHECK_IMPL_HPP
