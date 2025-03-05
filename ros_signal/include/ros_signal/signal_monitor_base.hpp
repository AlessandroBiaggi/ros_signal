#ifndef ROS_SIGNAL_SIGNAL_MONITOR_BASE_HPP
#define ROS_SIGNAL_SIGNAL_MONITOR_BASE_HPP

#include <algorithm>
#include <memory>
#include <type_traits>
#include <vector>

#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include "ros_signal/detail/check_impl.hpp"
#include "ros_signal/signal_monitor_options.hpp"
#include "ros_signal/traits.hpp"

#include <ros_signal_interfaces/msg/signal_monitor_state.hpp>


namespace ros_signal {
    template<typename MsgT, typename AllocT = std::allocator<void>>
    class SignalMonitorBase {
    public:
        using value_type = std::decay_t<MsgT>;

        using allocator_type = AllocT;

        RCLCPP_SMART_PTR_ALIASES_ONLY(SignalMonitorBase)

        template<typename A = std::allocator<void>>
        using state_message_type = ros_signal_interfaces::msg::SignalMonitorState_<A>;

        template<typename... CheckTs>
        explicit SignalMonitorBase(
            const ros_signal::SignalMonitorBaseOptions &options,
            CheckTs... checks
        ) : SignalMonitorBase(
            allocator_type(),
            options,
            std::move(checks)...
        ) {}

        template<typename... CheckTs>
        explicit SignalMonitorBase(
            const allocator_type &alloc,
            const ros_signal::SignalMonitorBaseOptions &options,
            CheckTs... checks
        );

        SignalMonitorBase(const SignalMonitorBase &) = delete;

        SignalMonitorBase(SignalMonitorBase &&) = default;

        SignalMonitorBase &operator=(const SignalMonitorBase &) = delete;

        SignalMonitorBase &operator=(SignalMonitorBase &&) = default;

        virtual ~SignalMonitorBase() = default;

        virtual void signal(const rclcpp::Time &stamp, const value_type &msg);

        void signal(const value_type &msg) {
            signal(ros_signal::get_stamp(msg), msg);
        }

        virtual bool check(const value_type &msg) const;

        [[nodiscard]] bool error() const;

        template<size_t Idx>
        [[nodiscard]] bool error() const;

        [[nodiscard]] bool error(std::size_t idx) const;

        void latch(const rclcpp::Time &stamp);

        void latch(const rclcpp::Time &stamp, std::size_t idx);

        void unlatch(const rclcpp::Time &stamp);

        void unlatch(const rclcpp::Time &stamp, std::size_t idx);

        template<typename A>
        void to_state_message(state_message_type<A> &msg) const noexcept;

        allocator_type get_allocator() {
            return alloc_;
        }

    protected:
        [[nodiscard]] bool can_unlatch() const {
            return can_unlatch_;
        }

        void can_unlatch(const bool can_unlatch) {
            can_unlatch_ = can_unlatch;
        }

        ros_signal::SignalMonitorBaseOptions signal_monitor_options_;

        std::vector<bool, typename std::allocator_traits<allocator_type>::template rebind_alloc<bool>> errors_;

        allocator_type alloc_;

    private:
        bool can_unlatch_;

        using errors_it = typename decltype(errors_)::iterator;
        std::unique_ptr<ros_signal::detail::CheckImplBase<value_type, errors_it>> checks_;

        virtual void latch_handler(const rclcpp::Time &) {
        }

        virtual void unlatch_handler(const rclcpp::Time &) {
        }
    };

    template<typename MsgT, typename AllocT>
	template<typename... CheckTs>
    SignalMonitorBase<MsgT, AllocT>::SignalMonitorBase(
        const allocator_type &alloc,
		const ros_signal::SignalMonitorBaseOptions &options,
        CheckTs... checks
	) : signal_monitor_options_(options.validate()),
        errors_(sizeof...(CheckTs), false, alloc),
        alloc_(alloc),
        can_unlatch_(false),
        checks_(std::make_unique<ros_signal::detail::CheckImpl<value_type, errors_it, CheckTs...>>(
            std::move(checks)...)) {
    }

    template<typename MsgT, typename AllocT>
	void SignalMonitorBase<MsgT, AllocT>::signal(const rclcpp::Time &stamp, const value_type &msg) {
        const auto ok = (*checks_)(msg, errors_.begin(), !can_unlatch());
        if (!ok) {
            latch_handler(stamp);
        } else if (can_unlatch()) {
            unlatch_handler(stamp);
        }
    }

    template<typename MsgT, typename AllocT>
    bool SignalMonitorBase<MsgT, AllocT>::check(const value_type &msg) const {
        return (*checks_)(msg, nullptr, true);
    }

    template<typename MsgT, typename AllocT>
    bool SignalMonitorBase<MsgT, AllocT>::error() const {
        return std::any_of(errors_.begin(), errors_.end(), [](const auto &e) { return e; });
    }

    template<typename MsgT, typename AllocT>
    template<size_t Idx>
    bool SignalMonitorBase<MsgT, AllocT>::error() const {
        return errors_.at(Idx);
    }

    template<typename MsgT, typename AllocT>
	bool SignalMonitorBase<MsgT, AllocT>::error(const std::size_t idx) const {
		return errors_.at(idx);
	}

	template<typename MsgT, typename AllocT>
	void SignalMonitorBase<MsgT, AllocT>::latch(const rclcpp::Time &stamp) {
		errors_.assign(errors_.size(), true);
		latch_handler(stamp);
	}

	template<typename MsgT, typename AllocT>
	void SignalMonitorBase<MsgT, AllocT>::latch(const rclcpp::Time &stamp, const std::size_t idx) {
		errors_.at(idx) = true;
		latch_handler(stamp);
	}

	template<typename MsgT, typename AllocT>
	void SignalMonitorBase<MsgT, AllocT>::unlatch(const rclcpp::Time &stamp) {
		errors_.assign(errors_.size(), false);
		unlatch_handler(stamp);
	}

	template<typename MsgT, typename AllocT>
	void SignalMonitorBase<MsgT, AllocT>::unlatch(const rclcpp::Time &stamp, const std::size_t idx) {
		errors_.at(idx) = false;
        unlatch_handler(stamp);
	}

	template<typename MsgT, typename AllocT>
	template<typename A>
	void SignalMonitorBase<MsgT, AllocT>::to_state_message(state_message_type<A> &msg) const noexcept {
        msg.error = error();
		msg.errors.assign(errors_.cbegin(), errors_.cend());
	}
} // namespace ros_signal

#endif // ROS_SIGNAL_SIGNAL_MONITOR_BASE_HPP
