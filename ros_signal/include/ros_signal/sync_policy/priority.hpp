#ifndef ROS_SIGNAL_SYNC_POLICY_PRIORITY_HPP
#define ROS_SIGNAL_SYNC_POLICY_PRIORITY_HPP

#include <cstddef>

#include <bitset>
#include <deque>
#include <variant>

#include <ros_signal_interfaces/msg/priority_state.hpp>

#include "ros_signal/entry.hpp"
#include "ros_signal/macros.hpp"


namespace ros_signal::sync_policy {
    template<typename... MsgTs>
    class Priority {
    public:
        static constexpr std::size_t message_count = sizeof...(MsgTs);

        template<typename T>
        using entry_type = Entry<T>;

        template<std::size_t I>
        using element_type = std::tuple_element_t<I, std::tuple<typename entry_type<MsgTs>::element_type...>>;

        using output_type = std::variant<entry_type<MsgTs>...>;

        template<typename AllocT>
        class storage_type;

        explicit Priority(const rclcpp::Duration &delta = rclcpp::Duration::from_seconds(0));

        explicit Priority(const double delta_s) : Priority(rclcpp::Duration::from_seconds(delta_s)) {
        }

        Priority(const Priority &) = default;

        Priority(Priority &&) = default;

        ~Priority() = default;

        Priority &operator=(const Priority &) = default;

        Priority &operator=(Priority &&) = default;

        [[nodiscard]] rclcpp::Duration delta() const;

        void delta(const rclcpp::Duration &delta);

        void delta(const double delta_s) {
            delta(rclcpp::Duration::from_seconds(delta_s));
        }

        template<std::size_t I, typename AllocT>
        void push(storage_type<AllocT> &storage, const entry_type<element_type<I>> &entry) const;

        template<std::size_t I, typename AllocT>
        void push(storage_type<AllocT> &storage, entry_type<element_type<I>> &&entry) const;

        template<std::size_t I, typename AllocT, typename... ArgTs>
        void emplace(storage_type<AllocT> &storage, const rclcpp::Time &stamp, ArgTs &&...args) const;

        template<typename AllocT>
        std::size_t clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
            return clear_helper(storage, up_to);
        }

        template<std::size_t I, typename AllocT>
        std::size_t clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const;

        template<typename AllocT>
        std::optional<output_type> try_sync(storage_type<AllocT> &storage) const;

    private:
        static constexpr auto index_sequence = std::make_index_sequence<message_count>();

        template<std::size_t I = 0, typename AllocT>
        void try_sync_helper(storage_type<AllocT> &storage, std::optional<output_type> &out) const;

        template<std::size_t I = 0, typename AllocT>
        std::size_t clear_helper(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const;

        rclcpp::Duration delta_;
    };

    template<typename... MsgTs>
    Priority<MsgTs...>::Priority(const rclcpp::Duration &delta)
        : delta_(delta) {
    }

    template<typename... MsgTs>
    rclcpp::Duration Priority<MsgTs...>::delta() const {
        return delta_;
    }

    template<typename... MsgTs>
    void Priority<MsgTs...>::delta(const rclcpp::Duration &delta) {
        delta_ = delta;
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void Priority<MsgTs...>::push(storage_type<AllocT> &storage, const entry_type<element_type<I>> &entry) const {
        static_assert(0 <= I && I < message_count);

        // Pre-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto pre_count = storage.count();
        const auto pre_count_I = storage.template count<I>();

        if (storage.template enabled<I>()) {
            auto &eq = std::get<I>(storage.entry_queues_);

            if (eq.empty() || rclcpp::Time(eq.back().stamp()) <= rclcpp::Time(entry.stamp())) {
                eq.push_back(entry);
            } else {
                auto pos = std::upper_bound(eq.begin(), eq.end(), entry, LessEntryStamp<element_type<I>>());
                eq.insert(pos, entry);
            }
        }

        // Post-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto post_count = storage.count();
        const auto post_count_I = storage.template count<I>();
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || pre_count == post_count);
        ros_signal_assert((storage.template enabled<I>() && pre_count_I + 1 == post_count_I && !storage.template empty<I>()) || pre_count_I == post_count_I);
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void Priority<MsgTs...>::push(storage_type<AllocT> &storage, entry_type<element_type<I>> &&entry) const {
        static_assert(0 <= I && I < message_count);

        // Pre-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto pre_count = storage.count();
        const auto pre_count_I = storage.template count<I>();

        if (storage.template enabled<I>()) {
            auto &eq = std::get<I>(storage.entry_queues_);

            if (eq.empty() || rclcpp::Time(eq.back().stamp()) <= rclcpp::Time(entry.stamp())) {
                eq.push_back(std::forward<entry_type<element_type<I>>>(entry));
            } else {
                auto pos = std::upper_bound(eq.begin(), eq.end(), entry, LessEntryStamp<element_type<I>>());
                eq.insert(pos, std::forward<entry_type<element_type<I>>>(entry));
            }
        }

        // Post-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto post_count = storage.count();
        const auto post_count_I = storage.template count<I>();
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || pre_count == post_count);
        ros_signal_assert((storage.template enabled<I>() && (pre_count_I + 1 == post_count_I && !storage.template empty<I>())) || pre_count_I == post_count_I);
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT, typename... ArgTs>
    void Priority<MsgTs...>::emplace(
            storage_type<AllocT> &storage, const rclcpp::Time &stamp, ArgTs &&...args) const {
        static_assert(0 <= I && I < message_count);

        // Pre-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto pre_count = storage.count();
        const auto pre_count_I = storage.template count<I>();

        if (storage.template enabled<I>()) {
            auto &eq = std::get<I>(storage.entry_queues_);
            if (eq.empty() || rclcpp::Time(eq.back().stamp()) <= stamp) {
                eq.emplace_back(stamp, std::forward<ArgTs>(args)...);
            } else {
                auto pos = std::upper_bound(eq.begin(), eq.end(), stamp, LessEntryStamp<element_type<I>>());
                eq.emplace(pos, stamp, std::forward<ArgTs>(args)...);
            }
        }

        // Post-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto post_count = storage.count();
        const auto post_count_I = storage.template count<I>();
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || pre_count == post_count);
        ros_signal_assert((storage.template enabled<I>() && pre_count_I + 1 == post_count_I) || pre_count_I == post_count_I);
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    std::size_t Priority<MsgTs...>::clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
        static_assert(0 <= I && I < message_count);

        // Pre-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto pre_count = storage.template count<I>();

        auto &eq = std::get<I>(storage.entry_queues_);

        auto pos = std::upper_bound(eq.begin(), eq.end(), up_to, LessEntryStamp<element_type<I>>());
        const auto erased = std::distance(eq.begin(), pos);
        eq.erase(eq.begin(), pos);

        // Post-condition(s)
        ros_signal_assert(storage.template is_sorted_helper<I>());
        const auto post_count = storage.template count<I>();
        ros_signal_assert(pre_count - erased == post_count);
        ros_signal_assert(std::all_of(
                eq.begin(), eq.end(),
                [&up_to](const auto &e) { return up_to < rclcpp::Time(e.stamp()); }));

        return erased;
    }

    template<typename... MsgTs>
    template<typename AllocT>
    auto Priority<MsgTs...>::try_sync(storage_type<AllocT> &storage) const -> std::optional<output_type> {
        // Pre-condition(s)
        ros_signal_assert(storage.are_sorted_helper(index_sequence));
        // const auto pre_total_count = storage.count();

        std::optional<output_type> out = std::nullopt;
        try_sync_helper(storage, out);

        // Post-condition(s)
        ros_signal_assert(storage.are_sorted_helper(index_sequence));

        return out;
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void Priority<MsgTs...>::try_sync_helper(storage_type<AllocT> &storage, std::optional<output_type> &out) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            // Pre-condition(s)
            ros_signal_assert(storage.template is_sorted_helper<I>());
            const auto pre_count = storage.template count<I>();
            std::size_t erased = 0;

            auto &eq = std::get<I>(storage.entry_queues_);

            if (!out.has_value() && storage.template enabled<I>() && !storage.template empty<I>()) {
                out.emplace(std::in_place_index<I>, std::move(eq.front()));
                eq.pop_front();

                erased = 1;
            }

            if (out.has_value() || !storage.template enabled<I>()) {
                try_sync_helper<I + 1>(storage, out);
            }

            if (out.has_value() && out->index() != I && !storage.template empty<I>()) {
                const auto stamp = std::visit([](const auto &e) { return rclcpp::Time(e.stamp()); }, *out);
                const auto min_time = stamp + delta_;

                if (rclcpp::Time(eq.front().stamp()) <= min_time) {
                    auto upper_bound = eq.end();
                    if (min_time < rclcpp::Time(eq.back().stamp())) {
                        upper_bound = std::upper_bound(
                                eq.begin(), eq.end(), min_time, LessEntryStamp<element_type<I>>());
                    }

                    erased = std::distance(eq.begin(), upper_bound);
                    eq.erase(eq.begin(), upper_bound);
                }
            }

            ros_signal_assert(storage.template is_sorted_helper<I>());
            const auto post_count = storage.template count<I>();
            ros_signal_assert(pre_count - erased == post_count);
        } else {
            // Done
        }
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    std::size_t Priority<MsgTs...>::clear_helper(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            return clear<I>(storage, up_to) + clear_helper<I + 1>(storage, up_to);
        } else {
            return 0;
        }
    }

    template<typename... MsgTs>
    template<typename AllocT>
    class Priority<MsgTs...>::storage_type {
        friend class Priority;

    public:
        using policy_type = Priority;

        template<typename A>
        using state_message_type = ros_signal_interfaces::msg::PriorityState_<A>;

        using allocator_type = AllocT;

    private:
        template<typename T>
        using allocator_rebind_type = typename std::allocator_traits<allocator_type>::template rebind_alloc<T>;

        template<typename T>
        using entry_queue_type = std::deque<entry_type<T>, allocator_rebind_type<entry_type<T>>>;

        using entry_queue_tuple_type = std::tuple<entry_queue_type<MsgTs>...>;

        using message_enabled_type = std::bitset<message_count>;

    public:
        explicit storage_type(const allocator_type &alloc = allocator_type());

        template<std::size_t I>
        [[nodiscard]] std::size_t count() const {
            static_assert(0 <= I && I < message_count);
            return std::get<I>(entry_queues_).size();
        }

        [[nodiscard]] std::size_t count() const;

        template<typename SequenceContainerT>
        void count(SequenceContainerT &c) const {
            count_helper(c, std::make_index_sequence<message_count>{});
        }

        template<std::size_t I>
        [[nodiscard]] bool empty() const {
            return std::get<I>(entry_queues_).empty();
        }

        [[nodiscard]] bool empty() const;

        template<typename SequenceContainerT>
        void empty(SequenceContainerT &c) const {
            empty_helper(c, index_sequence);
        }

        template<std::size_t I>
        [[nodiscard]] bool enabled() const {
            static_assert(0 <= I && I < message_count);
            return enabled_[I];
        }

        template<std::size_t I>
        void enabled(const bool e) {
            static_assert(0 <= I && I < message_count);
            enabled_[I] = e;
        }

        template<typename SequenceContainerT>
        void enabled(SequenceContainerT &c) const;

        template<typename A>
        void to_state_message(const policy_type &sync_policy, state_message_type<A> &msg) const;

        [[nodiscard]] allocator_type get_allocator() const {
            return alloc_;
        }

    private:
        template<std::size_t... Is>
        std::size_t count_helper(std::index_sequence<Is...>) const {
            return (count<Is>() + ...);
        }

        template<typename SequenceContainerT, std::size_t... Is>
        void count_helper(SequenceContainerT &c, std::index_sequence<Is...>) const {
            c.insert(c.end(), {count<Is>()...});
        }

        template<std::size_t... Is>
        bool empty_helper(std::index_sequence<Is...>) const {
            return ((enabled<Is>() && empty<Is>()) && ...);
        }

        template<typename SequenceContainerT, std::size_t... Is>
        void empty_helper(SequenceContainerT &c, std::index_sequence<Is...>) const {
            c.insert(c.end(), {empty<Is>()...});
        }

        template<std::size_t... Is>
        std::size_t enabled_count_helper(std::index_sequence<Is...>) const {
            return ((enabled<Is>() ? 1 : 0) + ...);
        }

        template<std::size_t I>
        [[nodiscard]] bool is_sorted_helper() const {
            const auto &eq = std::get<I>(entry_queues_);
            return std::is_sorted(eq.begin(), eq.end(), LessEntryStamp<element_type<I>>());
        }

        template<std::size_t... Is>
        [[nodiscard]] bool are_sorted_helper(std::index_sequence<Is...>) const {
            return (is_sorted_helper<Is>() && ...);
        }

        message_enabled_type enabled_;

        entry_queue_tuple_type entry_queues_;

        allocator_type alloc_;
    };

    template<typename... MsgTs>
    template<typename AllocT>
    Priority<MsgTs...>::storage_type<AllocT>::storage_type(const allocator_type &alloc)
        : enabled_(),
          entry_queues_(std::allocator_arg, alloc),
          alloc_(alloc) {
        enabled_.set();
    }

    template<typename... MsgTs>
    template<typename AllocT>
    std::size_t Priority<MsgTs...>::storage_type<AllocT>::count() const {
        return count_helper(index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    bool Priority<MsgTs...>::storage_type<AllocT>::empty() const {
        return empty_helper(index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    template<typename SequenceContainerT>
    void Priority<MsgTs...>::storage_type<AllocT>::enabled(SequenceContainerT &c) const {
        enabled_helper(c, index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    template<typename A>
    void Priority<MsgTs...>::storage_type<AllocT>::to_state_message(
            const policy_type &, state_message_type<A> &msg) const {
        msg.enabled.clear();
        msg.count.clear();

        msg.enabled.reserve(message_count);
        msg.count.reserve(message_count);

        enabled(msg.enabled);
        count(msg.count);
    }
}// namespace ros_signal::sync_policy

#endif// ROS_SIGNAL_SYNC_POLICY_PRIORITY_HPP
