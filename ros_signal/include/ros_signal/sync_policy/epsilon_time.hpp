#ifndef ROS_SIGNAL_SYNC_POLICY_EPSILON_TIME_HPP
#define ROS_SIGNAL_SYNC_POLICY_EPSILON_TIME_HPP

#include <cstddef>

#include <algorithm>
#include <array>
#include <bitset>
#include <queue>
#include <tuple>
#include <type_traits>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <ros_signal_interfaces/msg/epsilon_time_state.hpp>

#include "ros_signal/entry.hpp"
#include "ros_signal/macros.hpp"


namespace ros_signal::sync_policy {
    /**
     * @brief Message synchronization policy with epsilon time
     * @tparam MsgTs message types
     */
    template<typename... MsgTs>
    class EpsilonTime {
    public:
        /**
         * @brief Message type count
         */
        static constexpr std::size_t message_count = sizeof...(MsgTs);

        /**
         * @brief Entry type for message type `T`
         * @tparam T message type
         */
        template<typename T>
        using entry_type = Entry<T>;

        /**
         * @brief Message type at index `I`
         * @tparam I index of the message type
         */
        template<std::size_t I>
        using element_type = std::tuple_element_t<I, std::tuple<typename entry_type<MsgTs>::element_type...>>;

        /**
         * @brief Nullable entry type for message type `T`
         * @tparam T message type
         */
        template<typename T>
        using nullable_entry_type = typename entry_type<T>::nullable_type;

        /**
         * @brief Synchronized output type
         */
        using output_type = std::tuple<nullable_entry_type<MsgTs>...>;

        /**
         * @brief Message storage type
         * @tparam AllocT allocator type
         */
        template<typename AllocT>
        class storage_type;

        /**
         * @param epsilon Maximum time difference between messages in a synchronized output
         * @pre `epsilon >= rclcpp::Duration::from_seconds(0.0)`
         */
        explicit EpsilonTime(const rclcpp::Duration &epsilon = rclcpp::Duration::from_seconds(0));

        /**
         * @param epsilon_s maximum time difference between messages in a synchronized output in seconds
         * @pre `epsilon >= 0.0`
         */
        explicit EpsilonTime(const double epsilon_s) : EpsilonTime(rclcpp::Duration::from_seconds(epsilon_s)) {
        }

        /**
         * TODO
         */
        EpsilonTime(const EpsilonTime &) = default;

        /**
         * TODO
         */
        EpsilonTime(EpsilonTime &&) = default;

        /**
         * TODO
         */
        ~EpsilonTime() = default;

        /**
         * TODO
         */
        EpsilonTime &operator=(const EpsilonTime &) = default;

        /**
         * TODO
         */
        EpsilonTime &operator=(EpsilonTime &&) = default;

        /**
         * @brief Set epsilon
         * @param epsilon maximum time difference between messages in a synchronized output
         * @pre `epsilon >= rclcpp::Duration::from_seconds(0.0)`
         * @post `epsilon() == epsilon`
         * @post Given `out = sync(...)`, either `out == std::nullopt` or `a.stamp() - b.stamp() <= epsilon()` for each `a`, `b` in `out` such that `!entry_type<element_type<I>>::is_null(a) && !entry_type<element_type<I>>::is_null(b)`
         */
        void epsilon(const rclcpp::Duration &epsilon);

        /**
         * @brief Set epsilon
         * @param epsilon_s maximum time difference between messages in a synchronized output
         * @pre `epsilon_s >= 0`
         * @post `epsilon() == rclcpp::Duration::from_seconds(epsilon_s)`
         * @post Given `out = sync(...)`, either `out == std::nullopt` or `a.stamp() - b.stamp() <= epsilon()` for each `a`, `b` in `out` such that `!entry_type<element_type<I>>::is_null(a) && !entry_type<element_type<I>>::is_null(b)`
         */
        void epsilon(const double epsilon_s) {
            epsilon(rclcpp::Duration::from_seconds(epsilon_s));
        }

        /**
         * @return The maximum time difference between messages in a synchronized output
         */
        [[nodiscard]] rclcpp::Duration epsilon() const {
            return epsilon_;
        }

        /**
         * @brief Push an entry into the message storage
         * @tparam I index of the message type
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param entry entry to push
         * @pre Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `less_entry_time<element_type<I>>`
         * @post Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `less_entry_time<element_type<I>>`
         * @post if `storage.enabled<I>()` then `storage.count()` is incremented by one
         * @post if `storage.enabled<I>()` then `storage.count<I>()` is incremented by one
         * @post if `storage.enabled<I>()` then `!storage.empty<I>()`
         * @note If exceptions are thrown, this function has no effect on the storage
         */
        template<std::size_t I, typename AllocT>
        void push(storage_type<AllocT> &storage, const entry_type<element_type<I>> &entry) const;

        /**
         * @brief Push an entry into the message storage
         * @tparam I index of the message type
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param entry entry to push
         * @pre Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>`
         * @post Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>`
         * @post if `storage.enabled<I>()` then `storage.count()` is incremented by one
         * @post if `storage.enabled<I>()` then `storage.count<I>()` is incremented by one
         * @post if `storage.enabled<I>()` then `!storage.empty<I>()`
         * @note if exceptions are thrown, this function has no effect on the storage
         * @note if exceptions are thrown, all post conditions are not satisfied
         */
        template<std::size_t I, typename AllocT>
        void push(storage_type<AllocT> &storage, entry_type<element_type<I>> &&entry) const;

        /**
         * @brief Construct an entry in-place in the message storage
         * @tparam I index of the message type
         * @tparam AllocT message storage allocator type
         * @tparam ArgTs argument types
         * @param storage message storage
         * @param stamp time of the entry
         * @param args arguments to construct the entry
         * @pre Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>`
         * @pre The time of the constructed entry is equal to `time`
         * @post Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>`
         * @post if `storage.enabled<I>()` then `storage.count()` is incremented by one
         * @post if `storage.enabled<I>()` then `storage.count<I>()` is incremented by one
         * @post if `storage.enabled<I>()` then `!storage.empty<I>()`
         * @note If exceptions are thrown, this function has no effect on `storage`
         */
        template<std::size_t I, typename AllocT, typename... ArgTs>
        void emplace(storage_type<AllocT> &storage, const rclcpp::Time &stamp, ArgTs &&...args) const;

        /**
         * @brief Clear messages in the message storage up to a certain time
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param up_to maximum time for an entry to be cleared
         * @pre Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>` for each `I` in `[0, message_count)`
         * @post Entries of type `entry_type<element_type<I>>` in `storage` are sorted by `entry_compare_type<element_type<I>>` for each `I` in `[0, message_count)`
         * @post `up_to < e.stamp()`, for each entry `e` of type `entry_type<element_type<I>>` in `storage`, for each index `I` in `[0, message_count)`
         * @post The previous value of `storage.count()` minus the return value is equal to `storage.count()`
         * @post The return value is greater than or equal to zero and less than or equal to the previous value of `count()`
         * @note If exceptions are thrown, post condition (1) is still guaranteed
         * @note If exceptions are thrown, post condition (2, 3, 4) are not guaranteed
         * @return Number of entries cleared
         */
        template<typename AllocT>
        std::size_t clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
            return clear_helper(storage, up_to);
        }

        template<std::size_t I, typename AllocT>
        std::size_t clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const;

        /**
         * @brief Synchronize messages in the message storage
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @pre Entries of type `entry_type<J>` in `storage` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)`
         * @post Entries of type `entry_type<J>` in `storage` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)`
         * @note If exceptions are thrown, the post conditions are still guaranteed
         * @note If exceptions are thrown, it is not guaranteed that `storage.count<I>()` is the same as before for all `I` in `[0, message_count)`
         * @return Synchronized output if found, `std::nullopt` otherwise
         */
        template<typename AllocT>
        std::optional<output_type> try_sync(storage_type<AllocT> &storage) const;

    private:
        /**
         * @brief TODO
         */
        static constexpr auto index_sequence = std::make_index_sequence<message_count>();

        /**
         * @brief Helper function for `sync`
         * @tparam I index of the current message type
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param min minimum time for an entry to be included in the synchronized output
         * @param max maximum time for an entry to be included in the synchronized output
         * @pre messages in `std::get<J>(storage)` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)`
         * @pre `!empty<J>()` for each `J` in `[I, message_count)`
         * @pre `min <= max`
         * @post Entries of type `entry_type<J>` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)`
         * @post For each `J` in `[J, message_count)`, for each entry `e` of type `entry_type<J>` in `storage`, `min <= e.stamp()`
         * @return `true` if a synchronized output is found, `false` otherwise
         */
        template<std::size_t I = 0, typename AllocT>
        bool try_sync_helper(storage_type<AllocT> &storage, const rclcpp::Time &min, const rclcpp::Time &max) const;

        /**
         * @brief TODO
         * @tparam I TODO
         * @tparam AllocT TODO
         * @param storage TODO
         * @param max TODO
         * @return TODO
         */
        template<std::size_t I = 0, typename AllocT>
        std::optional<rclcpp::Time> get_max_stamp_helper(storage_type<AllocT> &storage) const;

        /**
         * @brief Helper function for `clear`
         * @tparam I index of the current message type
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param up_to maximum time for an entry to be cleared
         * @pre Entries of type `entry_type<J>` in `storage` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)`
         * @post Entries of type `entry_type<J>` in `storage` are sorted by `entry_compare_type<element_type<J>>` for each `J` in `[I, message_count)` (1)
         * @post For each index `J` in `[I, message_count)`, for each entry of type `entry_type<J>` in `storage`, `up_to < e.stamp()` (2)
         * @note if exceptions are thrown, post condition (1) is still guaranteed
         * @note if exceptions are thrown, post condition (2) is not guaranteed for all `J` in `[I, message_count)`
         * @return Number of entries cleared
         */
        template<std::size_t I = 0, typename AllocT>
        std::size_t clear_helper(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const;

        /**
         * @brief Helper function that constructs a synchronized output
         * @tparam I message type index
         * @tparam AllocT message storage allocator type
         * @param storage message storage
         * @param out synchronized output
         * @pre `!empty<J>()` for each `J` in `[I, message_count)`
         * @post `count<J>()` is decremented by one for each `J` in `[I, message_count) such that `enabled<J>()`
         */
        template<std::size_t I = 0, typename AllocT>
        void make_output_helper(storage_type<AllocT> &storage, output_type &out) const;

        /**
         * @brief TODO
         */
        rclcpp::Duration epsilon_;
    };

    template<typename... MsgTs>
    EpsilonTime<MsgTs...>::EpsilonTime(const rclcpp::Duration &epsilon) : epsilon_(epsilon) {
        // Pre-condition(s)
        ros_signal_assert(epsilon >= rclcpp::Duration::from_seconds(0));
    }

    template<typename... MsgTs>
    void EpsilonTime<MsgTs...>::epsilon(const rclcpp::Duration &epsilon) {
        // Pre-condition(s)
        ros_signal_assert(epsilon >= rclcpp::Duration::from_seconds(0));

        epsilon_ = epsilon;

        // Post-condition(s)
        ros_signal_assert(EpsilonTime::epsilon() == epsilon);
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void EpsilonTime<MsgTs...>::push(storage_type<AllocT> &storage, const entry_type<element_type<I>> &entry) const {
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
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || (!storage.template enabled<I>() && pre_count == post_count));
        ros_signal_assert((storage.template enabled<I>() && pre_count_I + 1 == post_count_I && !storage.template empty<I>()) || pre_count_I == post_count_I);
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void EpsilonTime<MsgTs...>::push(storage_type<AllocT> &storage, entry_type<element_type<I>> &&entry) const {
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
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || (!storage.template enabled<I>() && pre_count == post_count));
        ros_signal_assert((storage.template enabled<I>() && pre_count_I + 1 == post_count_I && !storage.template empty<I>()) || (!storage.template enabled<I>() && pre_count_I == post_count_I));
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT, typename... ArgTs>
    void EpsilonTime<MsgTs...>::emplace(
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
        ros_signal_assert((storage.template enabled<I>() && pre_count + 1 == post_count) || (!storage.template enabled<I>() && pre_count == post_count));
        ros_signal_assert((storage.template enabled<I>() && pre_count_I + 1 == post_count_I) || (!storage.template enabled<I>() && pre_count_I == post_count_I));
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    std::size_t EpsilonTime<MsgTs...>::clear(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
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
    auto EpsilonTime<MsgTs...>::try_sync(storage_type<AllocT> &storage) const -> std::optional<output_type> {
        // Pre-condition(s)
        ros_signal_assert(storage.are_sorted_helper(index_sequence));
        // const auto pre_total_count = storage.count();

        std::optional<output_type> out = std::nullopt;

        if (storage.any_enabled_helper(index_sequence)) {
            while (!(storage.empty() || out.has_value())) {
                const auto max_time = get_max_stamp_helper(storage);
                ros_signal_assert(max_time.has_value());

                const auto min_time = max_time.value() - epsilon_;

                /*
                 * SAFETY:
                 * - entry queues are sorted by contract
                 * - entry queues are non-empty by the while loop condition
                 * - `max_time` is the maximum time of all front entries in the queues
                 * - `epsilon_ >= 0` by contract, so `min_time := max_time - epsilon_ <= max_time`
                 */
                if (try_sync_helper(storage, min_time, max_time.value())) {
                    make_output_helper(storage, out.emplace(entry_type<MsgTs>::null()...));
                }
            }
        }

        // Post-condition(s)
        ros_signal_assert(storage.are_sorted_helper(index_sequence));

        return out;
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    bool EpsilonTime<MsgTs...>::try_sync_helper(
            storage_type<AllocT> &storage, const rclcpp::Time &min, const rclcpp::Time &max) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            // Pre-condition(s)
            ros_signal_assert(storage.template is_sorted_helper<I>());
            ros_signal_assert(!(storage.template enabled<I>() && storage.template empty<I>()));
            ros_signal_assert(min <= max);
            const auto pre_count = storage.template count<I>();
            std::size_t erased = 0;

            bool sync = true;
            if (storage.template enabled<I>()) {
                auto &eq = std::get<I>(storage.entry_queues_);

                // SAFETY: `eq` is sorted by contract
                if (const auto &b = eq.back(); rclcpp::Time(b.stamp()) <= max) {
                    auto upper_bound = eq.end();
                    if (min <= rclcpp::Time(b.stamp())) {
                        upper_bound = std::prev(upper_bound);
                    } else {
                        sync = false;
                    }

                    erased = std::distance(eq.begin(), upper_bound);
                    eq.erase(eq.begin(), upper_bound);
                } else {
                    auto upper_bound = std::upper_bound(eq.begin(), eq.end(), max, LessEntryStamp<element_type<I>>());
                    if (upper_bound != eq.begin()) {
                        if (const auto best = std::prev(upper_bound);
                            min <= rclcpp::Time(best->stamp()) && rclcpp::Time(best->stamp()) <= max) {
                            upper_bound = best;
                        } else {
                            sync = false;
                        }

                        erased = std::distance(eq.begin(), upper_bound);
                        eq.erase(eq.begin(), upper_bound);
                    }
                }
            }

            ros_signal_assert(storage.template is_sorted_helper<I>());
            ros_signal_assert(!(sync && storage.template enabled<I>() && storage.template empty<I>()));
            const auto post_count = storage.template count<I>();
            ros_signal_assert(pre_count - erased == post_count);

            const bool next_sync = try_sync_helper<I + 1>(storage, min, max);
            return sync && next_sync;
        } else {
            return true;
        }
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    std::optional<rclcpp::Time> EpsilonTime<MsgTs...>::get_max_stamp_helper(storage_type<AllocT> &storage) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            // Pre-condition (s)
            ros_signal_assert(!(storage.template enabled<I>() && storage.template empty<I>()));

            auto max_stamp = get_max_stamp_helper<I + 1>(storage);
            if (storage.template enabled<I>()) {
                const auto &eq = std::get<I>(storage.entry_queues_);
                const auto stamp = rclcpp::Time(eq.front().stamp());
                if (!max_stamp.has_value() || max_stamp.value() < stamp) {
                    max_stamp = stamp;
                }
            }

            return max_stamp;
        } else {
            return std::nullopt;
        }
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    std::size_t EpsilonTime<MsgTs...>::clear_helper(storage_type<AllocT> &storage, const rclcpp::Time &up_to) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            return clear<I>(storage, up_to) + clear_helper<I + 1>(storage, up_to);
        } else {
            return 0;
        }
    }

    template<typename... MsgTs>
    template<std::size_t I, typename AllocT>
    void EpsilonTime<MsgTs...>::make_output_helper(storage_type<AllocT> &storage, output_type &out) const {
        static_assert(0 <= I && I <= message_count);

        if constexpr (I < message_count) {
            // Pre-condition(s)
            ros_signal_assert(!(storage.template enabled<I>() && storage.template empty<I>()));
            const auto pre_count = storage.template count<I>();

            if (storage.template enabled<I>()) {
                auto &o = std::get<I>(out);
                ros_signal_assert(!o.has_value());

                auto &eq = std::get<I>(storage.entry_queues_);
                o = std::move(eq.front());
                eq.pop_front();
            }

            // Post-condition(s)
            const auto post_count = storage.template count<I>();
            ros_signal_assert((storage.template enabled<I>() && pre_count - 1 == post_count) || (!storage.template enabled<I>() && pre_count == post_count));

            make_output_helper<I + 1>(storage, out);
        } else {
            // Done
        }
    }

    template<typename... MsgTs>
    template<typename AllocT>
    class EpsilonTime<MsgTs...>::storage_type {
        friend class EpsilonTime;

    public:
        /**
         * @brief Synchronization policy for this storage type
         */
        using policy_type = EpsilonTime;

        /**
         * @brief Message storage state message type
         */
        template<typename A = std::allocator<void>>
        using state_message_type = ros_signal_interfaces::msg::EpsilonTimeState_<A>;

        /**
         * @brief Allocator type
         */
        using allocator_type = AllocT;

    private:
        /**
         * @brief TODO
         * @tparam T TODO
         */
        template<typename T>
        using allocator_rebind_type = typename std::allocator_traits<allocator_type>::template rebind_alloc<T>;

        /**
         * @brief Entry queue type for message type `T`
         * @tparam T message type
         */
        template<typename T>
        using entry_queue_type = std::deque<entry_type<T>, allocator_rebind_type<entry_type<T>>>;

        /**
         * @brief Tuple of entry queues for each message type
         */
        using entry_queue_tuple_type = std::tuple<entry_queue_type<MsgTs>...>;

        /**
         * @brief Container for flags indicating if a message type is enabled
         */
        using message_enabled_type = std::bitset<message_count>;

    public:
        /**
         * @brief Construct a message storage with the given allocator
         * @param alloc allocator
         */
        explicit storage_type(const allocator_type &alloc = allocator_type());

        /**
         * @brief Get the number of entries in the message storage by message type
         * @tparam I index of the message type
         * @post The returned value is greater than or equal to zero
         * @note `count<I>() == 0` iff `empty<I>()`
         * @return Number of entries in the message storage by message type
         */
        template<std::size_t I>
        [[nodiscard]] std::size_t count() const {
            return std::get<I>(entry_queues_).size();
        }

        /**
         * @brief Get the total number of entries in the message storage
         * @post The returned value is equal to the sum of `count<I>()` for each `I` in `[0, message_count)`
         * @note `count() == 0` iff `empty()`
         * @return The total number of entries in the message storage
         */
        [[nodiscard]] std::size_t count() const;

        /**
         * @brief Get the number of entries in the message storage by message type
         * @tparam SequenceContainerT Sequence container type
         * @param c sequence container to store the counts
         * @note `c[I] == count<I>()` for each `I` in `[0, message_count)`
         */
        template<typename SequenceContainerT>
        void count(SequenceContainerT &c) const {
            count_helper(c, std::make_index_sequence<message_count>{});
        }

        /**
         * @brief Check if the message storage is empty by message type
         * @tparam I index of the message type
         * @note `empty<I>()` iff `count<I>() == 0`
         * @return `true` if the message storage has no entries of type `entry_type<element_type<I>>`, `false` otherwise
         */
        template<std::size_t I>
        [[nodiscard]] bool empty() const {
            return std::get<I>(entry_queues_).empty();
        }

        /**
         * @brief Check if the message storage is empty
         * @note `empty()` iff `count() == 0`
         * @return `true` if the message storage is empty, `false` otherwise
         */
        [[nodiscard]] bool empty() const;

        /**
         * @brief Get the empty flags for each message type
         * @tparam SequenceContainerT sequence container type
         * @param c sequence container to store the empty flags
         */
        template<typename SequenceContainerT>
        void empty(SequenceContainerT &c) const {
            empty_helper(c, index_sequence);
        }

        /**
         * @brief Check if a message type is enabled
         * @tparam I index of the message type
         * @return `true` if messages of type `element_type<I>` are enabled, `false` otherwise
         */
        template<std::size_t I>
        [[nodiscard]] bool enabled() const {
            static_assert(0 <= I && I < message_count);
            return enabled_[I];
        }

        /**
         * @brief Set the flag indicating if a message type is enabled
         * @tparam I index of the message type
         * @param e new value for the flag
         * @post `enabled<I>() == e`
         */
        template<std::size_t I>
        void enabled(const bool e) {
            static_assert(0 <= I && I < message_count);
            enabled_[I] = e;
        }

        /**
         * @brief Get the enabled flags for each message type
         * @tparam SequenceContainerT sequence container type
         * @param c sequence container to store the enabled flags
         */
        template<typename SequenceContainerT>
        void enabled(SequenceContainerT &c) const;

        /**
         * @brief Write the state of the message storage to a message
         * @tparam A allocator type
         * @param sync_policy synchronization policy
         * @param msg message to store the state
         */
        template<typename A>
        void to_state_message(const policy_type &sync_policy, state_message_type<A> &msg) const;

        /**
         * @brief Get the allocator used by this message storage
         * @return The allocator used by this message storage
         */
        [[nodiscard]] allocator_type get_allocator() const {
            return alloc_;
        }

    private:
        /**
         * @brief Helper function to get the counts of entries in the message storage
         * @tparam Is index sequence
         * @return The counts of entries in the message storage
         */
        template<std::size_t... Is>
        std::size_t count_helper(std::index_sequence<Is...>) const {
            return (count<Is>() + ...);
        }

        /**
         * @brief Helper function to get the counts of entries in the message storage
         * @tparam SequenceContainerT sequence container type
         * @tparam Is index sequence
         * @param c sequence container to store the counts
         */
        template<typename SequenceContainerT, std::size_t... Is>
        void count_helper(SequenceContainerT &c, std::index_sequence<Is...>) const {
            c.insert(c.end(), {count<Is>()...});
        }

        /**
         * @brief Helper function to check if the message storage is empty
         * @tparam Is index sequence
         * @return `true` if the message storage is empty, `false` otherwise
         */
        template<std::size_t... Is>
        bool empty_helper(std::index_sequence<Is...>) const {
            return ((enabled<Is>() && empty<Is>()) || ...);
        }

        /**
         * @brief Helper function to get the empty flags for each message type
         * @tparam SequenceContainerT sequence container type
         * @tparam Is index sequence
         * @param c sequence container to store the empty flags
         */
        template<typename SequenceContainerT, std::size_t... Is>
        void empty_helper(SequenceContainerT &c, std::index_sequence<Is...>) const {
            c.insert(c.end(), {empty<Is>()...});
        }

        /**
         * @brief Helper function to get the enabled flags for each message type
         * @tparam SequenceContainerT sequence container type
         * @tparam Is index sequence
         * @param c sequence container to store the enabled flags
         */
        template<typename SequenceContainerT, std::size_t... Is>
        void enabled_helper(SequenceContainerT &c, std::index_sequence<Is...>) const {
            c.insert(c.end(), {enabled<Is>()...});
        }

        /**
         * @brief TODO
         * @tparam Is TODO
         * @return TODO
         */
        template<std::size_t... Is>
        std::size_t enabled_count_helper(std::index_sequence<Is...>) const {
            return ((enabled<Is>() ? 1 : 0) + ...);
        }

        /**
         * @brief TODO
         * @tparam Is TODO
         * @return TODO
         */
        template<std::size_t... Is>
        bool any_enabled_helper(std::index_sequence<Is...>) const {
            return (enabled<Is>() || ...);
        }

        /**
         * @brief TODO
         * @tparam I TODO
         * @return TODO
         */
        template<std::size_t I>
        [[nodiscard]] bool is_sorted_helper() const {
            const auto &eq = std::get<I>(entry_queues_);
            return std::is_sorted(eq.begin(), eq.end(), LessEntryStamp<element_type<I>>());
        }

        /**
         * @brief TODO
         * @tparam Is message type index sequence
         * @return TODO
         */
        template<std::size_t... Is>
        [[nodiscard]] bool are_sorted_helper(std::index_sequence<Is...>) const {
            return (is_sorted_helper<Is>() && ...);
        }

        /**
         * @brief Enabled flags for messages
         */
        message_enabled_type enabled_;

        /**
         * @brief Entry queues for messages
         */
        entry_queue_tuple_type entry_queues_;

        /**
         * @brief Allocator
         */
        allocator_type alloc_;
    };

    template<typename... MsgTs>
    template<typename AllocT>
    EpsilonTime<MsgTs...>::storage_type<AllocT>::storage_type(const allocator_type &alloc)
        : enabled_(),
          entry_queues_(std::allocator_arg, alloc),
          alloc_(alloc) {
        enabled_.set();
    }

    template<typename... MsgTs>
    template<typename AllocT>
    std::size_t EpsilonTime<MsgTs...>::storage_type<AllocT>::count() const {
        return count_helper(index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    bool EpsilonTime<MsgTs...>::storage_type<AllocT>::empty() const {
        return empty_helper(index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    template<typename SequenceContainerT>
    void EpsilonTime<MsgTs...>::storage_type<AllocT>::enabled(SequenceContainerT &c) const {
        enabled_helper(c, index_sequence);
    }

    template<typename... MsgTs>
    template<typename AllocT>
    template<typename A>
    void EpsilonTime<MsgTs...>::storage_type<AllocT>::to_state_message(
            const policy_type &sync_policy, state_message_type<A> &msg) const {
        msg.enabled.clear();
        msg.count.clear();

        msg.enabled.reserve(message_count);
        msg.count.reserve(message_count);

        msg.epsilon = sync_policy.epsilon();
        enabled(msg.enabled);
        count(msg.count);
    }
}// namespace ros_signal::sync_policy

#endif// ROS_SIGNAL_SYNC_POLICY_EPSILON_TIME_HPP
