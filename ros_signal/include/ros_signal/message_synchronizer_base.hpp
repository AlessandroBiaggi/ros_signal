#ifndef ROS_SIGNAL_MESSAGE_SYNCHRONIZER_BASE_HPP
#define ROS_SIGNAL_MESSAGE_SYNCHRONIZER_BASE_HPP

#include <cstddef>

#include <memory>
#include <type_traits>

#include <rclcpp/macros.hpp>


namespace ros_signal {
    template<typename SyncPolicyT, typename AllocT = std::allocator<void>>
    class MessageSynchronizerBase {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(MessageSynchronizerBase)

        using sync_policy_type = std::decay_t<SyncPolicyT>;
        using allocator_type = std::decay_t<AllocT>;

        template<typename MsgT>
        using entry_type = typename sync_policy_type::template entry_type<MsgT>;

        template<std::size_t I>
        using element_type = typename sync_policy_type::template element_type<I>;

        using output_type = typename sync_policy_type::output_type;

    private:
        using storage_type = typename sync_policy_type::template storage_type<allocator_type>;

    public:
        template<typename A = std::allocator<void>>
        using state_message_type = typename storage_type::template state_message_type<A>;

        explicit MessageSynchronizerBase(
            const sync_policy_type &sync_policy,
            const allocator_type &alloc = allocator_type()
        );

        virtual ~MessageSynchronizerBase() = default;

        template<std::size_t I>
        void push(const entry_type<element_type<I>> &entry) {
            sync_policy_.template push<I>(storage_, entry);
            try_sync_impl();
        }

        template<std::size_t I>
        void push(entry_type<element_type<I>> &&entry) {
            sync_policy_.template push<I>(storage_, std::forward<entry_type<element_type<I>>>(entry));
            try_sync_impl();
        }

        template<std::size_t I, typename... ArgTs>
        void emplace(ArgTs &&... args) {
            sync_policy_.template emplace<I>(storage_, std::forward<ArgTs>(args)...);
            try_sync_impl();
        }

        void clear(const rclcpp::Time &up_to) {
            storage_.clear(up_to);
        }
        template<std::size_t I>
        [[nodiscard]] bool enabled() const {
            return storage_.template enabled<I>();
        }

        template<std::size_t I>
        void enabled(const bool e) {
            storage_.template enabled<I>(e);
        }

        template<typename SequenceContainerT>
        void enabled(SequenceContainerT &c) const {
            storage_.template enabled<SequenceContainerT>(c);
        }

        template<typename A = std::allocator<void>>
        void to_state_message(state_message_type<A> &msg) const {
            storage_.to_state_message(sync_policy_, msg);
        }

        allocator_type get_allocator() {
            return storage_.get_allocator();
        }

    private:
        void try_sync_impl();

        virtual void sync_handler(output_type &) = 0;

        sync_policy_type sync_policy_;
        storage_type storage_;
    };

    template<typename SyncPolicyT, typename AllocT>
    MessageSynchronizerBase<SyncPolicyT, AllocT>::MessageSynchronizerBase(
        const sync_policy_type &sync_policy,
        const allocator_type &alloc
    ) : sync_policy_(sync_policy)
      , storage_(alloc) {
    }

    template<typename SyncPolicyT, typename AllocT>
    void MessageSynchronizerBase<SyncPolicyT, AllocT>::try_sync_impl() {
        auto out = sync_policy_.try_sync(storage_);
        if (out.has_value()) {
            sync_handler(out.value());
            out.reset();
        }
    }
} // namespace ros_signal

#endif // ROS_SIGNAL_MESSAGE_SYNCHRONIZER_BASE_HPP
