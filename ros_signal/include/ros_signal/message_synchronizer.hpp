#ifndef ROS_SIGNAL_MESSAGE_SYNCHRONIZER_HPP
#define ROS_SIGNAL_MESSAGE_SYNCHRONIZER_HPP

#include <cstddef>

#include <functional>
#include <memory>
#include <type_traits>

#include <rclcpp/macros.hpp>
#include <rclcpp/message_info.hpp>

#include "ros_signal/message_synchronizer_base.hpp"
#include "ros_signal/traits.hpp"


namespace ros_signal {
    template<typename SyncPolicyT, typename AllocT = std::allocator<void>>
    class MessageSynchronizer : public MessageSynchronizerBase<SyncPolicyT, AllocT> {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(MessageSynchronizer)

        using typename MessageSynchronizerBase<SyncPolicyT, AllocT>::sync_policy_type;

        template<typename MsgT>
        using entry_type = typename sync_policy_type::template entry_type<MsgT>;

        template<std::size_t I>
        using element_type = typename sync_policy_type::template element_type<I>;

        using typename MessageSynchronizerBase<SyncPolicyT, AllocT>::output_type;
        using typename MessageSynchronizerBase<SyncPolicyT, AllocT>::allocator_type;

        MessageSynchronizer(
            const sync_policy_type &sync_policy,
            std::function<void(output_type &)> callback,
            const allocator_type &alloc = allocator_type()
        );

    private:
        std::function<void(output_type &)> callback_;

        void sync_handler(output_type &out) override {
            callback_(out);
        }
    };

    template<typename SyncPolicyT, typename AllocT>
    MessageSynchronizer<SyncPolicyT, AllocT>::MessageSynchronizer(
            const sync_policy_type &sync_policy,
            std::function<void(output_type &)> callback,
            const allocator_type &alloc
    ) : MessageSynchronizerBase<SyncPolicyT, AllocT>(sync_policy, alloc)
      , callback_(std::move(callback)) {
    }
}

#endif // ROS_SIGNAL_MESSAGE_SYNCHRONIZER_HPP
