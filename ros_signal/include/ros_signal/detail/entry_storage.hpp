#ifndef ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP
#define ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP

#include <optional>
#include <type_traits>

#include <rclcpp/loaned_message.hpp>
#include "ros_signal/traits.hpp"


namespace ros_signal::detail {
    template<typename MsgT, typename = void>
    struct EntryMessageStorageTraitsHelper {
        using element_type = MsgT;
    };

    template<typename MsgPtrT>
    struct EntryMessageStorageTraitsHelper<MsgPtrT, std::enable_if_t<is_nullable_v<MsgPtrT> > > {
        using element_type = typename std::pointer_traits<MsgPtrT>::element_type;
    };

    template<typename MsgT, typename EntryT, typename = void>
    struct EntryTraits {
        using message_type = std::decay_t<MsgT>;
        using nullable_type = std::optional<EntryT>;

        using element_type = typename EntryMessageStorageTraitsHelper<message_type>::element_type;

        static constexpr nullable_type null() {
            return std::nullopt;
        }
    };

    template<typename MsgPtrT, typename EntryT>
    struct EntryTraits<MsgPtrT, EntryT, std::enable_if_t<is_nullable_v<MsgPtrT> and has_stamp_v<MsgPtrT>> > {
        using message_type = std::decay_t<MsgPtrT>;
        using nullable_type = EntryT;

        using element_type = typename std::pointer_traits<MsgPtrT>::element_type;

        static constexpr nullable_type null() {
            return nullable_type(nullptr);
        }
    };

    template<typename, typename, typename = void>
    class EntryStampStorage {
    public:
        using time_type = rclcpp::Time;

        explicit EntryStampStorage(const time_type &stamp)
            : stamp_(stamp) {
        }

        EntryStampStorage(const EntryStampStorage &) = default;

        EntryStampStorage(EntryStampStorage &&) = default;

        EntryStampStorage &operator=(const EntryStampStorage &) = default;

        EntryStampStorage &operator=(EntryStampStorage &&) = default;

        ~EntryStampStorage() = default;

        [[nodiscard]] const time_type &stamp() const {
            return stamp_;
        }

    private:
        time_type stamp_;
    };

    template<typename MsgT, typename EntryT>
    class EntryStampStorage<MsgT, EntryT, std::enable_if_t<has_stamp_v<MsgT> > > {
    public:
        using time_type = typename stamp<MsgT>::type;

        EntryStampStorage() = default;

        explicit EntryStampStorage(const time_type &) {
        }

        EntryStampStorage(const EntryStampStorage &) = default;

        EntryStampStorage(EntryStampStorage &&) = default;

        EntryStampStorage &operator=(const EntryStampStorage &) = default;

        EntryStampStorage &operator=(EntryStampStorage &&) = default;

        ~EntryStampStorage() = default;

        [[nodiscard]] const time_type &stamp() const {
            /*
             * SAFETY: https://en.cppreference.com/w/cpp/language/static_cast paragraph (7.e):
             *  An rvalue(until C++11)/A prvalue(since C++11) of type “pointer to cv1 Base” can be explicitly converted
             *  to the type “pointer to cv2 Derived” if all following conditions are satisfied:
             *   - Derived is a complete class type.
             *   - Base is a base class of Derived.
             *   - cv1 is not a greater cv-qualification than cv2.
             */
            static_assert(std::is_base_of_v<EntryStampStorage, EntryT>);
            return get_stamp(static_cast<const EntryT *>(this)->message());
        }
    };

    template<typename MsgT, typename EntryT, typename = void>
    class EntryMessageStorage {
    public:
        using message_type = typename EntryTraits<MsgT, EntryT>::message_type;

        explicit EntryMessageStorage(const message_type &message)
            : message_(message) {
        }

        explicit EntryMessageStorage(message_type &&message)
            : message_(std::forward<message_type>(message)) {
        }

        EntryMessageStorage(const EntryMessageStorage &) = default;

        EntryMessageStorage(EntryMessageStorage &&) = default;

        EntryMessageStorage &operator=(const EntryMessageStorage &) = default;

        EntryMessageStorage &operator=(EntryMessageStorage &&) = default;

        ~EntryMessageStorage() = default;

        operator message_type &() {
            return message();
        }

        operator const message_type &() const {
            return message();
        }

        [[nodiscard]] message_type &message() & {
            return message_;
        }

        [[nodiscard]] const message_type &message() const & {
            return message_;
        }

        [[nodiscard]] message_type &&message() && {
            return std::move(message_);
        }

        EntryT &operator=(const message_type &message) {
            message_ = message;
            /*
             * SAFETY: https://en.cppreference.com/w/cpp/language/static_cast paragraph (7.e):
             *  An rvalue(until C++11)/A prvalue(since C++11) of type “pointer to cv1 Base” can be explicitly converted
             *  to the type “pointer to cv2 Derived” if all following conditions are satisfied:
             *   - Derived is a complete class type.
             *   - Base is a base class of Derived.
             *   - cv1 is not a greater cv-qualification than cv2.
             */
            static_assert(std::is_base_of_v<EntryMessageStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        EntryT &operator=(message_type &&message) {
            message_ = std::forward<message_type>(message);
            /*
             * SAFETY: https://en.cppreference.com/w/cpp/language/static_cast paragraph (7.e):
             *  An rvalue(until C++11)/A prvalue(since C++11) of type “pointer to cv1 Base” can be explicitly converted
             *  to the type “pointer to cv2 Derived” if all following conditions are satisfied:
             *   - Derived is a complete class type.
             *   - Base is a base class of Derived.
             *   - cv1 is not a greater cv-qualification than cv2.
             */
            static_assert(std::is_base_of_v<EntryMessageStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        message_type *operator->() {
            return &message_;
        }

        const message_type *operator->() const {
            return &message_;
        }

    private:
        message_type message_;
    };

    template<typename MsgPtrT, typename EntryT>
    class EntryMessageStorage<MsgPtrT, EntryT, std::enable_if_t<is_nullable_v<MsgPtrT> > > {
    public:
        using message_type = typename EntryTraits<MsgPtrT, EntryT>::message_type;
        using element_type = typename EntryTraits<MsgPtrT, EntryT>::element_type;

        explicit EntryMessageStorage(const message_type &message_ptr)
            : message_ptr_(message_ptr) {
        }

        explicit EntryMessageStorage(message_type &&message_ptr)
            : message_ptr_(std::forward<message_type>(message_ptr)) {
        }

        EntryMessageStorage(const EntryMessageStorage &) = default;

        EntryMessageStorage(EntryMessageStorage &&) = default;

        EntryMessageStorage &operator=(const EntryMessageStorage &) = default;

        EntryMessageStorage &operator=(EntryMessageStorage &&) = default;

        ~EntryMessageStorage() = default;

        operator element_type &() {
            return message();
        }

        operator const element_type &() const {
            return message();
        }

        [[nodiscard]] element_type &message() & {
            return *message_ptr_;
        }

        [[nodiscard]] const element_type &message() const & {
            return *message_ptr_;
        }

        EntryT &operator=(const message_type &message_ptr) {
            message_ptr_ = message_ptr;
            static_assert(std::is_base_of_v<EntryMessageStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        EntryT &operator=(message_type &&message_ptr) {
            message_ptr_ = std::forward<message_type>(message_ptr);
            static_assert(std::is_base_of_v<EntryMessageStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        element_type *operator->() {
            return &*message_ptr_;
        }

        const element_type *operator->() const {
            return &*message_ptr_;
        }

        [[nodiscard]] bool has_value() const {
            return message_ptr_ != nullptr;
        }

        template<typename... ArgTs>
        element_type &emplace(ArgTs &&... args) {
            message_ptr_ = message_type(std::forward<ArgTs>(args)...);
            return message();
        }

    private:
        message_type message_ptr_;
    };

    template<typename MsgT, typename EntryT, typename AllocT>
    class EntryMessageStorage<rclcpp::LoanedMessage<MsgT, AllocT>, EntryT> {
    public:
        using message_type = typename EntryTraits<rclcpp::LoanedMessage<MsgT, AllocT>, EntryT>::message_type;
        using element_type = typename EntryTraits<rclcpp::LoanedMessage<MsgT, AllocT>, EntryT>::element_type;

        explicit EntryMessageStorage(message_type &&loaned_message)
            : loaned_message_(std::forward<message_type>(loaned_message)) {
        }

        EntryMessageStorage(const EntryMessageStorage &) = default;

        EntryMessageStorage(EntryMessageStorage &&) = default;

        EntryMessageStorage &operator=(const EntryMessageStorage &) = default;

        EntryMessageStorage &operator=(EntryMessageStorage &&) = default;

        ~EntryMessageStorage() = default;

        [[nodiscard]] element_type &message() & {
            return loaned_message_.get();
        }

        [[nodiscard]] const element_type &message() const & {
            return loaned_message_.get();
        }

        message_type loaned_message() && {
            return std::move(loaned_message_);
        }

        element_type *operator->() {
            return &loaned_message_.get();
        }

        const element_type *operator->() const {
            return &loaned_message_.get();
        }

    private:
        message_type loaned_message_;
    };
} // namespace ros_signal::detail

#endif // ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP
