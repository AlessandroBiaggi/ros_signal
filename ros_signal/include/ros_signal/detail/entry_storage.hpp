#ifndef ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP
#define ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP

#include <optional>
#include <type_traits>

#include "ros_signal/traits.hpp"
#include <rclcpp/loaned_message.hpp>


namespace ros_signal::detail {
    template<typename ValueT, typename = void>
    struct EntryValueStorageTraitsHelper {
        using value_type = std::decay_t<ValueT>;
        using element_type = value_type;
    };

    template<typename ValuePtrT>
    struct EntryValueStorageTraitsHelper<ValuePtrT, std::enable_if_t<is_nullable_v<ValuePtrT>>> {
        using value_ptr_type = std::decay_t<ValuePtrT>;
        using value_type = typename std::pointer_traits<value_ptr_type>::element_type;
        using element_type = value_ptr_type;
    };

    template<typename ValueT, typename EntryT, typename = void>
    struct EntryTraits {
        using value_type = typename EntryValueStorageTraitsHelper<ValueT>::value_type;
        using element_type = typename EntryValueStorageTraitsHelper<ValueT>::element_type;
        using nullable_type = std::optional<EntryT>;

        static constexpr nullable_type null() {
            return std::nullopt;
        }
    };

    template<typename ValuePtrT, typename EntryT>
    struct EntryTraits<ValuePtrT, EntryT, std::enable_if_t<is_nullable_v<ValuePtrT> and not has_stamp_v<ValuePtrT>>> {
        using value_ptr_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::value_ptr_type;
        using value_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::value_type;
        using element_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::element_type;
        using nullable_type = std::optional<EntryT>;

        static constexpr nullable_type null() {
            return std::nullopt;
        }
    };

    template<typename ValuePtrT, typename EntryT>
    struct EntryTraits<ValuePtrT, EntryT, std::enable_if_t<is_nullable_v<ValuePtrT> and has_stamp_v<ValuePtrT>>> {
        using value_ptr_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::value_ptr_type;
        using value_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::value_type;
        using element_type = typename EntryValueStorageTraitsHelper<ValuePtrT>::element_type;
        using nullable_type = EntryT;

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

    template<typename ValueT, typename EntryT>
    class EntryStampStorage<ValueT, EntryT, std::enable_if_t<has_stamp_v<ValueT>>> {
    public:
        using time_type = typename stamp<ValueT>::type;

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
            return ros_signal::get_stamp(static_cast<const EntryT *>(this)->value());
        }
    };

    template<typename ValueT, typename EntryT, typename = void>
    class EntryValueStorage {
    public:
        using value_type = typename EntryTraits<ValueT, EntryT>::value_type;

        explicit EntryValueStorage(const value_type &message)
            : value_(message) {
        }

        explicit EntryValueStorage(value_type &&message)
            : value_(std::forward<value_type>(message)) {
        }

        EntryValueStorage(const EntryValueStorage &) = default;

        EntryValueStorage(EntryValueStorage &&) = default;

        EntryValueStorage &operator=(const EntryValueStorage &) = default;

        EntryValueStorage &operator=(EntryValueStorage &&) = default;

        ~EntryValueStorage() = default;

        operator value_type &() & {
            return value();
        }

        operator const value_type &() const & {
            return value();
        }

        operator value_type &&() && {
            return std::move(value());
        }

        [[nodiscard]] value_type &value() & {
            return value_;
        }

        [[nodiscard]] const value_type &value() const & {
            return value_;
        }

        [[nodiscard]] value_type &&value() && {
            return std::move(value_);
        }

        EntryT &operator=(const value_type &message) {
            value_ = message;
            /*
             * SAFETY: https://en.cppreference.com/w/cpp/language/static_cast paragraph (7.e):
             *  An rvalue(until C++11)/A prvalue(since C++11) of type “pointer to cv1 Base” can be explicitly converted
             *  to the type “pointer to cv2 Derived” if all following conditions are satisfied:
             *   - Derived is a complete class type.
             *   - Base is a base class of Derived.
             *   - cv1 is not a greater cv-qualification than cv2.
             */
            static_assert(std::is_base_of_v<EntryValueStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        EntryT &operator=(value_type &&message) {
            value_ = std::forward<value_type>(message);
            /*
             * SAFETY: https://en.cppreference.com/w/cpp/language/static_cast paragraph (7.e):
             *  An rvalue(until C++11)/A prvalue(since C++11) of type “pointer to cv1 Base” can be explicitly converted
             *  to the type “pointer to cv2 Derived” if all following conditions are satisfied:
             *   - Derived is a complete class type.
             *   - Base is a base class of Derived.
             *   - cv1 is not a greater cv-qualification than cv2.
             */
            static_assert(std::is_base_of_v<EntryValueStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        value_type *operator*() {
            return &value_;
        }

        const value_type *operator*() const {
            return &value_;
        }

        value_type *operator->() {
            return &value_;
        }

        const value_type *operator->() const {
            return &value_;
        }

    private:
        value_type value_;
    };

    template<typename ValuePtrT, typename EntryT>
    class EntryValueStorage<ValuePtrT, EntryT, std::enable_if_t<ros_signal::is_nullable_v<ValuePtrT>>> {
    public:
        using value_ptr_type = typename EntryTraits<ValuePtrT, EntryT>::value_ptr_type;
        using value_type = typename EntryTraits<ValuePtrT, EntryT>::value_type;

        explicit EntryValueStorage(const value_ptr_type &message_ptr)
            : value_ptr_(message_ptr) {
        }

        explicit EntryValueStorage(value_ptr_type &&message_ptr)
            : value_ptr_(std::forward<value_ptr_type>(message_ptr)) {
        }

        EntryValueStorage(const EntryValueStorage &) = default;

        EntryValueStorage(EntryValueStorage &&) = default;

        EntryValueStorage &operator=(const EntryValueStorage &) = default;

        EntryValueStorage &operator=(EntryValueStorage &&) = default;

        ~EntryValueStorage() = default;

        operator value_type &() & {
            return value();
        }

        operator const value_type &() const & {
            return value();
        }

        operator value_type &&() && {
            return std::move(value());
        }

        [[nodiscard]] value_type &value() & {
            return *value_ptr_;
        }

        [[nodiscard]] const value_type &value() const & {
            return *value_ptr_;
        }

        [[nodiscard]] value_type &&value() && {
            return std::move(*value_ptr_);
        }

        [[nodiscard]] value_ptr_type &value_ptr() & {
            return value_ptr_;
        }

        [[nodiscard]] const value_ptr_type &value_ptr() const & {
            return value_ptr_;
        }

        [[nodiscard]] value_ptr_type &&value_ptr() && {
            return std::move(value_ptr_);
        }

        EntryT &operator=(const value_type &message_ptr) {
            *value_ptr_ = message_ptr;
            static_assert(std::is_base_of_v<EntryValueStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        EntryT &operator=(value_type &&message_ptr) {
            *value_ptr_ = std::forward<value_type>(message_ptr);
            static_assert(std::is_base_of_v<EntryValueStorage, EntryT>);
            return *static_cast<EntryT *>(this);
        }

        value_type &operator*() {
            return *value_ptr_;
        }

        const value_type &operator*() const {
            return *value_ptr_;
        }

        value_type *operator->() {
            return &*value_ptr_;
        }

        const value_type *operator->() const {
            return &*value_ptr_;
        }

        [[nodiscard]] bool has_value() const {
            return value_ptr_ != nullptr;
        }

        template<typename... ArgTs>
        value_type &emplace(ArgTs &&...args) {
            *value_ptr_ = value_type(std::forward<ArgTs>(args)...);
            return value();
        }

        template<typename... ArgTs>
        value_type &emplace_ptr(ArgTs &&...args) {
            value_ptr_ = value_ptr_type(std::forward<ArgTs>(args)...);
            return value();
        }

    private:
        value_ptr_type value_ptr_;
    };

    template<typename ValueT, typename EntryT, typename AllocT>
    class EntryValueStorage<rclcpp::LoanedMessage<ValueT, AllocT>, EntryT> {
    public:
        using value_loaned_type = rclcpp::LoanedMessage<ValueT, AllocT>;
        using value_type = typename EntryTraits<ValueT, EntryT>::value_type;

        explicit EntryValueStorage(value_loaned_type &&loaned_message)
            : loaned_value_(std::forward<value_loaned_type>(loaned_message)) {
        }

        EntryValueStorage(const EntryValueStorage &) = default;

        EntryValueStorage(EntryValueStorage &&) = default;

        EntryValueStorage &operator=(const EntryValueStorage &) = default;

        EntryValueStorage &operator=(EntryValueStorage &&) = default;

        ~EntryValueStorage() = default;

        [[nodiscard]] value_type &value() & {
            return loaned_value_.get();
        }

        [[nodiscard]] const value_type &value() const & {
            return loaned_value_.get();
        }

        value_loaned_type &loaned_value() & {
            return loaned_value_;
        }

        const value_loaned_type &loaned_value() const & {
            return loaned_value_;
        }

        value_loaned_type loaned_value() && {
            return std::move(loaned_value_);
        }

        value_type &operator*() {
            return loaned_value_.get();
        }

        const value_type *&operator*() const {
            return loaned_value_.get();
        }

        value_type *operator->() {
            return &loaned_value_.get();
        }

        const value_type *operator->() const {
            return &loaned_value_.get();
        }

    private:
        value_loaned_type loaned_value_;
    };
}// namespace ros_signal::detail

#endif// ROS_SIGNAL_DETAIL_ENTRY_STORAGE_HPP
