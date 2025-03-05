#ifndef ROS_SIGNAL_MACROS_HPP
#define ROS_SIGNAL_MACROS_HPP

#ifndef ros_signal_assert
#    ifndef NDEBUG
#        include <cassert>
#        define ros_signal_assert(expr) assert(expr)
#    else // NDEBUG
#        define ros_signal_assert(expr) ((void) (expr))
#    endif // NDEBUG
#endif // ros_signal_assert

#endif // ROS_SIGNAL_MACROS_HPP
