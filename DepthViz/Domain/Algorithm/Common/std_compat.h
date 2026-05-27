//
//  std_compat.h
//  DepthViz
//
//  C++17 compatibility for removed standard library components
//  Provides drop-in replacements for std::binary_function and std::unary_function
//

#ifndef std_compat_h
#define std_compat_h

#include <functional>

// C++17+ removed std::binary_function and std::unary_function
// These compatibility definitions allow older code (like Boost) to compile

namespace std {

#if __cplusplus >= 201703L
    // In C++17+, define these for backward compatibility
    
    /// \brief Binary function base class (removed in C++17)
    /// Provides type aliases for binary functors
    template<typename Arg1, typename Arg2, typename Result>
    struct binary_function {
        typedef Arg1 first_argument_type;   ///< First argument type
        typedef Arg2 second_argument_type;  ///< Second argument type
        typedef Result result_type;         ///< Result type
    };
    
    /// \brief Unary function base class (removed in C++17)
    /// Provides type aliases for unary functors
    template<typename Arg, typename Result>
    struct unary_function {
        typedef Arg argument_type;  ///< Argument type
        typedef Result result_type; ///< Result type
    };
    
#endif

} // namespace std

// Additionally provide boost compatibility if needed
namespace boost {
#if __cplusplus >= 201703L
    
    template<typename Arg1, typename Arg2, typename Result>
    struct binary_function {
        typedef Arg1 first_argument_type;
        typedef Arg2 second_argument_type;
        typedef Result result_type;
    };
    
    template<typename Arg, typename Result>
    struct unary_function {
        typedef Arg argument_type;
        typedef Result result_type;
    };
    
#endif
} // namespace boost

#endif /* std_compat_h */
