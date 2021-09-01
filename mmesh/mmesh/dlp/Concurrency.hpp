#ifndef MMESH_CONCURRENCY_H
#define MMESH_CONCURRENCY_H

#ifdef USE_TBB_MODULE

#include <tbb/spin_mutex.h>
#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <numeric>

#include <memory>
#include <array>
#include <algorithm>
#include <ostream>
#include <iostream>
#include <math.h>
#include <queue>
#include <sstream>
#include <cstdio>
#include <stdint.h>
#include <stdarg.h>
#include <vector>
#include <cassert>
#include <cmath>
#include <type_traits>



namespace mmesh {

    using coord_t = int32_t;

    template<typename T, typename Q>
    inline T unscale(Q v) { return T(v) * T(SCALING_FACTOR); }

    enum Axis {
        X = 0,
        Y,
        Z,
        E,
        F,
        NUM_AXES,
        // For the GCodeReader to mark a parsed axis, which is not in "XYZEF", it was parsed correctly.
        UNKNOWN_AXIS = NUM_AXES,
        NUM_AXES_WITH_UNKNOWN,
    };

    template <typename T>
    inline void append(std::vector<T>& dest, const std::vector<T>& src)
    {
        if (dest.empty())
            dest = src;
        else
            dest.insert(dest.end(), src.begin(), src.end());
    }

    template <typename T>
    inline void append(std::vector<T>& dest, std::vector<T>&& src)
    {
        if (dest.empty())
            dest = std::move(src);
        else {
            dest.reserve(dest.size() + src.size());
            std::move(std::begin(src), std::end(src), std::back_inserter(dest));
        }
        src.clear();
        src.shrink_to_fit();
    }

    // Append the source in reverse.
    template <typename T>
    inline void append_reversed(std::vector<T>& dest, const std::vector<T>& src)
    {
        if (dest.empty())
            dest = src;
        else
            dest.insert(dest.end(), src.rbegin(), src.rend());
    }

    // Append the source in reverse.
    template <typename T>
    inline void append_reversed(std::vector<T>& dest, std::vector<T>&& src)
    {
        if (dest.empty())
            dest = std::move(src);
        else {
            dest.reserve(dest.size() + src.size());
            std::move(std::rbegin(src), std::rend(src), std::back_inserter(dest));
        }
        src.clear();
        src.shrink_to_fit();
    }

    // Casting an std::vector<> from one type to another type without warnings about a loss of accuracy.
    template<typename T_TO, typename T_FROM>
    std::vector<T_TO> cast(const std::vector<T_FROM>& src)
    {
        std::vector<T_TO> dst;
        dst.reserve(src.size());
        for (const T_FROM& a : src)
            dst.emplace_back((T_TO)a);
        return dst;
    }

    template <typename T>
    inline void remove_nulls(std::vector<T*>& vec)
    {
        vec.erase(
            std::remove_if(vec.begin(), vec.end(), [](const T* ptr) { return ptr == nullptr; }),
            vec.end());
    }

    template <typename T>
    inline void sort_remove_duplicates(std::vector<T>& vec)
    {
        std::sort(vec.begin(), vec.end());
        vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    }

    // Older compilers do not provide a std::make_unique template. Provide a simple one.
    template<typename T, typename... Args>
    inline std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

    // Variant of std::lower_bound() with compare predicate, but without the key.
    // This variant is very useful in case that the T type is large or it does not even have a public constructor.
    template<class ForwardIt, class LowerThanKeyPredicate>
    ForwardIt lower_bound_by_predicate(ForwardIt first, ForwardIt last, LowerThanKeyPredicate lower_thank_key)
    {
        ForwardIt it;
        typename std::iterator_traits<ForwardIt>::difference_type count, step;
        count = std::distance(first, last);

        while (count > 0) {
            it = first;
            step = count / 2;
            std::advance(it, step);
            if (lower_thank_key(*it)) {
                first = ++it;
                count -= step + 1;
            }
            else
                count = step;
        }
        return first;
    }

    // from https://en.cppreference.com/w/cpp/algorithm/lower_bound
    template<class ForwardIt, class T, class Compare = std::less<>>
    ForwardIt binary_find(ForwardIt first, ForwardIt last, const T& value, Compare comp = {})
    {
        // Note: BOTH type T and the type after ForwardIt is dereferenced 
        // must be implicitly convertible to BOTH Type1 and Type2, used in Compare. 
        // This is stricter than lower_bound requirement (see above)

        first = std::lower_bound(first, last, value, comp);
        return first != last && !comp(value, *first) ? first : last;
    }

    // from https://en.cppreference.com/w/cpp/algorithm/lower_bound
    template<class ForwardIt, class LowerThanKeyPredicate, class EqualToKeyPredicate>
    ForwardIt binary_find_by_predicate(ForwardIt first, ForwardIt last, LowerThanKeyPredicate lower_thank_key, EqualToKeyPredicate equal_to_key)
    {
        // Note: BOTH type T and the type after ForwardIt is dereferenced 
        // must be implicitly convertible to BOTH Type1 and Type2, used in Compare. 
        // This is stricter than lower_bound requirement (see above)

        first = lower_bound_by_predicate(first, last, lower_thank_key);
        return first != last && equal_to_key(*first) ? first : last;
    }

    template<typename T>
    static inline T sqr(T x)
    {
        return x * x;
    }

    template <typename T>
    static inline T clamp(const T low, const T high, const T value)
    {
        return std::max(low, std::min(high, value));
    }

    template <typename T, typename Number>
    static inline T lerp(const T& a, const T& b, Number t)
    {
        assert((t >= Number(-EPSILON)) && (t <= Number(1) + Number(EPSILON)));
        return (Number(1) - t) * a + t * b;
    }

    template <typename Number>
    static inline bool is_approx(Number value, Number test_value)
    {
        return std::fabs(double(value) - double(test_value)) < double(EPSILON);
    }

    // A meta-predicate which is true for integers wider than or equal to coord_t
    template<class I> struct is_scaled_coord
    {
        static const constexpr bool value =
            std::is_integral<I>::value &&
            std::numeric_limits<I>::digits >=
            std::numeric_limits<coord_t>::digits;
    };

    // Meta predicates for floating, 'scaled coord' and generic arithmetic types
    // Can be used to restrict templates to work for only the specified set of types.
    // parameter T is the type we want to restrict
    // parameter O (Optional defaults to T) is the type that the whole expression
    // will be evaluated to.
    // e.g. template<class T> FloatingOnly<T, bool> is_nan(T val);
    // The whole template will be defined only for floating point types and the
    // return type will be bool.
    // For more info how to use, see docs for std::enable_if
    //
    template<class T, class O = T>
    using FloatingOnly = std::enable_if_t<std::is_floating_point<T>::value, O>;

    template<class T, class O = T>
    using ScaledCoordOnly = std::enable_if_t<is_scaled_coord<T>::value, O>;

    template<class T, class O = T>
    using IntegerOnly = std::enable_if_t<std::is_integral<T>::value, O>;

    template<class T, class O = T>
    using ArithmeticOnly = std::enable_if_t<std::is_arithmetic<T>::value, O>;

    template<class T, class O = T>
    using IteratorOnly = std::enable_if_t<
        !std::is_same_v<typename std::iterator_traits<T>::value_type, void>, O
    >;

    template<class T, class I, class... Args> // Arbitrary allocator can be used
    IntegerOnly<I, std::vector<T, Args...>> reserve_vector(I capacity)
    {
        std::vector<T, Args...> ret;
        if (capacity > I(0)) ret.reserve(size_t(capacity));

        return ret;
    }

// Set this to true to enable full parallelism in this module.
// Only the well tested parts will be concurrent if this is set to false.
const constexpr bool USE_FULL_CONCURRENCY = true;


template<bool> struct _ccr {};

template<> struct _ccr<true>
{
    using SpinningMutex = tbb::spin_mutex;
    using BlockingMutex = tbb::mutex;

    template<class Fn, class It>
    static IteratorOnly<It, void> loop_(const tbb::blocked_range<It> &range, Fn &&fn)
    {
        for (auto &el : range) fn(el);
    }

    template<class Fn, class I>
    static IntegerOnly<I, void> loop_(const tbb::blocked_range<I> &range, Fn &&fn)
    {
        for (I i = range.begin(); i < range.end(); ++i) fn(i);
    }

    template<class It, class Fn>
    static void for_each(It from, It to, Fn &&fn, size_t granularity = 1)
    {
        tbb::parallel_for(tbb::blocked_range<It>{from, to, granularity},
                          [&fn, from](const auto &range) {
            loop_(range, std::forward<Fn>(fn));
        });
    }

    template<class I, class MergeFn, class T, class AccessFn>
    static T reduce(I          from,
                    I          to,
                    const T   &init,
                    MergeFn  &&mergefn,
                    AccessFn &&access,
                    size_t     granularity = 1
                    )
    {
        return tbb::parallel_reduce(
            tbb::blocked_range<I>{from, to, granularity}, init,
            [&](const auto &range, T subinit) {
                T acc = subinit;
                loop_(range, [&](auto &i) { acc = mergefn(acc, access(i)); });
                return acc;
            },
            std::forward<MergeFn>(mergefn));
    }

    template<class I, class MergeFn, class T>
    static IteratorOnly<I, T> reduce(I         from,
                                     I         to,
                                     const T & init,
                                     MergeFn &&mergefn,
                                     size_t    granularity = 1)
    {
        return reduce(
            from, to, init, std::forward<MergeFn>(mergefn),
            [](typename I::value_type &i) { return i; }, granularity);
    }
};

template<> struct _ccr<false>
{
private:
    struct _Mtx { inline void lock() {} inline void unlock() {} };
    
public:
    using SpinningMutex = _Mtx;
    using BlockingMutex = _Mtx;

    template<class Fn, class It>
    static IteratorOnly<It, void> loop_(It from, It to, Fn &&fn)
    {
        for (auto it = from; it != to; ++it) fn(*it);
    }

    template<class Fn, class I>
    static IntegerOnly<I, void> loop_(I from, I to, Fn &&fn)
    {
        for (I i = from; i < to; ++i) fn(i);
    }

    template<class It, class Fn>
    static void for_each(It   from,
                         It   to,
                         Fn &&fn,
                         size_t /* ignore granularity */ = 1)
    {
        loop_(from, to, std::forward<Fn>(fn));
    }

    template<class I, class MergeFn, class T, class AccessFn>
    static T reduce(I         from,
                    I         to,
                    const T & init,
                    MergeFn  &&mergefn,
                    AccessFn &&access,
                    size_t   /*granularity*/ = 1
                    )
    {
        T acc = init;
        loop_(from, to, [&](auto &i) { acc = mergefn(acc, access(i)); });
        return acc;
    }

    template<class I, class MergeFn, class T>
    static IteratorOnly<I, T> reduce(I          from,
                                     I          to,
                                     const T   &init,
                                     MergeFn  &&mergefn,
                                     size_t     /*granularity*/ = 1
                                     )
    {
        return reduce(from, to, init, std::forward<MergeFn>(mergefn),
                      [](typename I::value_type &i) { return i; });
    }
};

using ccr = _ccr<USE_FULL_CONCURRENCY>;
using ccr_seq = _ccr<false>;
using ccr_par = _ccr<true>;

} // namespace mmesh
#endif  //	 USE_TBB_MODULE
#endif // MMESHCONCURRENCY_H
