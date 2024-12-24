#pragma once

#ifdef __cplusplus

#include <stdbool.h>
#include <stdint.h>

namespace gpm
{

template <typename T>
class atomic
{
public:

#if 0
	// Ensure that all operations are lock-free, so that 'atomic' can be used from
	// IRQ handlers. This might not be required everywhere though.
	static_assert(__atomic_always_lock_free(sizeof(T), 0), "atomic is not lock-free for the given type T");
#endif

	atomic() = default;
	explicit atomic(T value) : _value(value) {}

	inline T load() const { return __atomic_load_n(&_value, __ATOMIC_SEQ_CST); }

	inline void store(T value) { __atomic_store(&_value, &value, __ATOMIC_SEQ_CST); }

	inline T fetch_add(T num) { return __atomic_fetch_add(&_value, num, __ATOMIC_SEQ_CST); }

	inline T fetch_sub(T num) { return __atomic_fetch_sub(&_value, num, __ATOMIC_SEQ_CST); }

	inline T fetch_and(T num) { return __atomic_fetch_and(&_value, num, __ATOMIC_SEQ_CST); }

	inline T fetch_xor(T num) { return __atomic_fetch_xor(&_value, num, __ATOMIC_SEQ_CST); }

	inline T fetch_or(T num) { return __atomic_fetch_or(&_value, num, __ATOMIC_SEQ_CST); }

	inline T fetch_nand(T num) { return __atomic_fetch_nand(&_value, num, __ATOMIC_SEQ_CST); }

	inline bool compare_exchange(T *expected, T desired)
	{
		return __atomic_compare_exchange(&_value, expected, &desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

private:
	T _value {};
};

using atomic_int = atomic<int>;
using atomic_int32_t = atomic<int32_t>;
using atomic_bool = atomic<bool>;

}

#endif /* __cplusplus */
