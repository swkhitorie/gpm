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


template <size_t N>
class AtomicBitset
{
public:

	AtomicBitset() = default;

	size_t count() const
	{
		size_t total = 0;
		for (const auto &x : _data) {
			uint32_t y = x.load();
			while (y) {
				total += y & 1;
				y >>= 1;
			}
		}
		return total;
	}

	size_t size() const { return N; }

	bool operator[](size_t position) const
	{
		return _data[array_index(position)].load() & element_mask(position);
	}

	void set(size_t pos, bool val = true)
	{
		const uint32_t bitmask = element_mask(pos);
		if (val) {
			_data[array_index(pos)].fetch_or(bitmask);
		} else {
			_data[array_index(pos)].fetch_and(~bitmask);
		}
	}

private:
	static constexpr uint8_t BITS_PER_ELEMENT = 32;
	static constexpr size_t ARRAY_SIZE = ((N % BITS_PER_ELEMENT) == 0) ? (N / BITS_PER_ELEMENT) :
                            (N / BITS_PER_ELEMENT + 1);
	static constexpr size_t ALLOCATED_BITS = ARRAY_SIZE * BITS_PER_ELEMENT;

	size_t array_index(size_t position) const { return position / BITS_PER_ELEMENT; }
	uint32_t element_mask(size_t position) const { return (1 << (position % BITS_PER_ELEMENT)); }

	gpm::atomic<uint32_t> _data[ARRAY_SIZE];
};

}

#endif /* __cplusplus */
