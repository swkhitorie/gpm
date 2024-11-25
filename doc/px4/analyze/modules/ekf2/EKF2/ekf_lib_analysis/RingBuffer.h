/**
 * @file RingBuffer.h
 * @author Roman Bapst <bapstroman@gmail.com>
 * Template RingBuffer.
 */

#include <inttypes.h>
#include <cstdio>
#include <cstring>

template <typename data_type>
class RingBuffer
{
public:
	RingBuffer()
	{
		if (allocate(1)) {
			// initialize with one empty sample
			data_type d = {};
			push(d);
		}
	}
	~RingBuffer() { delete[] _buffer; }

	// no copy, assignment, move, move assignment
	RingBuffer(const RingBuffer &) = delete;
	RingBuffer &operator=(const RingBuffer &) = delete;
	RingBuffer(RingBuffer &&) = delete;
	RingBuffer &operator=(RingBuffer &&) = delete;

	/***************************
		分配环形数组, 如果已经分配, 则重新分配
	***************************/
	bool allocate(uint8_t size)
	{

		if (_buffer != nullptr) {
			delete[] _buffer;
		}

		_buffer = new data_type[size];

		if (_buffer == nullptr) {
			return false;
		}

		_size = size;

		_head = 0;
		_tail = 0;

		// set the time elements to zero so that bad data is not
		// retrieved from the buffers
		for (uint8_t index = 0; index < _size; index++) {
			_buffer[index] = {};
		}

		_first_write = true;

		return true;
	}

	/***************************
		释放环形数组
	***************************/
	void unallocate()
	{
		delete[] _buffer;
		_buffer = nullptr;
	}

	/***************************
		push一个数据到环形数组中 _head索引始终指向最新的数据, _tail索引始终指向最老的数据
	***************************/
	void push(const data_type &sample)
	{

		uint8_t head_new = _head;

		if (!_first_write) {
			head_new = (_head + 1) % _size;
		}

		_buffer[head_new] = sample;
		_head = head_new;

		// move tail if we overwrite it
		if (_head == _tail && !_first_write) {
			_tail = (_tail + 1) % _size;

		} else {
			_first_write = false;
		}
	}

	uint8_t get_length() const { return _size; }

	data_type &operator[](const uint8_t index) { return _buffer[index]; }

	const data_type &get_newest() { return _buffer[_head]; }
	const data_type &get_oldest() { return _buffer[_tail]; }

	uint8_t get_oldest_index() const { return _tail; }

	/***************************
		弹出比指定时间戳数据还要老的数据
		找到返回true, 没找到返回false
		如果 idx_timstamp < user_timestamp < idx_timstamp + 0.1s, 则找到比指定时间戳要老的数据
	***************************/
	bool pop_first_older_than(const uint64_t &timestamp, data_type *sample)
	{
		// start looking from newest observation data
		for (uint8_t i = 0; i < _size; i++) {
			int index = (_head - i);
			index = index < 0 ? _size + index : index;

			if (timestamp >= _buffer[index].time_us && timestamp < _buffer[index].time_us + (uint64_t)1e5) {
				*sample = _buffer[index];

				// Now we can set the tail to the item which
				// comes after the one we removed since we don't
				// want to have any older data in the buffer
				if (index == _head) {
					_tail = _head;
					_first_write = true;

				} else {
					_tail = (index + 1) % _size;
				}

				_buffer[index].time_us = 0;

				return true;
			}

			if (index == _tail) {
				// we have reached the tail and haven't got a
				// match
				return false;
			}
		}

		return false;
	}

	int get_total_size() { return sizeof(*this) + sizeof(data_type) * _size; }

private:
	data_type *_buffer{nullptr};

	uint8_t _head{0};
	uint8_t _tail{0};
	uint8_t _size{0};

	bool _first_write{true};
};
