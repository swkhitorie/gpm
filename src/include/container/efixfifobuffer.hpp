
#ifndef __EFIXFIFOBUFFER_H_
#define __EFIXFIFOBUFFER_H_

#include "stddef.h"

namespace ESAF 
{
template <typename _Tp, size_t capacity>
class efixfifobuffer
{
public:
    typedef _Tp value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type* iterator;
    typedef const value_type* const_iterator;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef size_t size_type;
    typedef size_t fifo_index;
    typedef char fifo_loop;
public:
    efixfifobuffer() : _read_index(0), _write_index(0), _read_ctrl(0), _write_ctrl(0), _size(0) {
        for (size_t i = 0; i < capacity; i++)
            _mem_start[i] = 0;    
    }
    ~efixfifobuffer() {}
    iterator mem_begin() { return &_mem_start[0]; }
    iterator mem_end() { return &_mem_start[capacity]; }
    iterator begin() { return &_mem_start[0] + _read_index; }
    iterator end() { return &_mem_start[0] + _write_index; }
    iterator write_begin() { return &_mem_start[0] + _write_index; }
    iterator write_end() { return &_mem_start[0] + _read_index; }
    size_type size() { return _size; }
    size_type fixcapacity() { return capacity; }
    bool empty() { 
        if (_read_index == _write_index) 
            return (_read_ctrl == _write_ctrl) ? true : false;
        else return false;
    }

    bool full() {
        if (_read_index == _write_index)
            return (_read_ctrl != _write_ctrl) ? true : false;
        else return false;
    }

    void clear() {
        _size = 0;
        _read_index = 0;
        _read_ctrl = 0;
        _write_index = 0;
        _write_ctrl = 0;
    }

	bool write(const_pointer __pdata, size_t __len)
    {
        size_t max_size = fixcapacity();
        if (full() || ((_size + __len) > max_size))
            return false;
        /* if array put-process will be begin from 0 index or not */
        if ((_write_index + __len) > max_size) {
            size_t res_size = max_size - _write_index;
            /* put to first part */
            copy_size(__pdata, res_size, &_mem_start[0] + _write_index);
            /* write pointer overflow */
            _write_index = 0;
            loop_buffer_ctrl(_write_ctrl);
            /* put to remain part */
            copy_size(__pdata + res_size, (__len - res_size), &_mem_start[0]);
            _write_index += __len - res_size;
        }else {
            copy_size(__pdata, __len, &_mem_start[0] + _write_index);
            _write_index += __len;
            if (_write_index == max_size) {
                _write_index = 0;
                loop_buffer_ctrl(_write_ctrl);
            }
        }
        _size += __len;
        return true;
    }

	bool read(pointer __pdata, size_t __len)
    {
        size_t max_size = fixcapacity();

        if (empty() || (__len > _size))
            return false;
        
        /* if array get-process will be begin from 0 index or not */
        if ((_read_index + __len) > max_size) {
            size_t res_size = max_size - _read_index;
            /* get from first part */
            copy_size((&_mem_start[0] + _read_index), res_size, __pdata);
            /* read pointer overflow */
            _read_index = 0;
            loop_buffer_ctrl(_read_ctrl);
            /* get from remain part */
            copy_size(&_mem_start[0], (__len - res_size), (__pdata + res_size));
            _read_index += 	__len - res_size;
        }else {
            /* directly get from buffer */
            copy_size((&_mem_start[0] + _read_index), __len, __pdata);
            _read_index += __len;
            if (_read_index == max_size) {
                _read_index = 0;
                loop_buffer_ctrl(_read_ctrl);
            }
        }
        _size -= __len;
        return true;
    }

	bool search(size_t __offset, pointer __pdata, size_t __len)
    {
        size_t _s_read_index = _read_index + __offset;
        size_t max_size = fixcapacity();
        
        if (empty() || (__offset + __len) > _size)
            return false;
        
        if (_s_read_index >= max_size) {
            _s_read_index -= max_size;
            /* directly get from buffer */
            copy_size((&_mem_start[0] + _s_read_index), __len, __pdata);
        }else {
            /* if query index will be begin from 0 index or not */
            if ((_s_read_index + __len) > max_size) {
                size_t res_size = max_size - _s_read_index;
                /* get from first part */
                copy_size((&_mem_start[0] + _s_read_index), res_size, __pdata);
                copy_size(&_mem_start[0], __len - res_size, __pdata + res_size);			
            }else {
                /* directly get from buffer */
                copy_size((&_mem_start[0] + _s_read_index), __len, __pdata);
            }
        }
        return true;
    }

protected:
    iterator copy(iterator first, iterator last, iterator result) {
        while (first != last) {
            *result = *first;
            ++result;
            ++first;
        }
        return result;
    }

    void fill(iterator first, size_type n, const_reference &result) {
        size_type i = 0;
        for (; i < n; i++) {
            *first = result;
            ++first;           
        }
    }

    void copy_size(const_iterator src, size_type n, iterator dst) {
        size_type i = 0;
        for (; i < n; i++) {
            dst[i] = src[i];
        }
    }

    void loop_buffer_ctrl(fifo_loop &value) {
        (value == 0) ? value = 1 : value = 0;
    }

public:
    fifo_index _read_index;
    fifo_index _write_index;

    /**
     * @note init 0 -> turn to !value, every time read index go around 
     */
    fifo_loop _read_ctrl;
    fifo_loop _write_ctrl;
    size_t _size;
    value_type _mem_start[capacity];
};

}  // namespace ESAF

#endif  // efixfifobuffer_H
