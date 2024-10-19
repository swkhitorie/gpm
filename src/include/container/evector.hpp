
#ifndef _EVECTOR_H_
#define _EVECTOR_H_

#include "eallocator.hpp"
#include "ememprocess.hpp"
#include "econstructor.hpp"

#define emax(x, y) ((x > y) ? x : y)

namespace ESAF 
{
template <typename _Tp>
class evector
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
    typedef ptrdiff_t difference_type;
    typedef eallocator<value_type> data_allocator;
public:
    evector() : _start(0), _finish(0), _end_of_storage(0) {}
    evector(size_type n) {
        initialize(n);
    }
    evector(size_type n, const_reference value) {
        fill_initialize(n, value);
    }
    ~evector() {
        get_allocator().deallocate(_start, capacity()); 
    }

    iterator begin() { return _start; }
    iterator end() { return _finish; }
    size_type size() { return size_type(end() - begin()); }
    size_type capacity() { return size_type(_end_of_storage - begin()); }
    bool empty() { return begin() == end(); }
    bool full() { return (end() == _end_of_storage); }
    void clear() { _finish = _start; }
    reference front() { return *begin(); }
    reference back() { return *(end() - 1); }
    reference at(size_type __n) { return (*this)[__n]; }
    const_reference at(size_type __n) const { return (*this)[__n]; }
    reference operator[](size_type n) { return *(begin() + n); }

    void insert(iterator __position, const_reference __x) {
        if (_finish != _end_of_storage) {
            *__position = __x;
            ++_finish;
        }
    }
    void push_back(const_reference __x) {
        if (_finish != _end_of_storage) {
            _construct(_finish, __x);
            ++_finish;
        }else {
            insert_aux(end(), __x);
        }
    }
    void pop_back() {
        --_finish;
        destroy(_finish);
    }
    iterator erase(iterator position) {
        if (position + 1 != end()) 
            copy(position + 1, _finish, position);
        --_finish;
        destroy(_finish);
        return position;
    }
    iterator erase(iterator first, iterator last) {
        iterator i = copy(last, _finish, first);
        destroy(i, _finish);
        _finish = _finish - (last - first);
        return first;
    }
    void insert_aux(iterator position, const_reference x) {
        if (_finish != _end_of_storage) {
            _construct(_finish, *(_finish - 1));
            ++_finish;
            value_type x_copy = x;
            copy_backward(position, _finish - 2, _finish - 1);
            *position = x_copy;
        }else {
            const size_type old_size = size();
            const size_type len = old_size != 0 ? 2 * old_size : 1;
            iterator new_start = get_allocator().allocate(len);
            iterator new_finish = new_start;

            // try {
            new_finish = uninitialized_copy(_start, position, new_start);
            _construct(new_finish, x);
            ++new_finish;
            new_finish = uninitialized_copy(position, _finish, new_finish);
            // }

            // catch(...) {
            // destroy(new_start, new_finish);
            // get_allocator().deallocate(new_start,len);
            // throw;
            //}
            destroy(begin(), end());
            get_allocator().deallocate(begin(), capacity());
            _start = new_start;
            _finish = new_finish;
            _end_of_storage = _finish;
        }
    }
    void insert(iterator position, size_type n, const_reference x) {
        if (n != 0) {
            if (size_type(_end_of_storage - _finish) >= n) {
                value_type x_copy = x;
                const size_type elem_after = _finish - position;
                iterator old_finish = _finish;
                if (elem_after > n) {
                    uninitialized_copy(_finish - n, _finish, _finish);
                    _finish += n;
                    copy_backward(position, old_finish - n, old_finish);
                    fill(position, position + n, x_copy);
                }else {
                    uninitialized_fill_n(_finish, n - elem_after, x_copy);
                    _finish += n - elem_after;
                    uninitialized_copy(position, old_finish, _finish);
                    _finish += elem_after;
                    fill(position, old_finish, x_copy);
                }
            }else {
                const size_type old_size = size();
                const size_type len = old_size + emax(old_size, n);
                iterator new_start = get_allocator().allocate(len);
                iterator new_finish = new_start;

                // try {
                new_finish = uninitialized_copy(_start, position, new_start);
                new_finish = uninitialized_fill_n(new_finish, n, x);
                new_finish = uninitialized_copy(position, _finish, new_finish);
                // }

                // catch(...) {
                // destroy(new_start, new_finish);
                // get_allocator().deallocate(new_start, len);
                // }
                destroy(_start, _finish);
                get_allocator().deallocate(begin(),capacity());
                _start = new_start;
                _finish = new_finish;
                _end_of_storage = _finish;
            }
        }
    }

protected:
    data_allocator& get_allocator() {
        return _mdata_allocator;
    }

    iterator allocate_and_fill(size_type n, const_reference value) {
        iterator result = get_allocator().allocate(n);
        for (size_t i = 0; i < n; i++)
            _construct(result + i);
        uninitialized_fill_n(result, n, value);
        return result;
    }

    void fill_initialize(size_type n, const_reference value) {
        _start = allocate_and_fill(n, value);
        _finish = _start + n;
        _end_of_storage = _finish;
    }

    void initialize(size_type n) {
        _start = get_allocator().allocate(n);
        for (size_t i = 0; i < n; i++)
            _construct(_start + i);
        _finish = _start;
        _end_of_storage = _start + n;        
    }

public:
    iterator _start;
    iterator _finish;
    iterator _end_of_storage;
    data_allocator _mdata_allocator;
};

}  // namespace ESAF

#endif  // evector_H
