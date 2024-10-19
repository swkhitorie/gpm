
#ifndef __ECONTAINER_DRIVER_H_
#define __ECONTAINER_DRIVER_H_

#include "econtainer_type.hpp"
#include "utility/thread_manager.hpp"

namespace ESAF
{
enum DEVBUFFER_TYPE { DEVBUFIN = 0x00, DEVBUFOUT = 0x01};

template<typename _Tp>
class edevbuffer
{
public:
    typedef _Tp value_type;
    typedef size_t size_type;
    typedef value_type* iterator;
    typedef const value_type* const_iterator;
public:
    edevbuffer() = delete;
    edevbuffer(size_t size, Mutex *rm = NULL, Mutex *wm = NULL)
        : _write_overflow(0), _read_overflow(0)
    {
        _read_buffer = new efifobuffer<value_type>(size);
        _write_buffer = new efifobuffer<value_type>(size);
        if (wm) {
            _writeMutex = wm;
        } else {
            _writeMutex = new MutexDefault();
        }
        if (rm) {
            _readMutex = rm;
        } else {
            _readMutex = new MutexDefault();
        }
    }
    edevbuffer(size_t read_size, size_t write_size, Mutex *rm = NULL, Mutex *wm = NULL) 
        : _write_overflow(0), _read_overflow(0)
    {
        _read_buffer = new efifobuffer<value_type>(read_size);
        _write_buffer = new efifobuffer<value_type>(write_size);
        if (wm) {
            _writeMutex = wm;
        } else {
            _writeMutex = new MutexDefault();
        }
        if (rm) {
            _readMutex = rm;
        } else {
            _readMutex = new MutexDefault();
        }
    }
    ~edevbuffer() {
        delete _read_buffer;
        delete _write_buffer;
        delete _writeMutex;
        delete _readMutex;
    }

	void clear(enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            if (_read_buffer != NULL) {
                if (!_readMutex->islocked()) {
                    _readMutex->lock();
                    _read_buffer->clear();
                    _readMutex->unlock();
                }
            }
            break;
        case DEVBUFOUT:
            if (_write_buffer != NULL) {
                if (!_writeMutex->islocked()) {
                    _writeMutex->lock();
                    _write_buffer->clear();
                    _writeMutex->unlock();
                }
            }
            break;
        }       
    }
	void size(size_type &size, enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            if (_read_buffer != NULL) {
                if (!_readMutex->islocked()) {
                    _readMutex->lock();
                    size = _read_buffer->size();
                    _readMutex->unlock();
                }
            }
            break;
        case DEVBUFOUT:
            if (_write_buffer != NULL) {
                if (!_writeMutex->islocked()) {
                    _writeMutex->lock();
                    size = _write_buffer->size();
                    _writeMutex->unlock();
                }
            }
            break;
        }
    }

	void capacity(size_type &_capacity, enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            if (_read_buffer != NULL) {
                _capacity = _read_buffer->capacity();
            }
            break;
        case DEVBUFOUT:
            if (_write_buffer != NULL) {
                _capacity = _write_buffer->capacity();
            }
            break;
        }
    }
	size_type overflow(enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            return _read_overflow;
        case DEVBUFOUT:
            return _write_overflow;
        default: return 0;
        }
    }
	bool puts(const_iterator pdata, size_type size, enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            {
                if (_read_buffer == NULL)
                    return false;
                if (!_readMutex->islocked()) {
                    _readMutex->lock();
                    bool res = _read_buffer->write(pdata, size);
                    if (!res) {
                        _read_overflow += size - (_read_buffer->capacity() - _read_buffer->size());
                    }
                    _readMutex->unlock();
                    return res;
                } else {
                    return false;
                }
            }
        case DEVBUFOUT:
            {
                if (_write_buffer == NULL)
                    return false;
                if (!_writeMutex->islocked()) {
                    _writeMutex->lock();
                    bool res = _write_buffer->write(pdata, size);
                    if (!res) {
                        _write_overflow += size - (_write_buffer->capacity() - _write_buffer->size());
                    }
                    _writeMutex->unlock();
                    return res;
                } else {
                    return false;
                }        
            }
        default : return false;
        }
        return false;
    }
	bool gets(iterator pdata, size_type size, enum DEVBUFFER_TYPE type)
    {
        switch (type) {
        case DEVBUFIN:
            {
                if (_read_buffer == NULL)
                    return false;
                if (!_readMutex->islocked()) {
                    _readMutex->lock();
                    bool res = _read_buffer->read(pdata, size);
                    if (!res) {
                        _read_overflow += size - (_read_buffer->capacity() - _read_buffer->size());
                    }
                    _readMutex->unlock();
                    return res; 
                } else {
                    return false;
                }
            }
        case DEVBUFOUT:
            {
                if (_write_buffer == NULL)
                    return false;
                if (!_writeMutex->islocked()) {
                    _writeMutex->lock();
                    bool res = _write_buffer->read(pdata, size);
                    if (!res) {
                        _write_overflow += size - (_write_buffer->capacity() - _write_buffer->size());
                    }
                    _writeMutex->unlock();
                    return res;
                } else {
                    return false;
                }
            }
        default : return false;
        }
        return false;
    }
    bool rxquery(size_type __offset, iterator __p, size_type size)
    {
        if (_read_buffer == NULL)
            return false;
        if (!_readMutex->islocked()) {
            _readMutex->lock();
            bool res = _read_buffer->search(__offset, __p, size);
            if (!res) {
                _read_overflow += size - (_read_buffer->capacity() - _read_buffer->size());
            }
            _readMutex->unlock();
            return res;
        } else {
            return false;
        }
    }
public:
	size_type _write_overflow;
	size_type _read_overflow;
    efifobuffer<value_type> *_read_buffer;
    efifobuffer<value_type> *_write_buffer;
	Mutex *_writeMutex;
	Mutex *_readMutex;
	value_type _write_value;
	value_type _read_value;
};

}  // namespace ESAF

#endif  // econtainer_driver_H
