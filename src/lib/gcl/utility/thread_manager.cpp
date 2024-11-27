#include "thread_manager.hpp"

namespace ESAF
{
class MutexPrivate
{
  public:
    MutexPrivate() {}
    ~MutexPrivate() {}

  public:
    inline void lock() {}
    inline void unlock() {}
    inline bool islocked() { return false; }
};  // class MutexPrivate

}  // namespace ESAF

using namespace ESAF;

Thread::Thread() {}

Thread::~Thread() {}

bool Thread::getStopCondition()
{
    return this->stop_condition;
}

void Thread::setStopCondition(bool condition)
{
    this->stop_condition = condition;
}

void ThreadAbstract::lockProtocolHeader()
{
    ;
}

void ThreadAbstract::freeProtocolHeader()
{
    ;
}

void ThreadAbstract::lockNonBlockCBAck()
{
    ;
}

void ThreadAbstract::freeNonBlockCBAck()
{
    ;
}

void ThreadAbstract::notifyNonBlockCBAckRecv()
{
    ;
}

void ThreadAbstract::nonBlockWait()
{
    ;
}

void ThreadAbstract::lockStopCond()
{
    ;
}

void ThreadAbstract::freeStopCond()
{
    ;
}

void ThreadAbstract::lockFrame()
{
    ;
}

void ThreadAbstract::freeFrame()
{
    ;
}

Mutex::Mutex() {}

Mutex::~Mutex() {}

MutexDefault::MutexDefault() : instance(new MutexPrivate()) {}

MutexDefault::~MutexDefault()
{
    delete instance;
}

void MutexDefault::lock()
{
    instance->lock();
}

void MutexDefault::unlock()
{
    instance->unlock();
}

bool MutexDefault::islocked()
{
    return instance->islocked();
}
