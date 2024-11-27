
#ifndef __THREAD_MANAGER_H_
#define __THREAD_MANAGER_H_

namespace ESAF
{

class Mutex
{
  public:
    Mutex();
    virtual ~Mutex();
    virtual void lock() = 0;

  public:
    virtual void unlock()   = 0;
    virtual bool islocked() = 0;
};  // class Mutex

class MutexPrivate;
class MutexDefault : public Mutex
{
  public:
    MutexDefault();
    ~MutexDefault();

  public:
    void lock();
    void unlock();
    bool islocked();

  private:
    MutexPrivate *instance;
};  // class MutexDefault

class ThreadAbstract
{
  public:
    ThreadAbstract();
    virtual ~ThreadAbstract();

    // Mutex operations
  public:
    virtual void lockRecvContainer() = 0;
    virtual void freeRecvContainer() = 0;

    virtual void lockMSG() = 0;
    virtual void freeMSG() = 0;

    virtual void lockACK() = 0;
    virtual void freeACK() = 0;

    virtual void lockProtocolHeader();
    virtual void freeProtocolHeader();

    virtual void lockNonBlockCBAck();
    virtual void freeNonBlockCBAck();

    virtual void notifyNonBlockCBAckRecv();
    virtual void nonBlockWait();

    virtual void lockStopCond();
    virtual void freeStopCond();

    virtual void lockFrame();
    virtual void freeFrame();

    // Thread comm/sync
  public:
    virtual void notify()          = 0;
    virtual void wait(int timeout) = 0;

  public:
    virtual void init() = 0;
};  // class ThreadAbstract

class Thread
{
  public:
    Thread();
    virtual ~Thread();

    virtual bool createThread() = 0;
    virtual int stopThread()    = 0;

    bool getStopCondition();
    void setStopCondition(bool condition);

  protected:
    int type;
    bool stop_condition;
};  // class Thread

}  // namespace ESAF

#endif  // thread_manager_H
