#pragma once

#include "sem.h"

/**
 * Smart locking object that uses a semaphore. It automatically
 * takes the lock when created and releases the lock when the object goes out of
 * scope. Use like this:
 *   gpm_sem_t my_lock;
 *   int ret = gpm_sem_init(&my_lock, 0, 1);
 *   ...
 *
 *   {
 *       SmartLock smart_lock(my_lock);
 *       //critical section start
 *       ...
 *       //critical section end
 *   }
 */
class SmartLock
{
public:
	SmartLock(gpm_sem_t &sem) : _sem(sem) { do {} while (gpm_sem_wait(&_sem) != 0); }
	~SmartLock() { gpm_sem_post(&_sem); }
private:
	gpm_sem_t &_sem;
};
