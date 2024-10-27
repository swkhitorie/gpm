#include "ememrelocation.hpp"

static void *nullback() { return NULL; }
void osEEnterCritical() {}
void osEExitCritical() {}

void *sys_mem_malloc(size_t size)
{
    osEEnterCritical();
    void *mem = malloc(size);
    osEExitCritical();
    return mem;
}

void sys_mem_free(void *pointer)
{
    osEEnterCritical();
    free(pointer);
    osEExitCritical();
}

void *sys_mem_realloc(void *p, size_t old_sz, size_t new_sz)
{
    sys_mem_free(p);
    return sys_mem_malloc(new_sz);
}

void *operator new(size_t size)
{
    if (void *mem = sys_mem_malloc(size))
    {
        return mem;
    }
    else
    {
#ifdef USE_THROW_CXX_MEM
        throw std::bad_alloc();
#endif
        return nullback();
    }
}

void *operator new[](size_t size)
{
    if (void *mem = sys_mem_malloc(size))
    {
        return mem;
    }
    else
    {
#ifdef USE_THROW_CXX_MEM
        throw std::bad_alloc();
#endif
        return nullback();
    }
}

void operator delete(void *pointer)  // noexcept
{
    sys_mem_free(pointer);
    pointer = nullptr;
}

void operator delete[](void *pointer)  // noexcept
{
    sys_mem_free(pointer);
    pointer = nullptr;
}
