
#ifndef __EMEM_RELOCATION_H_
#define __EMEM_RELOCATION_H_

#include <new>

//#define USE_THROW_CXX_MEM

// Global memory manager
void osEEnterCritical();
void osEExitCritical();

/**
 * @brief system memory manager interface, malloc
 * @param size user size wanted
 * @return void* allocated memory
 */
void *sys_mem_malloc(size_t size);

/**
 * @brief system memory manager interface, free
 * @param pointer user pointer want release
 */
void sys_mem_free(void *pointer);
void *sys_mem_realloc(void *p, size_t old_sz, size_t new_sz);

/**
 * @brief redirect new operator
 * @param size user size wanted
 * @return void* allocated memory
 */
void *operator new(size_t size);
void *operator new[](size_t size);

/**
 * @brief redirect delete operator
 * @param pointer user pointer want release
 */
void operator delete(void *pointer);
void operator delete[](void *pointer);

#endif  // ememredirect_H
