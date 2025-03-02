
#ifndef __FREE_RTOS_OPERATORS_HPP
#define __FREE_RTOS_OPERATORS_HPP
#include "FreeRTOS.h"
#include "task.h"

void * operator new( size_t size )
{
    return pvPortMalloc(size);
}

void * operator new[]( size_t size )
{
    return pvPortMalloc( size );
}

void operator delete( void * ptr ) noexcept
{
    vPortFree( ptr );
}

void operator delete(void* p, size_t size) noexcept 
{
    vPortFree(p);
}

void operator delete [](void * ptr, size_t size) noexcept
{
    vPortFree( ptr );
}

void operator delete[]( void * ptr ) noexcept
{
    vPortFree( ptr );
}
#endif //__FREE_RTOS_OPERATORS_HPP