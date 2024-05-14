//=============================================================================
//=============================================================================
// File:    CoreQueue.h
// Brief:   File with prototypes for exchanging data between RP2040 cores
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/13/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
//=============================================================================

//=============================================================================
// Include Files
//=============================================================================
#include "CoreQueue.h"

//=========================================================================
// Name:        QueueFIFO::begin
// Brief:       The begin/initialization function for the queue
//=========================================================================
void QueueFIFO::begin()
{
    // Initialize the queue
    queue_init(&_queue32, sizeof(uint32_t), _qsize);
    queue_init(&_queuef, sizeof(float), _qsize);
}

//=========================================================================
// Name:        QueueFIFO::push(float)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// NOTE:        This function blocks until space is available
//=========================================================================
void QueueFIFO::push(float data) 
{
    while (!push_nb(data));
}

//=========================================================================
// Name:        QueueFIFO::push_nb(float)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::push_nb(float data) 
{
    return queue_try_add(&_queuef, &data);
}

//=========================================================================
// Name:        QueueFIFO::push(uint32_t)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// NOTE:        This function blocks until space is available
//=========================================================================
void QueueFIFO::push(uint32_t data) 
{
    while (!push_nb(data));
}

//=========================================================================
// Name:        QueueFIFO::push_nb(uint32_t)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::push_nb(uint32_t data) 
{
    return queue_try_add(&_queue32, &data);
}

//=========================================================================
// Name:        QueueFIFO::popf(float)
// Brief:       Pops float data from the queue
// Retval:      The FIFO data
// NOTE:        This function blocks until data is successfully popped
//=========================================================================
float QueueFIFO::popf() 
{
    float ret;
    while (!popf_nb(&ret));
    return ret;
}

//=========================================================================
// Name:        QueueFIFO::popf_nb(float)
// Brief:       Pops float data from the queue
// Param[out]:  data = The popped value
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::popf_nb(float *data) 
{
    if (queue_get_level(&_queuef))
    {
        return queue_try_remove(&_queuef, &data);
    }
    else
    {
        return 0;
    }
}

//=========================================================================
// Name:        QueueFIFO::pop32(uint32)
// Brief:       Pops uint32 data from the queue
// Retval:      The FIFO data
// NOTE:        This function blocks until data is successfully popped
//=========================================================================
uint32_t QueueFIFO::pop32() 
{
    uint32_t ret;
    while (!pop32_nb(&ret));
    return ret;
}

//=========================================================================
// Name:        QueueFIFO::pop32_nb(uint32_t)
// Brief:       Pops float data from the queue
// Param[out]:  data = The popped value
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::pop32_nb(uint32_t *data) 
{
    if (queue_get_level(&_queue32))
    {
        return queue_try_remove(&_queue32, &data);\
    }
    else
    {
        return 0;
    }
}

//=========================================================================
// Name:        QueueFIFO::available
// Brief:       Check if there is data in the queue
// Retval:      >0 for data available
// NOTE:        This function blocks until data is successfully popped
//=========================================================================
int QueueFIFO::available() 
{
    return (queue_get_level(&_queue32) || queue_get_level(&_queuef));
}

//=========================================================================
// End of File
//=========================================================================