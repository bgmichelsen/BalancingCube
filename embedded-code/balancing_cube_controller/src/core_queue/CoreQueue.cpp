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
    queue_init(&_queue, sizeof(queue_cmd_t), _qsize);
}

//=========================================================================
// Name:        QueueFIFO::push(float)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// NOTE:        This function blocks until space is available
//=========================================================================
void QueueFIFO::push(queue_cmd_t data) 
{
    while (!push_nb(data)){};
}

//=========================================================================
// Name:        QueueFIFO::push_nb(float)
// Brief:       Pushes data onto the queue
// Param[in]:   data = The value to push
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::push_nb(queue_cmd_t data) 
{
    return queue_try_add(&_queue, &data);
}

//=========================================================================
// Name:        QueueFIFO::popf(float)
// Brief:       Pops float data from the queue
// Retval:      The FIFO data
// NOTE:        This function blocks until data is successfully popped
//=========================================================================
queue_cmd_t QueueFIFO::pop() 
{
    queue_cmd_t ret;
    while (!pop_nb(&ret)){};
    return ret;
}

//=========================================================================
// Name:        QueueFIFO::popf_nb(float)
// Brief:       Pops float data from the queue
// Param[out]:  data = The popped value
// Retval:      1 on successfull push
// NOTE:        This function is non-blocking
//=========================================================================
bool QueueFIFO::pop_nb(queue_cmd_t *const data) 
{
    if (queue_get_level(&_queue))
    {
        return queue_try_remove(&_queue, data);
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
    return (queue_get_level(&_queue));
}

//=========================================================================
// Name:        QueueFIFO::available
// Brief:       Check if there is data in the queue
// Retval:      >0 for data available
//=========================================================================
size_t QueueFIFO::size()
{
    return _qsize;
}

//=========================================================================
// End of File
//=========================================================================