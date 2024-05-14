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

#ifndef __CORE_QUEUE_H
#define __CORE_QUEUE_H

//=============================================================================
// Include Files
//=============================================================================
#include <pico/util/queue.h>
#include <stdint.h>

//=============================================================================
// Class Definitions
//=============================================================================
class QueueFIFO 
{
public:
    // Constructor/destructor
    QueueFIFO(size_t size) : _qsize(size) {}
    ~QueueFIFO() {}

    //=========================================================================
    // Name:        QueueFIFO::begin
    // Brief:       The begin/initialization function for the queue
    //=========================================================================
    void begin();

    //=========================================================================
    // Name:        QueueFIFO::push(float)
    // Brief:       Pushes data onto the queue
    // Param[in]:   data = The value to push
    // NOTE:        This function blocks until space is available
    //=========================================================================
    void push(float data);

    //=========================================================================
    // Name:        QueueFIFO::push_nb(float)
    // Brief:       Pushes data onto the queue
    // Param[in]:   data = The value to push
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool push_nb(float data);

    //=========================================================================
    // Name:        QueueFIFO::push(uint32_t)
    // Brief:       Pushes data onto the queue
    // Param[in]:   data = The value to push
    // NOTE:        This function blocks until space is available
    //=========================================================================
    void push(uint32_t data);

    //=========================================================================
    // Name:        QueueFIFO::push_nb(uint32_t)
    // Brief:       Pushes data onto the queue
    // Param[in]:   data = The value to push
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool push_nb(uint32_t data);

    //=========================================================================
    // Name:        QueueFIFO::popf(float)
    // Brief:       Pops float data from the queue
    // Retval:      The FIFO data
    // NOTE:        This function blocks until data is successfully popped
    //=========================================================================
    float popf();

    //=========================================================================
    // Name:        QueueFIFO::popf_nb(float)
    // Brief:       Pops float data from the queue
    // Param[out]:  data = The popped value
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool popf_nb(float *const data);

    //=========================================================================
    // Name:        QueueFIFO::pop32(uint32)
    // Brief:       Pops uint32 data from the queue
    // Retval:      The FIFO data
    // NOTE:        This function blocks until data is successfully popped
    //=========================================================================
    uint32_t pop32();

    //=========================================================================
    // Name:        QueueFIFO::pop32_nb(uint32_t)
    // Brief:       Pops float data from the queue
    // Param[out]:  data = The popped value
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool pop32_nb(uint32_t *const data);

    //=========================================================================
    // Name:        QueueFIFO::available
    // Brief:       Check if there is data in the queue
    // Retval:      >0 for data available
    // NOTE:        This function blocks until data is successfully popped
    //=========================================================================
    int available();

private:
    queue_t     _queue32;       // The queue to use for uint32 data
    queue_t     _queuef;        // The queue to use for float data
    size_t      _qsize;         // The size of the queue
};


#endif // __CORE_QUEUE_H