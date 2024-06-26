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
// Defines
//=============================================================================

//=============================================================================
// Structs and Types
//=============================================================================
typedef struct queue_cmd
{
    uint32_t    cmd;
    float       data[8];
} queue_cmd_t;

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
    void push(queue_cmd_t data);

    //=========================================================================
    // Name:        QueueFIFO::push_nb(float)
    // Brief:       Pushes data onto the queue
    // Param[in]:   data = The value to push
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool push_nb(queue_cmd_t data);

    //=========================================================================
    // Name:        QueueFIFO::popf(float)
    // Brief:       Pops float data from the queue
    // Retval:      The FIFO data
    // NOTE:        This function blocks until data is successfully popped
    //=========================================================================
    queue_cmd_t pop();

    //=========================================================================
    // Name:        QueueFIFO::popf_nb(float)
    // Brief:       Pops float data from the queue
    // Param[out]:  data = The popped value
    // Retval:      1 on successfull push
    // NOTE:        This function is non-blocking
    //=========================================================================
    bool pop_nb(queue_cmd_t *const data);

    //=========================================================================
    // Name:        QueueFIFO::available
    // Brief:       Check if there is data in the queue
    // Retval:      >0 for data available
    //=========================================================================
    int available();

    //=========================================================================
    // Name:        QueueFIFO::available
    // Brief:       Check if there is data in the queue
    // Retval:      >0 for data available
    //=========================================================================
    size_t size();

private:
    queue_t         _queue;        // The queue to use for float data
    size_t          _qsize;         // The size of the queue
};


#endif // __CORE_QUEUE_H

//=============================================================================
// End of file
//=============================================================================