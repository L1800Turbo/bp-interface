/*
 * circular_buffer.h
 *
 *  Created on: 26.12.2021
 *      Author: kai
 */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stddef.h>

typedef struct circular_buffer
{
    void *buffer;     // data buffer
    void *buffer_end; // end of data buffer
    size_t capacity;  // maximum number of items in the buffer
    size_t count;     // number of items in the buffer
    size_t dt_Size;   // size of each item in the buffer
    void *head;       // pointer to head
    void *tail;       // pointer to tail
} circular_buffer;


typedef enum {
	CB_OK = 0,
	CB_ERR,
	CB_FULL,
	CB_NO_DATA
}circular_buffer_status_en;

void cb_init(circular_buffer *cb, size_t capacity, size_t dt_Size);
circular_buffer_status_en cb_push_back(circular_buffer *cb, const void *item);
circular_buffer_status_en cb_pop_front(circular_buffer *cb, void *item);



#endif /* INC_CIRCULAR_BUFFER_H_ */
