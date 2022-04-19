/*
 * circular_buffer.c
 *
 *  Created on: 26.12.2021
 *      Author: kai
 */

// von: https://www.it-swarm.com.de/de/c/wie-implementiert-man-einen-ringpuffer-c/957988995/
#include "circular_buffer.h"
#include <string.h>
#include <stdlib.h>

void cb_init(circular_buffer *cb, size_t capacity, size_t dt_Size)
{
    cb->buffer = malloc(capacity * dt_Size);

    //if(cb->buffer == NULL)
        // handle error

    cb->buffer_end = (char *)cb->buffer + capacity * dt_Size;
    cb->capacity = capacity;
    cb->count = 0;
    cb->dt_Size = dt_Size;
    cb->head = cb->buffer;
    cb->tail = cb->buffer;
}

void cb_free(circular_buffer *cb)
{
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

circular_buffer_status_en cb_push_back(circular_buffer *cb, const void *item)
{
    if(cb->count == cb->capacity)
    {
        return CB_FULL;
    }

    memcpy(cb->head, item, cb->dt_Size);

    cb->head = (char*)cb->head + cb->dt_Size;

    if(cb->head == cb->buffer_end)
    {
        cb->head = cb->buffer;
    }

    cb->count++;

    return CB_OK;
}

/* Get the first buffer item without taking it from the buffer */
circular_buffer_status_en cb_get_front(circular_buffer *cb, void *item)
{
    if(cb->count == 0)
    {
        return CB_NO_DATA;
    }

    memcpy(item, cb->tail, cb->dt_Size);

    return CB_OK;
}

/* Clear the first item from the buffer */
circular_buffer_status_en cb_clear_front(circular_buffer *cb)
{
    if(cb->count == 0)
    {
        return CB_NO_DATA;
    }

    cb->tail = (char*)cb->tail + cb->dt_Size;

    // If end is reached
    if(cb->tail == cb->buffer_end)
    {
        cb->tail = cb->buffer;
    }

    cb->count--;

    return CB_OK;
}


// TODO: noch alte funktion, die oberen sollen das dann abdecken, kann man hier dann einbauen...
circular_buffer_status_en cb_pop_front(circular_buffer *cb, void *item)
{
    if(cb->count == 0)
    {
        return CB_NO_DATA;
    }

    memcpy(item, cb->tail, cb->dt_Size);

    cb->tail = (char*)cb->tail + cb->dt_Size;

    // If end is reached
    if(cb->tail == cb->buffer_end)
    {
        cb->tail = cb->buffer;
    }

    cb->count--;

    return CB_OK;
}

