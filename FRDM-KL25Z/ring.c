#include <MKL04Z4.h>
#include "common.h"
#include "ring.h"

void buf_reset(RingBuffer *buf, int size)
{
    buf->head = buf->tail = 0;
    buf->size = (uint16_t)size;
}

int buf_len(RingBuffer *buf)
{
    int len = buf->tail - buf->head;
    if (len < 0)
        len += buf->size;
    
    return len;
}

int buf_isfull(RingBuffer *buf)
{
    return buf_len(buf) == (buf->size-1);
}

int buf_isempty(RingBuffer *buf)
{
    return buf->head == buf->tail;
}

uint8_t buf_get_byte(RingBuffer *buf)
{
    uint8_t item;
    
    item = buf->data[buf->head++];
    if (buf->head == buf->size)         // Wrap
        buf->head = 0;
        
    return item;
}

void buf_put_byte(RingBuffer *buf, uint8_t val)
{
    buf->data[buf->tail++] = val;
    if (buf->tail == buf->size)
        buf->tail = 0;
}
