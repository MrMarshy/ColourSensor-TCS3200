#ifndef RING_H
#define RING_H

#include <stdint.h>

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t size;
    volatile uint8_t data[];
} RingBuffer;

void buf_reset(RingBuffer *buf, int size);
int buf_len(RingBuffer *buf);
int buf_isfull(RingBuffer *buf);
int buf_isempty(RingBuffer *buf);
uint8_t buf_get_byte(RingBuffer *buf);
void buf_put_byte(RingBuffer *buf, uint8_t val);

#endif // RING_H
