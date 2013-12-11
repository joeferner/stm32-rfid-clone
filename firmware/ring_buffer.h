
#ifndef _ring_buffer_h_
#define _ring_buffer_h_

#include <stdint.h>

typedef struct _ring_buffer_u8 {
  uint8_t* storage;
  uint8_t* end;
  uint16_t size;
  uint8_t* read;
  uint8_t* write;
  uint16_t available;
} ring_buffer_u8;

void ring_buffer_u8_init(ring_buffer_u8* ring, uint8_t* storage, uint16_t size);
void ring_buffer_u8_read(ring_buffer_u8* ring, uint8_t* buffer, uint16_t size);
uint8_t ring_buffer_u8_read_byte(ring_buffer_u8* ring);
void ring_buffer_u8_write(ring_buffer_u8* ring, const uint8_t* buffer, uint16_t size);
void ring_buffer_u8_write_byte(ring_buffer_u8* ring, uint8_t b);
uint16_t ring_buffer_u8_readline(ring_buffer_u8* ring, char* buffer, uint16_t size);
uint8_t ring_buffer_u8_peekn(ring_buffer_u8* ring, uint16_t i);

#endif
