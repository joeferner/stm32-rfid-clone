
#include "CuTest.h"
#include "../ring_buffer.h"

#define RING_BUFFER_DATA_SIZE 1000
uint8_t RING_BUFFER_DATA[RING_BUFFER_DATA_SIZE];

void test_ring_buffer_available(CuTest *tc) {
  int i;
  int dataWrite = 0;
  int dataRead = 0;
  uint8_t buffer[10];
  ring_buffer_u8 ringBuffer;
  ring_buffer_u8_init(&ringBuffer, buffer, 10);

  CuAssertIntEquals(tc, 0, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 10, ring_buffer_u8_free(&ringBuffer));

  ring_buffer_u8_write_byte(&ringBuffer, RING_BUFFER_DATA[dataWrite++]);

  CuAssertIntEquals(tc, 1, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 9, ring_buffer_u8_free(&ringBuffer));

  for (i = 0; i < 9; i++) {
    ring_buffer_u8_write_byte(&ringBuffer, RING_BUFFER_DATA[dataWrite++]);
  }

  CuAssertIntEquals(tc, 10, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 0, ring_buffer_u8_free(&ringBuffer));

  ring_buffer_u8_write_byte(&ringBuffer, RING_BUFFER_DATA[dataWrite++]);
  dataRead++; // the last call should have overwritten the first byte in

  CuAssertIntEquals(tc, 10, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 0, ring_buffer_u8_free(&ringBuffer));

  for (i = 0; i < 10; i++) {
    CuAssertIntEquals(tc, RING_BUFFER_DATA[dataRead++], ring_buffer_u8_read_byte(&ringBuffer));
  }

  CuAssertIntEquals(tc, 0, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 10, ring_buffer_u8_free(&ringBuffer));
}

void test_ring_buffer_bulk_read_write(CuTest *tc) {
  int i;
  int dataWrite = 0;
  int dataRead = 0;
  uint8_t data[10];
  uint8_t buffer[10];
  ring_buffer_u8 ringBuffer;
  ring_buffer_u8_init(&ringBuffer, buffer, 10);

  ring_buffer_u8_write(&ringBuffer, RING_BUFFER_DATA + dataWrite, 10);

  CuAssertIntEquals(tc, 10, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 0, ring_buffer_u8_free(&ringBuffer));

  ring_buffer_u8_read(&ringBuffer, data, 10);
  for (i = 0; i < 10; i++) {
    CuAssertIntEquals(tc, RING_BUFFER_DATA[dataRead++], data[i]);
  }

  CuAssertIntEquals(tc, 0, ring_buffer_u8_available(&ringBuffer));
  CuAssertIntEquals(tc, 10, ring_buffer_u8_free(&ringBuffer));
}

CuSuite* ring_buffer_suite() {
  int i;
  for (i = 0; i < RING_BUFFER_DATA_SIZE; i++) {
    RING_BUFFER_DATA[i] = i & 0xff;
  }

  CuSuite* suite = CuSuiteNew();
  SUITE_ADD_TEST(suite, test_ring_buffer_available);
  SUITE_ADD_TEST(suite, test_ring_buffer_bulk_read_write);
  return suite;
}

