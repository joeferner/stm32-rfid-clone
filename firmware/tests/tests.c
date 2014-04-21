
#include "CuTest.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

extern CuSuite* em4x05_suite();
extern CuSuite* ring_buffer_suite();

int main(int argc, const char* args[]) {
  CuString *output = CuStringNew();
  CuSuite* suite = CuSuiteNew();

  CuSuiteAddSuite(suite, em4x05_suite());
  CuSuiteAddSuite(suite, ring_buffer_suite());

  CuSuiteRun(suite);
  CuSuiteSummary(suite, output);
  CuSuiteDetails(suite, output);
  printf("%s\n", output->buffer);
}

void debug_write_line(const char* str) {
  printf("%s\n", str);
}

void debug_write(const char* str) {
  printf("%s", str);
}

void debug_write_ch(char ch) {
  printf("%c", ch);
}

void debug_write_u16(uint16_t val, uint8_t base) {
  if (base == 16) {
    printf("%04x", val);
  } else {
    printf("not implemented");
  }
}

void debug_write_u32(uint32_t val, uint8_t base) {
  if (base == 16) {
    printf("%08x", val);
  } else {
    printf("not implemented");
  }
}

void delay_ms(uint32_t ms) {

}

void delay_us(uint32_t us) {
}

void rf_tx_on() {
  //printf("rf_tx_on\n");
}

void rf_tx_off() {
  //printf("rf_tx_off\n");
}

void _em4x05_tx(int i) {
  printf("_em4x05_tx: %d\n", i);
}

void sump_trigger() {

}

void debug_led_set(uint8_t i) {

}