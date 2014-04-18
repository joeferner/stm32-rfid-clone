
#include "CuTest.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../em4x05.h"

extern uint32_t _em4x05_config_to_uint32(em4x05_config* cfg);
extern void _em4x05_write_uint32_calc(uint32_t data, uint8_t* bits);

void test_em4x05(CuTest *tc) {
  int i;
  em4x05_config cfg;
  em4x05_config_init(&cfg);
  cfg.dataRate = EM4X05_DATA_RATE_32;
  cfg.encoder = EM4X05_ENCODER_MANCHESTER;
  cfg.delay = EM4X05_DELAY_ON_NONE;
  cfg.lastDefaultReadWord = EM4X05_LWR_6;
  cfg.readLogin = EM4X05_READ_LOGIN_OFF;
  cfg.writeLogin = EM4X05_WRITE_LOGIN_OFF;
  cfg.allowDisable = EM4X05_ALLOW_DISABLE_OFF;
  cfg.readerTalkFirst = EM4X05_RTF_OFF;
  cfg.pigeonMode = EM4X05_PIGEON_MODE_OFF;
  uint32_t found = _em4x05_config_to_uint32(&cfg);
  if (0xf2018000 != found) {
    CuAssertIntEquals(tc, 0xf2018001, found);
  }

  uint8_t bits[45];
  uint8_t expectedBits[45] = {
    1, 1, 1, 1, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 1, 0, 0, 1, 1, 0
  };
  _em4x05_write_uint32_calc(found, bits);
  for (i = 0; i < 45; i++) {
    if (bits[i] != expectedBits[i]) {
      CuAssertIntEquals(tc, expectedBits[i], bits[i]);
    }
  }

  em4x05_write_config(&cfg);
}

CuSuite* em4x05_suite() {
  CuSuite* suite = CuSuiteNew();
  SUITE_ADD_TEST(suite, test_em4x05);
  return suite;
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
