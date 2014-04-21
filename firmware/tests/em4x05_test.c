
#include "CuTest.h"
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
