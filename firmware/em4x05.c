
#include "em4x05.h"
#include "delay.h"
#include "debug.h"

#define US_PER_RF_TICK 8

#define COMMAND_READ  0x9
#define COMMAND_WRITE 0x5

extern void rf_tx_on();
extern void rf_tx_off();

void _em4x05_tx_first_stop();
void _em4x05_tx(int i);
void _em4x05_tx_command(uint8_t command);
void _em4x05_tx_addr(uint8_t addr);
void _em4x05_delay(int rfTicks);
uint32_t _em4x05_config_to_uint32(em4x05_config* cfg);
void _em4x05_write_uint32_calc(uint32_t data, uint8_t* bits);

void em4x05_read(uint8_t addr) {
  debug_write("em4x05_read: 0x");
  debug_write_u16(addr, 16);
  debug_write_line("");

  _em4x05_tx_first_stop();
  _em4x05_tx_command(COMMAND_READ);
  _em4x05_tx_addr(addr);
}

void _em4x05_tx_command(uint8_t command) {
  _em4x05_tx((command >> 3) & 0x1);
  _em4x05_tx((command >> 2) & 0x1);
  _em4x05_tx((command >> 1) & 0x1);
  _em4x05_tx((command >> 0) & 0x1);
}

void _em4x05_tx_addr(uint8_t addr) {
  _em4x05_tx((addr >> 3) & 0x1);
  _em4x05_tx((addr >> 2) & 0x1);
  _em4x05_tx((addr >> 1) & 0x1);
  _em4x05_tx((addr >> 0) & 0x1);
  _em4x05_tx(0);
  _em4x05_tx(0);

  int parity =
          ((addr & 0x8) ? 1 : 0)
          ^ ((addr & 0x4) ? 1 : 0)
          ^ ((addr & 0x2) ? 1 : 0)
          ^ ((addr & 0x1) ? 1 : 0);
  _em4x05_tx(parity);
}

void em4x05_config_init(em4x05_config* cfg) {
  uint32_t* val = (uint32_t*) cfg;
  *val = 0;

  cfg->dataRate = EM4X05_DATA_RATE_32;
  cfg->encoder = EM4X05_ENCODER_MANCHESTER;
  cfg->delay = EM4X05_DELAY_ON_NONE;
  cfg->lastDefaultReadWord = EM4X05_LWR_6;
  cfg->readLogin = EM4X05_READ_LOGIN_OFF;
  cfg->writeLogin = EM4X05_WRITE_LOGIN_OFF;
  cfg->allowDisable = EM4X05_ALLOW_DISABLE_OFF;
  cfg->readerTalkFirst = EM4X05_RTF_OFF;
  cfg->pigeonMode = EM4X05_PIGEON_MODE_OFF;
}

void em4x05_write_config(em4x05_config* cfg) {
  uint32_t bits = _em4x05_config_to_uint32(cfg);
  em4x05_write(EM4X05_ADDR_CONFIG, bits);
}

uint32_t _em4x05_config_to_uint32(em4x05_config* cfg) {
  uint32_t bits = 0;

  bits |= ((uint32_t) cfg->dataRate & 0x3f) << 26;
  bits |= ((uint32_t) cfg->encoder & 0x0f) << 22;
  bits |= (0x00 & 0x03) << 20;
  bits |= ((uint32_t) cfg->delay & 0x03) << 18;
  bits |= ((uint32_t) cfg->lastDefaultReadWord & 0x0f) << 14;
  bits |= ((uint32_t) (cfg->readLogin ? 1 : 0) & 0x01) << 13;
  bits |= (0x00 & 0x01) << 12;
  bits |= ((uint32_t) (cfg->writeLogin ? 1 : 0) & 0x01) << 11;
  bits |= (0x00 & 0x03) << 9;
  bits |= ((uint32_t) (cfg->allowDisable ? 1 : 0) & 0x01) << 8;
  bits |= ((uint32_t) (cfg->readerTalkFirst ? 1 : 0) & 0x01) << 7;
  bits |= (0x00 & 0x01) << 6;
  bits |= ((uint32_t) (cfg->pigeonMode ? 1 : 0) & 0x01) << 5;
  bits |= (0x00 & 0x1f) << 0;

  return bits;
}

void em4x05_write(uint8_t addr, uint32_t value) {
  debug_write("BEGIN em4x05_write: 0x");
  debug_write_u16(addr, 16);
  debug_write(", 0x");
  debug_write_u32(value, 16);
  debug_write_line("");

  uint8_t bits[45];
  int i;
  _em4x05_write_uint32_calc(value, bits);

  _em4x05_tx_first_stop();
  _em4x05_tx_command(COMMAND_WRITE);
  _em4x05_tx_addr(addr);
  for (i = 0; i < 45; i++) {
    _em4x05_tx(bits[i]);
  }
  delay_ms(100);

  debug_write_line("END em4x05_write");
}

void _em4x05_write_uint32_calc(uint32_t data, uint8_t* bits) {
  // row 0
  bits[0] = (data >> 31) & 0x01;
  bits[1] = (data >> 30) & 0x01;
  bits[2] = (data >> 29) & 0x01;
  bits[3] = (data >> 28) & 0x01;
  bits[4] = (data >> 27) & 0x01;
  bits[5] = (data >> 26) & 0x01;
  bits[6] = (data >> 25) & 0x01;
  bits[7] = (data >> 24) & 0x01;
  bits[8] = bits[0] ^ bits[1] ^ bits[2] ^ bits[3] ^ bits[4] ^ bits[5] ^ bits[6] ^ bits[7];

  // row 1
  bits[9] = (data >> 23) & 0x01;
  bits[10] = (data >> 22) & 0x01;
  bits[11] = (data >> 21) & 0x01;
  bits[12] = (data >> 20) & 0x01;
  bits[13] = (data >> 19) & 0x01;
  bits[14] = (data >> 18) & 0x01;
  bits[15] = (data >> 17) & 0x01;
  bits[16] = (data >> 16) & 0x01;
  bits[17] = bits[9] ^ bits[10] ^ bits[11] ^ bits[12] ^ bits[13] ^ bits[14] ^ bits[15] ^ bits[16];

  // row 2
  bits[18] = (data >> 15) & 0x01;
  bits[19] = (data >> 14) & 0x01;
  bits[20] = (data >> 13) & 0x01;
  bits[21] = (data >> 12) & 0x01;
  bits[22] = (data >> 11) & 0x01;
  bits[23] = (data >> 10) & 0x01;
  bits[24] = (data >> 9) & 0x01;
  bits[25] = (data >> 8) & 0x01;
  bits[26] = bits[18] ^ bits[19] ^ bits[20] ^ bits[21] ^ bits[22] ^ bits[23] ^ bits[24] ^ bits[25];

  // row 3
  bits[27] = (data >> 7) & 0x01;
  bits[28] = (data >> 6) & 0x01;
  bits[29] = (data >> 5) & 0x01;
  bits[30] = (data >> 4) & 0x01;
  bits[31] = (data >> 3) & 0x01;
  bits[32] = (data >> 2) & 0x01;
  bits[33] = (data >> 1) & 0x01;
  bits[34] = (data >> 0) & 0x01;
  bits[35] = bits[27] ^ bits[28] ^ bits[29] ^ bits[30] ^ bits[31] ^ bits[32] ^ bits[33] ^ bits[34];

  // row 4
  bits[36] = bits[0] ^ bits[9] ^ bits[18] ^ bits[27];
  bits[37] = bits[1] ^ bits[10] ^ bits[19] ^ bits[28];
  bits[38] = bits[2] ^ bits[11] ^ bits[20] ^ bits[29];
  bits[39] = bits[3] ^ bits[12] ^ bits[21] ^ bits[30];
  bits[40] = bits[4] ^ bits[13] ^ bits[22] ^ bits[31];
  bits[41] = bits[5] ^ bits[14] ^ bits[23] ^ bits[32];
  bits[42] = bits[6] ^ bits[15] ^ bits[24] ^ bits[33];
  bits[43] = bits[7] ^ bits[16] ^ bits[25] ^ bits[34];
  bits[44] = 0;
}

void _em4x05_tx_first_stop() {
  rf_tx_off();
  _em4x05_delay(55);
  rf_tx_on();
}

#ifndef TEST

void _em4x05_tx(int i) {
  if (i) {
    rf_tx_on();
    _em4x05_delay(32);
  } else {
    rf_tx_on();
    _em4x05_delay(18);
    rf_tx_off();
    _em4x05_delay(14);
    rf_tx_on();
  }
}
#endif

void _em4x05_delay(int rfTicks) {
  delay_us(US_PER_RF_TICK * rfTicks);
}