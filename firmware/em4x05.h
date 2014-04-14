#ifndef EM4X05_H
#define	EM4X05_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define EM4X05_ADDR_CHIP_TYPE 0x00
#define EM4X05_ADDR_UID       0x01
#define EM4X05_ADDR_CONFIG    0x04

#define EM4X05_DATA_RATE_8        0x30
#define EM4X05_DATA_RATE_16       0x38
#define EM4X05_DATA_RATE_32       0x3c /* 0b111100 */
#define EM4X05_DATA_RATE_40       0x32
#define EM4X05_DATA_RATE_64       0x3e

#define EM4X05_ENCODER_MANCHESTER 0x08 /* 0b1000 */

#define EM4X05_DELAY_ON_NONE      0x00

#define EM4X05_LWR_6              6

#define EM4X05_READ_LOGIN_OFF     0

#define EM4X05_WRITE_LOGIN_OFF    0

#define EM4X05_ALLOW_DISABLE_OFF  0

#define EM4X05_RTF_OFF            0

#define EM4X05_PIGEON_MODE_OFF    0

#pragma pack(push, 1)

  typedef struct {
    uint32_t dataRate : 6;
    uint32_t encoder : 4;
    uint32_t notUsed0 : 2;
    uint32_t delay : 2;
    uint32_t lastDefaultReadWord : 4;
    uint32_t readLogin : 1;
    uint32_t notUsed1 : 1;
    uint32_t writeLogin : 1;
    uint32_t notUsed2 : 2;
    uint32_t allowDisable : 1;
    uint32_t readerTalkFirst : 1;
    uint32_t notUsed3 : 1;
    uint32_t pigeonMode : 1;
    uint32_t notUsed4 : 5;
  } em4x05_config;

#pragma pack(pop)

  void em4x05_read(uint8_t addr);
  void em4x05_write(uint8_t addr, uint32_t value);
  void em4x05_config_init(em4x05_config* cfg);
  void em4x05_write_config(em4x05_config* cfg);

#ifdef	__cplusplus
}
#endif

#endif	/* EM4X05_H */

