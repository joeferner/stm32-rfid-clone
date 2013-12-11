
#ifndef DEBUG_H
#define	DEBUG_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

  void debug_setup();
  void debug_led_set(int v);
  void debug_write_line(const char* str);
  void debug_write_bytes(const uint8_t *data, uint16_t len);
  void debug_write(const char* str);
  void debug_write_ch(char ch);
  void debug_write_u8(uint8_t val, uint8_t base);
  void debug_write_u8_array(uint8_t *p, int len);
  void debug_write_u16(uint16_t val, uint8_t base);
  void debug_write_u32(uint32_t val, uint8_t base);
  void debug_write_i32(int32_t val, uint8_t base);
  extern void debug_on_rx(uint8_t* data, uint16_t len);

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

