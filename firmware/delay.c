
#include "delay.h"

void delay_ms(uint32_t ms) {
  volatile uint32_t i;
  for(i = ms; i != 0; i--) {
    delay_us(1000);
  }
}

void delay_us(uint32_t us) {
  volatile uint32_t i;
  for(i = (5 * us); i != 0; i--) {}
}
