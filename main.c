
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "debug.h"
#include "delay.h"
#include "time.h"
#include "platform_config.h"

void setup();
void loop();
void status_led_setup();
void status_led_on();
void status_led_off();

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  debug_setup();
  status_led_setup();
}

void loop() {
  status_led_on();
  delay_ms(500);
  status_led_off();
  delay_ms(500);
}

void assert_failed(uint8_t* file, uint32_t line) {
  debug_write("assert_failed: file ");
  debug_write((const char*) file);
  debug_write(" on line ");
  debug_write_u32(line, 10);
  debug_write_line("");

  /* Infinite loop */
  while (1) {
  }
}

void status_led_setup() {
  GPIO_InitTypeDef gpioConfig;

  RCC_APB2PeriphClockCmd(STATUS_LED_RCC, ENABLE);
  gpioConfig.GPIO_Pin = STATUS_LED_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(STATUS_LED_PORT, &gpioConfig);
}

void status_led_on() {
  GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN);
}

void status_led_off() {
  GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN);
}
