
#include "platform_config.h"
#include "debug.h"
#include <stm32f10x_usart.h>
#include <stm32f10x_misc.h>

#define TO_HEX(i) ( (((i) & 0xf) <= 9) ? ('0' + ((i) & 0xf)) : ('A' - 10 + ((i) & 0xf)) )

char* itoa(int32_t value, char* result, int base);
char* uitoa(uint32_t value, char* result, int base);

void debug_setup() {
  USART_InitTypeDef usartInitStructure;
  GPIO_InitTypeDef gpioInitStructure;
  NVIC_InitTypeDef nvicInitStructure;

  RCC_APB2PeriphClockCmd(DEBUG_LED_RCC, ENABLE);
  gpioInitStructure.GPIO_Pin = DEBUG_LED_PIN;
  gpioInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(DEBUG_LED_PORT, &gpioInitStructure);

  /* Enable the USART1 Interrupt */
  nvicInitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  nvicInitStructure.NVIC_IRQChannelSubPriority = 3;
  nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInitStructure);

  usartInitStructure.USART_BaudRate = DEBUG_USART_BAUD;
  usartInitStructure.USART_WordLength = USART_WordLength_8b;
  usartInitStructure.USART_Parity = USART_Parity_No;
  usartInitStructure.USART_StopBits = USART_StopBits_1;
  usartInitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usartInitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Enable clocks */
  RCC_APB2PeriphClockCmd(DEBUG_USART_RCC, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  gpioInitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  gpioInitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
  gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(DEBUG_USART_TX, &gpioInitStructure);

  /* Configure USART Rx as input floating */
  gpioInitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioInitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX, &gpioInitStructure);

  /* USART configuration */
  USART_Init(DEBUG_USART, &usartInitStructure);

  /* Enable USART */
  USART_Cmd(DEBUG_USART, ENABLE);

  /* Enable the USART interrupts */
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(DEBUG_USART, USART_IT_TXE, DISABLE);
}

void debug_led_set(int v) {
  if (v) {
    GPIO_SetBits(DEBUG_LED_PORT, DEBUG_LED_PIN);
  } else {
    GPIO_ResetBits(DEBUG_LED_PORT, DEBUG_LED_PIN);
  }
}

void debug_write_line(const char* str) {
  debug_write(str);
  debug_write_ch('\n');
}

void debug_write_bytes(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    debug_write_ch((char) data[i]);
  }
}

void debug_write(const char* str) {
  const char *p = str;
  while (*p) {
    debug_write_ch(*p);
    p++;
  }
}

void debug_write_ch(char ch) {
  USART_SendData(DEBUG_USART, ch);
  while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

void debug_write_u8(uint8_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_u16(uint16_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 12));
    debug_write_ch(TO_HEX(val >> 8));
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_u32(uint32_t val, uint8_t base) {
  if (base == 16) {
    debug_write_ch(TO_HEX(val >> 28));
    debug_write_ch(TO_HEX(val >> 24));
    debug_write_ch(TO_HEX(val >> 20));
    debug_write_ch(TO_HEX(val >> 16));
    debug_write_ch(TO_HEX(val >> 12));
    debug_write_ch(TO_HEX(val >> 8));
    debug_write_ch(TO_HEX(val >> 4));
    debug_write_ch(TO_HEX(val >> 0));
  } else {
    char buffer[20];
    uitoa(val, buffer, base);
    debug_write(buffer);
  }
}

void debug_write_i32(int32_t val, uint8_t base) {
  char buffer[20];
  itoa(val, buffer, base);
  debug_write(buffer);
}

void debug_write_u8_array(uint8_t *p, int len) {
  for (int i = 0; i < len; i++) {
    debug_write_u8(p[i], 16);
    debug_write_ch(' ');
  }
}

char* itoa(int32_t value, char* result, int base) {
  // check that the base if valid
  if (base < 2 || base > 36) {
    *result = '\0';
    return result;
  }

  char* ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do {
    tmp_value = value;
    value /= base;
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
  } while (value);

  // Apply negative sign
  if (tmp_value < 0) {
    *ptr++ = '-';
  }
  *ptr-- = '\0';
  while (ptr1 < ptr) {
    tmp_char = *ptr;
    *ptr-- = *ptr1;
    *ptr1++ = tmp_char;
  }
  return result;
}

char* uitoa(uint32_t value, char* result, int base) {
  // check that the base if valid
  if (base < 2 || base > 36) {
    *result = '\0';
    return result;
  }

  char* ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do {
    tmp_value = value;
    value /= base;
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
  } while (value);

  *ptr-- = '\0';
  while (ptr1 < ptr) {
    tmp_char = *ptr;
    *ptr-- = *ptr1;
    *ptr1++ = tmp_char;
  }
  return result;
}
