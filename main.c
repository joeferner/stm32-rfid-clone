
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>
#include "debug.h"
#include "delay.h"
#include "time.h"
#include "usb.h"
#include "ring_buffer.h"
#include "platform_config.h"

#define PWM_PERIOD 285

void setup();
void loop();
void rf_tx_setup();
void rf_tx_on();
void rf_tx_off();
void rf_rx_setup();
void disable_jtag();

#define INPUT_BUFFER_SIZE 100
uint8_t usb_input_buffer[INPUT_BUFFER_SIZE];
ring_buffer_u8 usb_input_ring_buffer;

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  // Configure the NVIC Preemption Priority Bits
  // 2 bit for pre-emption priority, 2 bits for subpriority
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  ring_buffer_u8_init(&usb_input_ring_buffer, usb_input_buffer, INPUT_BUFFER_SIZE);

  debug_setup();

  usb_setup();

  //delay_ms(1000); // !!!! IMPORTANT: Keep this line in here. If we have a JTAG issue we need this time to get in before JTAG is disabled.
  //disable_jtag();

  time_setup();

  rf_tx_setup();
  rf_rx_setup();

  rf_tx_on();

  debug_led_set(0);

  debug_write_line("?END setup");
}

typedef struct {
  uint32_t time;
} report_t;

void loop() {
  report_t r;

  delay_ms(1000);
  r.time = time_ms();
  usb_write((const uint8_t*) &r, sizeof (report_t));

  //  delay_ms(500);
  //  status_led_off();
  //  rf_tx_off();
  //  delay_ms(500);
}

void assert_failed(uint8_t* file, uint32_t line) {
  debug_write("-assert_failed: file ");
  debug_write((const char*) file);
  debug_write(" on line ");
  debug_write_u32(line, 10);
  debug_write_line("");

  /* Infinite loop */
  while (1) {
  }
}

void disable_jtag() {
  debug_write_line("?disable_jtag");
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

void rf_tx_setup() {
  GPIO_InitTypeDef GPIO_Config;
  uint16_t prescalerValue = 0;
  TIM_TimeBaseInitTypeDef timeBaseInit;
  TIM_OCInitTypeDef ocInit;

  debug_write_line("?BEGIN rf_tx_setup");

  RCC_APB1PeriphClockCmd(RF_TX_TIMER_RCC, ENABLE);

  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

  RCC_APB2PeriphClockCmd(RF_TX_RCC, ENABLE);
  GPIO_Config.GPIO_Pin = RF_TX_PIN;
  GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RF_TX_PORT, &GPIO_Config);

  /*
   * TIM2 Configuration: generate 2 PWM signals with 2 different duty cycles:
   * The TIM2CLK frequency is set to SystemCoreClock (Hz), to get TIM2 counter
   * clock at 24 MHz the Pre-scaler is computed as following:
   *  - Pre-scaler = (TIM2CLK / TIM2 counter clock) - 1
   * SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   * and Connectivity line devices and to 24 MHz for Low-Density Value line and
   * Medium-Density Value line devices
   *
   * The TIM2 is running at 36 KHz: TIM2 Frequency = TIM2 counter clock/(ARR + 1) = 24 MHz / 666 = 36 KHz
   *     TIM2 Channel1 duty cycle = (TIM2_CCR1 / TIM2_ARR) * 100 = 50%
   *     TIM2 Channel2 duty cycle = (TIM2_CCR2 / TIM2_ARR) * 100 = 37.5%
   */

  // Compute the pre-scaler value
  prescalerValue = (uint16_t) (SystemCoreClock / 30000000) - 1;

  // Time base configuration
  TIM_TimeBaseStructInit(&timeBaseInit);
  timeBaseInit.TIM_Period = PWM_PERIOD;
  timeBaseInit.TIM_Prescaler = prescalerValue;
  timeBaseInit.TIM_ClockDivision = 0;
  timeBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(RF_TX_TIMER, &timeBaseInit);

  TIM_OCStructInit(&ocInit);
  ocInit.TIM_OCMode = TIM_OCMode_PWM1;
  ocInit.TIM_OCPolarity = TIM_OCPolarity_High;
  ocInit.TIM_OutputState = TIM_OutputState_Enable;
  ocInit.TIM_Pulse = 0;
  RF_TX_TIMER_CH_Init(RF_TX_TIMER, &ocInit);
  RF_TX_TIMER_CH_PreloadConfig(RF_TX_TIMER, TIM_OCPreload_Enable);

  TIM_SelectOnePulseMode(RF_TX_TIMER, TIM_OPMode_Repetitive);
  TIM_ARRPreloadConfig(RF_TX_TIMER, ENABLE);
  TIM_Cmd(RF_TX_TIMER, ENABLE);

  debug_write_line("?END rf_tx_setup");
}

void rf_tx_on() {
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, PWM_PERIOD / 2);
}

void rf_tx_off() {
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, 0);
}

void rf_rx_setup() {
  GPIO_InitTypeDef gpioConfig;

  debug_write_line("?BEGIN rf_rx_setup");

  RCC_APB2PeriphClockCmd(RF_RX_RCC, ENABLE);
  gpioConfig.GPIO_Pin = RF_RX_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RF_RX_PORT, &gpioConfig);

  GPIO_EXTILineConfig(RF_RX_EXTI_PORT, RF_RX_EXTI_PIN);

  EXTI_InitTypeDef extiConfig;
  extiConfig.EXTI_Line = RF_RX_EXTI_LINE;
  extiConfig.EXTI_Mode = EXTI_Mode_Interrupt;
  extiConfig.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiConfig.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiConfig);

  NVIC_InitTypeDef nvicConfig;
  nvicConfig.NVIC_IRQChannel = RF_RX_EXTI_IRQ;
  nvicConfig.NVIC_IRQChannelPreemptionPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelSubPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicConfig);

  debug_write_line("?END rf_rx_setup");
}

uint32_t setTime = 0;

void on_exti9_5_irq() {
  if (EXTI_GetITStatus(RF_RX_EXTI_LINE) == SET) {
    if (GPIO_ReadInputDataBit(RF_RX_PORT, RF_RX_PIN)) {
      debug_led_set(1);
      setTime = time_us();
    } else {
      debug_led_set(0);
      uint32_t z = time_us() - setTime;
      debug_write_u32(z, 10);
      debug_write_line("");
    }
  }
  EXTI_ClearITPendingBit(RF_RX_EXTI_LINE);
}

void usb_on_rx(uint8_t* data, uint16_t len) {
#define MAX_LINE_LENGTH 100
  char line[MAX_LINE_LENGTH];

  ring_buffer_u8_write(&usb_input_ring_buffer, data, len);
  while (ring_buffer_u8_readline(&usb_input_ring_buffer, line, MAX_LINE_LENGTH) > 0) {
    debug_write_line(line);
  }
}
