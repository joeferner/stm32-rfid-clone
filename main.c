
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>
#include "debug.h"
#include "delay.h"
#include "time.h"
#include "platform_config.h"

#define PWM_PERIOD 0x2000

void setup();
void loop();
void status_led_setup();
void status_led_on();
void status_led_off();
void rf_tx_setup();
void rf_tx_on();
void rf_tx_off();
void rf_rx_setup();

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

  debug_setup();
  status_led_setup();
  rf_tx_setup();
  rf_rx_setup();

  debug_write_line("?END setup");
}

void loop() {
  status_led_on();
  rf_tx_on();
  delay_ms(500);
  status_led_off();
  rf_tx_off();
  delay_ms(500);
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

void status_led_setup() {
  GPIO_InitTypeDef gpioConfig;

  debug_write_line("?BEGIN status_led_setup");

  RCC_APB2PeriphClockCmd(STATUS_LED_RCC, ENABLE);
  gpioConfig.GPIO_Pin = STATUS_LED_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(STATUS_LED_PORT, &gpioConfig);

  debug_write_line("?END status_led_setup");
}

void status_led_on() {
  GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN);
}

void status_led_off() {
  GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN);
}

void rf_tx_setup() {
  GPIO_InitTypeDef GPIO_Config;
  uint16_t prescalerValue = 0;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  debug_write_line("?BEGIN rf_tx_setup");

  RCC_APB1PeriphClockCmd(RF_TX_TIMER_RCC, ENABLE);

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
  prescalerValue = (uint16_t) (SystemCoreClock / 3000000) - 1;

  // Time base configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(RF_TX_TIMER, &TIM_TimeBaseStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  // PWM1 Mode configuration: Channel4
  TIM_OCInitStructure.TIM_Pulse = 0;
  RF_TX_TIMER_CH_Init(RF_TX_TIMER, &TIM_OCInitStructure);
  RF_TX_TIMER_CH_PreloadConfig(RF_TX_TIMER, TIM_OCPreload_Enable);

  TIM_SelectOnePulseMode(RF_TX_TIMER, TIM_OPMode_Repetitive);
  TIM_ARRPreloadConfig(RF_TX_TIMER, ENABLE);
  TIM_Cmd(RF_TX_TIMER, ENABLE);

  debug_write_line("?END rf_tx_setup");
}

void rf_tx_on() {
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, 100);
}

void rf_tx_off() {
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, 0);
}

void rf_rx_setup() {
  GPIO_InitTypeDef gpioConfig;

  debug_write_line("?BEGIN rf_rx_setup");

  RCC_APB2PeriphClockCmd(RF_RX_RCC, ENABLE);
  gpioConfig.GPIO_Pin = RF_RX_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RF_RX_PORT, &gpioConfig);

  debug_write_line("?END rf_rx_setup");
}
