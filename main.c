
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

#define RX_TX_CARRIER_FREQ 125000
#define RF_TX_PWM_PERIOD   (SystemCoreClock / RX_TX_CARRIER_FREQ)

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

#define CAPTURE_BUFFER_LEN 500
uint32_t lastRfRxTime = 0;
uint16_t capture1, capture2;
uint16_t lastCapture;
uint16_t capture1Diff, capture2Diff;
uint16_t captureBuffer[CAPTURE_BUFFER_LEN];
uint16_t captureBufferIndex = 0;

typedef struct {
  uint32_t start;
  uint32_t duration;
  uint32_t bit;
} report_t;

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

  //usb_setup();

  //delay_ms(1000); // !!!! IMPORTANT: Keep this line in here. If we have a JTAG issue we need this time to get in before JTAG is disabled.
  //disable_jtag();

  time_setup();

  rf_tx_setup();
  rf_rx_setup();

  rf_tx_on();

  debug_led_set(0);

  debug_write_line("?END setup");
}

void loop() {
  delay_ms(100);
  if (captureBufferIndex == CAPTURE_BUFFER_LEN) {
    for (int i = 0; i < CAPTURE_BUFFER_LEN; i++) {
      debug_write_i32(captureBuffer[i], 10);
      debug_write_line("");
    }
    captureBufferIndex++;
  }

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

  // Time base configuration
  TIM_TimeBaseStructInit(&timeBaseInit);
  timeBaseInit.TIM_Period = RF_TX_PWM_PERIOD;
  timeBaseInit.TIM_Prescaler = 0; //(uint16_t) (SystemCoreClock / 30000000) - 1;
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
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, RF_TX_PWM_PERIOD / 2);
}

void rf_tx_off() {
  RF_TX_TIMER_CH_SetCompare(RF_TX_TIMER, 0);
}

void rf_rx_setup() {
  GPIO_InitTypeDef gpioConfig;
  NVIC_InitTypeDef nvicInit;
  TIM_ICInitTypeDef timerInputCaptureInit;

  debug_write_line("?BEGIN rf_rx_setup");

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  RCC_APB2PeriphClockCmd(RF_RX_RCC, ENABLE);
  gpioConfig.GPIO_Pin = RF_RX_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RF_RX_PORT, &gpioConfig);

  nvicInit.NVIC_IRQChannel = TIM1_CC_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInit.NVIC_IRQChannelSubPriority = 1;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  TIM_ICStructInit(&timerInputCaptureInit);
  timerInputCaptureInit.TIM_Channel = TIM_Channel_1;
  timerInputCaptureInit.TIM_ICPolarity = TIM_ICPolarity_Rising;
  timerInputCaptureInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
  timerInputCaptureInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  timerInputCaptureInit.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &timerInputCaptureInit);

  TIM_Cmd(TIM1, ENABLE);

  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);

  debug_write_line("?END rf_rx_setup");
}

void on_time1_cc_irq() {
  if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    capture1 = TIM_GetCapture1(TIM1);
    capture1Diff = capture1 - lastCapture;
    lastCapture = capture1;

    if (captureBufferIndex < CAPTURE_BUFFER_LEN) {
      captureBuffer[captureBufferIndex++] = capture1Diff;
    }
  }
}

void usb_on_rx(uint8_t* data, uint16_t len) {
#define MAX_LINE_LENGTH 100
  char line[MAX_LINE_LENGTH];

  ring_buffer_u8_write(&usb_input_ring_buffer, data, len);
  while (ring_buffer_u8_readline(&usb_input_ring_buffer, line, MAX_LINE_LENGTH) > 0) {
    debug_write_line(line);
  }
}
