
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
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
void rf_rx_enable();
void rf_rx_disable();
void disable_jtag();
void rf_rx_process_capture_buffer();

#define INPUT_BUFFER_SIZE 100
uint8_t usb_input_buffer[INPUT_BUFFER_SIZE];
ring_buffer_u8 usb_input_ring_buffer;

#define RF_RX_CAPTURE_BUFFER_LEN 1000
uint16_t rfRxCaptureBuffer[RF_RX_CAPTURE_BUFFER_LEN];
uint rfRxProcessOffset;
uint rfRxDmaLastCNDTR;
volatile uint rfRxDmaToBeProcessedCount;

typedef struct {
  uint32_t start;
  uint32_t duration;
  uint32_t bit;
} report_t;

int main(void) {
  setup();

  rf_rx_enable();

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

  rfRxProcessOffset = 0;

  debug_write_line("?END setup");
}

void loop() {
  rf_rx_process_capture_buffer();

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
  TIM_ICInitTypeDef timerInputCaptureInit;
  DMA_InitTypeDef dmaInit;
  NVIC_InitTypeDef nvicInit;

  debug_write_line("?BEGIN rf_rx_setup");

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  RCC_APB2PeriphClockCmd(RF_RX_RCC, ENABLE);
  gpioConfig.GPIO_Pin = RF_RX_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RF_RX_PORT, &gpioConfig);

  nvicInit.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInit.NVIC_IRQChannelSubPriority = 1;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  DMA_DeInit(DMA1_Channel2);
  DMA_StructInit(&dmaInit);
  dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & TIM1->CCR1;
  dmaInit.DMA_MemoryBaseAddr = (uint32_t) & rfRxCaptureBuffer;
  dmaInit.DMA_DIR = DMA_DIR_PeripheralSRC;
  dmaInit.DMA_BufferSize = RF_RX_CAPTURE_BUFFER_LEN;
  dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_Mode = DMA_Mode_Circular;
  dmaInit.DMA_Priority = DMA_Priority_High;
  dmaInit.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &dmaInit);

  DMA_ITConfig(DMA1_Channel2, DMA_IT_HT | DMA_IT_TC, ENABLE);

  TIM_ICStructInit(&timerInputCaptureInit);
  timerInputCaptureInit.TIM_Channel = TIM_Channel_1;
  timerInputCaptureInit.TIM_ICPolarity = TIM_ICPolarity_Rising;
  timerInputCaptureInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
  timerInputCaptureInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  timerInputCaptureInit.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &timerInputCaptureInit);

  TIM_SelectCCDMA(TIM1, ENABLE);

  TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);

  debug_write_line("?END rf_rx_setup");
}

void on_dma1_ch2_irq() {
  if (DMA_GetITStatus(DMA1_IT_TC2) || DMA_GetITStatus(DMA1_IT_HT2)) {
    DMA_ClearITPendingBit(DMA1_IT_GL2);
    int count = rfRxDmaLastCNDTR - DMA1_Channel2->CNDTR;
    rfRxDmaLastCNDTR = DMA1_Channel2->CNDTR;
    if (count < 0) {
      count += RF_RX_CAPTURE_BUFFER_LEN;
    }
    rfRxDmaToBeProcessedCount += count;
    if (rfRxDmaToBeProcessedCount > RF_RX_CAPTURE_BUFFER_LEN) {
      rfRxDmaToBeProcessedCount = 0;
      debug_write_line("Buffer overflow");
    }
  }
}

void rf_rx_process_capture_buffer() {
  while (rfRxDmaToBeProcessedCount > 0) {
    uint16_t val = rfRxCaptureBuffer[rfRxProcessOffset];

    debug_write_i32(val, 10);
    debug_write_line("");

    rfRxProcessOffset++;
    if (rfRxProcessOffset >= RF_RX_CAPTURE_BUFFER_LEN) {
      rfRxProcessOffset = 0;
    }
    rfRxDmaToBeProcessedCount--;
  }
}

void rf_rx_enable() {
  rfRxProcessOffset = 0;
  rfRxDmaToBeProcessedCount = 0;
  rfRxDmaLastCNDTR = RF_RX_CAPTURE_BUFFER_LEN;
  DMA1_Channel2->CNDTR = RF_RX_CAPTURE_BUFFER_LEN;
  DMA1_Channel2->CMAR = (uint32_t) & rfRxCaptureBuffer;
  DMA_Cmd(DMA1_Channel2, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}

void rf_rx_disable() {
  DMA_Cmd(DMA1_Channel2, DISABLE);
  TIM_Cmd(TIM1, DISABLE);
}

void usb_on_rx(uint8_t* data, uint16_t len) {
#define MAX_LINE_LENGTH 100
  char line[MAX_LINE_LENGTH];

  ring_buffer_u8_write(&usb_input_ring_buffer, data, len);
  while (ring_buffer_u8_readline(&usb_input_ring_buffer, line, MAX_LINE_LENGTH) > 0) {
    debug_write_line(line);
  }
}
