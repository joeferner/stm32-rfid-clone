
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

#define RF_TX_CARRIER_FREQ       125000
#define RF_TX_PWM_PERIOD         (SystemCoreClock / RF_TX_CARRIER_FREQ)
#define RF_RX_CENTER_0           (SystemCoreClock / (RF_TX_CARRIER_FREQ / 8))  /* 15.6kHz */
#define RF_RX_CENTER_1           (SystemCoreClock / (RF_TX_CARRIER_FREQ / 10)) /* 12.5kHz */
#define RF_RX_CENTER_DIFF        (RF_RX_CENTER_1 - RF_RX_CENTER_0)
#define RF_RX_CENTER_DIFF_DIV_2  (RF_RX_CENTER_DIFF / 2)
#define RF_RX_MIN_0              (RF_RX_CENTER_0 - RF_RX_CENTER_DIFF_DIV_2)
#define RF_RX_MAX_0              (RF_RX_CENTER_0 + RF_RX_CENTER_DIFF_DIV_2)
#define RF_RX_MIN_1              (RF_RX_CENTER_1 - RF_RX_CENTER_DIFF_DIV_2)
#define RF_RX_MAX_1              (RF_RX_CENTER_1 + RF_RX_CENTER_DIFF_DIV_2)
#define RF_RX_0                  0
#define RF_RX_1                  1
#define RF_RX_UNKNOWN            2

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
uint16_t rf_rx_get(int i);
void rf_rx_process(int i);
void rf_rx_reset_read();
void rf_rx_process_read_buffer();

#define INPUT_BUFFER_SIZE 100
uint8_t usbInputBuffer[INPUT_BUFFER_SIZE];
ring_buffer_u8 usbInputRingBuffer;

#define RX_RX_CAPTURE_BUFFER_HIGH 0x01
#define RX_RX_CAPTURE_BUFFER_LOW  0x02
#define RF_RX_CAPTURE_BUFFER_LEN 1000
uint16_t rfRxCaptureBuffer[RF_RX_CAPTURE_BUFFER_LEN];
volatile uint8_t rfRxCaptureBufferReady;

#define READ_BUFFER_LEN 100
#define READ_STATE_WAIT_FOR_START        0
#define READ_STATE_WAIT_FOR_END_OF_START 1
#define READ_STATE_DATA                  2
#define READ_STATE_END                   3
uint8_t readState;
int8_t readCount; // >0 number of 1s. <0 number of 0s. 0 waiting for 0 or 1.
uint16_t readBufferOffset;
uint8_t readBuffer[READ_BUFFER_LEN];

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

  ring_buffer_u8_init(&usbInputRingBuffer, usbInputBuffer, INPUT_BUFFER_SIZE);

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
  timeBaseInit.TIM_Prescaler = 0;
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
  timerInputCaptureInit.TIM_ICFilter = 0x8;
  TIM_ICInit(TIM1, &timerInputCaptureInit);

  TIM_SelectCCDMA(TIM1, ENABLE);

  TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);

  debug_write_line("?END rf_rx_setup");
}

void on_dma1_ch2_irq() {
  if (DMA_GetITStatus(DMA1_IT_TC2) || DMA_GetITStatus(DMA1_IT_HT2)) {
    DMA_ClearITPendingBit(DMA1_IT_GL2);
    uint16_t dmaLeftToWrite = DMA1_Channel2->CNDTR;
    if (dmaLeftToWrite > (RF_RX_CAPTURE_BUFFER_LEN / 2)) {
      rfRxCaptureBufferReady |= RX_RX_CAPTURE_BUFFER_HIGH;
    } else {
      rfRxCaptureBufferReady |= RX_RX_CAPTURE_BUFFER_LOW;
    }
  }
}

void rf_rx_process_capture_buffer() {
  if (rfRxCaptureBufferReady & RX_RX_CAPTURE_BUFFER_LOW) {
    rfRxCaptureBufferReady &= ~RX_RX_CAPTURE_BUFFER_LOW;
    for (int i = 0; i < RF_RX_CAPTURE_BUFFER_LEN / 2; i++) {
      rf_rx_process(i);
    }
  }

  if (rfRxCaptureBufferReady & RX_RX_CAPTURE_BUFFER_HIGH) {
    rfRxCaptureBufferReady &= ~RX_RX_CAPTURE_BUFFER_HIGH;
    for (int i = RF_RX_CAPTURE_BUFFER_LEN / 2; i < RF_RX_CAPTURE_BUFFER_LEN; i++) {
      rf_rx_process(i);
    }
  }
}

uint16_t rf_rx_get(int i) {
  if (i > 0) {
    return rfRxCaptureBuffer[i] - rfRxCaptureBuffer[i - 1];
  } else {
    return rfRxCaptureBuffer[i] - rfRxCaptureBuffer[RF_RX_CAPTURE_BUFFER_LEN - 1];
  }
}

void rf_rx_process(int i) {
  uint16_t t = rf_rx_get(i);
  if (t == 0) {
    return;
  }

  int bit;
  if (t >= RF_RX_MIN_0 && t <= RF_RX_MAX_0) {
    bit = RF_RX_0;
  } else if (t >= RF_RX_MIN_1 && t <= RF_RX_MAX_1) {
    bit = RF_RX_1;
  } else {
    bit = RF_RX_UNKNOWN;
  }

  // we could be adding two bits to the read buffer
  if (bit == RF_RX_UNKNOWN || readBufferOffset >= (READ_BUFFER_LEN - 2)) {
    rf_rx_reset_read();
    return;
  }

  switch (readState) {
    case READ_STATE_WAIT_FOR_START:
      if (bit == RF_RX_1) {
        readCount++;
        if (readCount >= 15) {
          readState = READ_STATE_WAIT_FOR_END_OF_START;
        }
      } else {
        readCount = 0;
      }
      break;

    case READ_STATE_WAIT_FOR_END_OF_START:
      if (bit == RF_RX_0) {
        readState = READ_STATE_DATA;
        readCount = 0;
      } else {
        break;
      }
      // NOTE: yes the lack of break here is intentional, if this is a data bit we need to process it.

    case READ_STATE_DATA:
      if (bit == RF_RX_0) {
        if (readCount > 0) { // if last read is >0 it means the last bit was a one.
          if (readCount >= 3 && readCount <= 8) {
            readBuffer[readBufferOffset++] = 1;
          } else if (readCount <= 14) {
            readBuffer[readBufferOffset++] = 1;
            readBuffer[readBufferOffset++] = 1;
          } else {
            readState = READ_STATE_END;
            rf_rx_process_read_buffer();
            return;
          }
          readCount = 0;
        }
        readCount--;
      } else /* (bit == RF_RX_1) */ {
        if (readCount < 0) { // if last read is <0 it means the last bit was a zero.
          readCount = -readCount;
          if (readCount >= 3 && readCount <= 8) {
            readBuffer[readBufferOffset++] = 0;
          } else if (readCount <= 14) {
            readBuffer[readBufferOffset++] = 0;
            readBuffer[readBufferOffset++] = 0;
          } else {
            readState = READ_STATE_END;
            rf_rx_process_read_buffer();
            return;
          }
          readCount = 0;
        }
        readCount++;
      }
      break;
  }
}

void rf_rx_process_read_buffer() {
  rf_rx_disable();
  debug_write_line("");
  for (uint16_t i = 0; i < readBufferOffset; i += 2) {
    if (readBuffer[i] == 1 && readBuffer[i + 1] == 0) {
      debug_write_ch('1');
    } else if (readBuffer[i] == 0 && readBuffer[i + 1] == 1) {
      debug_write_ch('0');
    } else {
      debug_write_ch('E');
    }
  }
  debug_write_line("");
  rf_rx_enable();
}

void rf_rx_reset_read() {
  readState = READ_STATE_WAIT_FOR_START;
  readCount = 0;
  readBufferOffset = 0;
}

void rf_rx_enable() {
  debug_write_line("?rf_rx_enable");
  rf_rx_reset_read();
  DMA_Cmd(DMA1_Channel2, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}

void rf_rx_disable() {
  debug_write_line("?rf_rx_disable");
  DMA_Cmd(DMA1_Channel2, DISABLE);
  TIM_Cmd(TIM1, DISABLE);
}

void usb_on_rx(uint8_t* data, uint16_t len) {
#define MAX_LINE_LENGTH 100
  char line[MAX_LINE_LENGTH];

  ring_buffer_u8_write(&usbInputRingBuffer, data, len);
  while (ring_buffer_u8_readline(&usbInputRingBuffer, line, MAX_LINE_LENGTH) > 0) {
    debug_write_line(line);
  }
}
