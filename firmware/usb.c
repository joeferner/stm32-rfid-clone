#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_gpio.h>
#include <misc.h>
#include "usb.h"
#include "debug.h"
#include "platform_config.h"
#include "ring_buffer.h"
#include "delay.h"

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

#define USB_TX_BUFFER_SIZE 100
uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
ring_buffer_u8 usb_tx_ring_buffer;
uint8_t g_usb_initialized = 0;

void usb_setup(void) {
  GPIO_InitTypeDef gpioInitStructure;
  EXTI_InitTypeDef extiInitStructure;
  NVIC_InitTypeDef nvicInitStructure;

  g_usb_initialized = 0;
  debug_write_line("?BEGIN usb_setup");

  ring_buffer_u8_init(&usb_tx_ring_buffer, usb_tx_buffer, USB_TX_BUFFER_SIZE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Enable the USB disconnect GPIO clock */
  RCC_APB2PeriphClockCmd(USB_DISCONNECT_RCC, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  gpioInitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  gpioInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(USB_DISCONNECT_PORT, &gpioInitStructure);

  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  extiInitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInitStructure.EXTI_Line = EXTI_Line18;
  extiInitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  extiInitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInitStructure);

  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  USB_Cable_Config(DISABLE);

  nvicInitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  nvicInitStructure.NVIC_IRQChannelSubPriority = 0;
  nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInitStructure);

  /* Enable the USB Wake-up interrupt */
  nvicInitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&nvicInitStructure);

  debug_write_line("?USB_Init");
  USB_Init();
  delay_ms(100); // TODO remove?

  debug_write_line("?END usb_setup");

  g_usb_initialized = 1;
}

int usb_detect() {
  GPIO_InitTypeDef gpioInitStructure;

  /* USB_DETECT input */
  RCC_APB2PeriphClockCmd(USB_DETECT_RCC, ENABLE);
  gpioInitStructure.GPIO_Pin = USB_DETECT_PIN;
  gpioInitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(USB_DETECT_PORT, &gpioInitStructure);

  return GPIO_ReadInputDataBit(USB_DETECT_PORT, USB_DETECT_PIN);
}

/* !!! Interrupt handler - Don't change this function name !!! */
void USB_LP_CAN1_RX0_IRQHandler(void) {
  USB_Istr();
}

/* !!! Interrupt handler - Don't change this function name !!! */
void USBWakeUp_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line18);
}

void usb_write(const uint8_t* data, uint16_t len) {
  ring_buffer_u8_write(&usb_tx_ring_buffer, data, len);
}

void usb_write_u8(uint8_t data) {
  ring_buffer_u8_write_byte(&usb_tx_ring_buffer, data);
}

uint16_t usb_write_free() {
  return ring_buffer_u8_free(&usb_tx_ring_buffer);
}

/**
 * send data to USB.
 */
void usb_send_data(void) {
  uint8_t txBuffer[VIRTUAL_COM_PORT_DATA_SIZE];
  uint16_t txCount;

  if (g_usb_deviceState != CONFIGURED) {
    return;
  }

  txCount = VIRTUAL_COM_PORT_DATA_SIZE;
  if (txCount > usb_tx_ring_buffer.available) {
    txCount = usb_tx_ring_buffer.available;
  }
  if (txCount > 0) {
    ring_buffer_u8_read(&usb_tx_ring_buffer, txBuffer, txCount);
    UserToPMABufferCopy(txBuffer, ENDP1_TXADDR, txCount);
    SetEPTxCount(ENDP1, txCount);
    SetEPTxValid(ENDP1);
  }
}

void EP1_IN_Callback(void) {
  usb_send_data();
}

void EP3_OUT_Callback(void) {
  uint8_t rxBuffer[VIRTUAL_COM_PORT_DATA_SIZE];
  uint16_t rxCount;

  /* Get the received data buffer and update the counter */
  rxCount = USB_SIL_Read(EP3_OUT, rxBuffer);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the USART Xfer */

  usb_on_rx(rxBuffer, rxCount);

  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
}

void SOF_Callback(void) {
  static uint32_t FrameCount = 0;

  if (g_usb_deviceState == CONFIGURED) {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL) {
      /* Reset the frame counter */
      FrameCount = 0;

      /* Check the data to be sent through IN pipe */
      usb_send_data();
    }
  }
}
