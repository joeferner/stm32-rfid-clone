#include <stm32f10x_exti.h>
#include <misc.h>
#include "usb_lib.h"
#include "usb.h"
#include "hw_config.h"
#include "debug.h"
#include "platform_config.h"

ErrorStatus HSEStartUpStatus;
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

extern LINE_CODING linecoding;

/**
 * Power-off system clocks and power while entering suspend mode
 */
void Enter_LowPowerMode(void) {
  /* Set the device state to suspend */
  g_usb_deviceState = SUSPENDED;
}

/**
 * Restores system clocks and power while exiting suspend mode
 */
void Leave_LowPowerMode(void) {
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0) {
    /* Device configured */
    g_usb_deviceState = CONFIGURED;
  } else {
    g_usb_deviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
  SystemInit();
}

/**
 * Software Connection/Disconnection of USB Cable
 */
void USB_Cable_Config(FunctionalState NewState) {
  if (NewState != DISABLE) {
    // P-channel MOSFET - ON - Pull USB D+ to 3.3V
    GPIO_SetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
  } else {
    // P-channel MOSFET - OFF - Give control to the micro
    GPIO_ResetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
  }
}

/**
 * Create the serial number string descriptor.
 */
void Get_SerialNum(void) {
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*) ID1;
  Device_Serial1 = *(uint32_t*) ID2;
  Device_Serial2 = *(uint32_t*) ID3;

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0) {
    IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2], 8);
    IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/**
 * Convert Hex 32Bits value into char.
 */
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++) {
    if (((value >> 28)) < 0xA) {
      pbuf[ 2 * idx] = (value >> 28) + '0';
    } else {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2 * idx + 1] = 0;
  }
}
