#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include <stm32f10x.h>
#include <usb_lib.h>
#include <stdio.h>
#include <stdint.h>

#define USART_RX_DATA_SIZE   2048

void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Cable_Config (FunctionalState NewState);
void Get_SerialNum(void);

#endif  /*__HW_CONFIG_H*/
