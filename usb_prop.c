#include "usb.h"
#include "hw_config.h"
#include "delay.h"

uint8_t Request = 0;

LINE_CODING linecoding = {
  115200, /* baud rate*/
  0x00, /* stop bits-1*/
  0x00, /* parity - none*/
  0x08 /* no. of bits 8*/
};

DEVICE Device_Table = {
  EP_NUM,
  1
};

DEVICE_PROP Device_Property = {
  Virtual_Com_Port_init,
  Virtual_Com_Port_Reset,
  Virtual_Com_Port_Status_In,
  Virtual_Com_Port_Status_Out,
  Virtual_Com_Port_Data_Setup,
  Virtual_Com_Port_NoData_Setup,
  Virtual_Com_Port_Get_Interface_Setting,
  Virtual_Com_Port_GetDeviceDescriptor,
  Virtual_Com_Port_GetConfigDescriptor,
  Virtual_Com_Port_GetStringDescriptor,
  0,
  0x40 /*MAX PACKET SIZE*/
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
  Virtual_Com_Port_GetConfiguration,
  Virtual_Com_Port_SetConfiguration,
  Virtual_Com_Port_GetInterface,
  Virtual_Com_Port_SetInterface,
  Virtual_Com_Port_GetStatus,
  Virtual_Com_Port_ClearFeature,
  Virtual_Com_Port_SetEndPointFeature,
  Virtual_Com_Port_SetDeviceFeature,
  Virtual_Com_Port_SetDeviceAddress
};

ONE_DESCRIPTOR Device_Descriptor = {
  (uint8_t*) Virtual_Com_Port_DeviceDescriptor,
  VIRTUAL_COM_PORT_SIZ_DEVICE_DESC
};

ONE_DESCRIPTOR Config_Descriptor = {
  (uint8_t*) Virtual_Com_Port_ConfigDescriptor,
  VIRTUAL_COM_PORT_SIZ_CONFIG_DESC
};

ONE_DESCRIPTOR String_Descriptor[5] = {
  {(uint8_t*) Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID},
  {(uint8_t*) Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR},
  {(uint8_t*) Virtual_Com_Port_StringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT},
  {(uint8_t*) Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL},
  {NULL, 0}
};

void Virtual_Com_Port_init() {
  /* Update the serial number string descriptor with the data from the unique ID */
  Get_SerialNum();

  pInformation->Current_Configuration = 0;

  delay_ms(1); // TODO figure out why this is needed? If this isn't there the USB_Init process hangs.

  /* Connect the device */
  PowerOn();

  /* Perform basic device initialization operations */
  USB_SIL_Init();

  g_usb_deviceState = UNCONNECTED;
}

void Virtual_Com_Port_Reset() {
  /* Set Virtual_Com_Port DEVICE as not configured */
  pInformation->Current_Configuration = 0;

  /* Current Feature initialization */
  pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];

  /* Set Virtual_Com_Port DEVICE with the default Interface*/
  pInformation->Current_Interface = 0;

  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
  SetEPTxStatus(ENDP0, EP_TX_STALL);
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  Clear_Status_Out(ENDP0);
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
  SetEPRxValid(ENDP0);

  /* Initialize Endpoint 1 */
  SetEPType(ENDP1, EP_BULK);
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);
  SetEPTxStatus(ENDP1, EP_TX_NAK);
  SetEPRxStatus(ENDP1, EP_RX_DIS);

  /* Initialize Endpoint 2 */
  SetEPType(ENDP2, EP_INTERRUPT);
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);
  SetEPRxStatus(ENDP2, EP_RX_DIS);
  SetEPTxStatus(ENDP2, EP_TX_NAK);

  /* Initialize Endpoint 3 */
  SetEPType(ENDP3, EP_BULK);
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
  SetEPRxStatus(ENDP3, EP_RX_VALID);
  SetEPTxStatus(ENDP3, EP_TX_DIS);

  /* Set this device to response on default address */
  SetDeviceAddress(0);

  g_usb_deviceState = ATTACHED;
}

void Virtual_Com_Port_SetConfiguration() {
  DEVICE_INFO *pInfo = &Device_Info;

  if (pInfo->Current_Configuration != 0) {
    g_usb_deviceState = CONFIGURED;
  }
}

void Virtual_Com_Port_SetDeviceAddress() {
  g_usb_deviceState = ADDRESSED;
}

void Virtual_Com_Port_Status_In() {
  if (Request == SET_LINE_CODING) {
    Request = 0;
  }
}

void Virtual_Com_Port_Status_Out() {
}

RESULT Virtual_Com_Port_Data_Setup(uint8_t requestNo) {
  uint8_t * (*CopyRoutine)(uint16_t);
  CopyRoutine = NULL;

  if (requestNo == GET_LINE_CODING) {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
      CopyRoutine = Virtual_Com_Port_GetLineCoding;
    }
  } else if (requestNo == SET_LINE_CODING) {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
      CopyRoutine = Virtual_Com_Port_SetLineCoding;
    }
    Request = SET_LINE_CODING;
  }

  if (CopyRoutine == NULL) {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);
  return USB_SUCCESS;
}

RESULT Virtual_Com_Port_NoData_Setup(uint8_t requestNo) {
  if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
    if (requestNo == SET_COMM_FEATURE) {
      return USB_SUCCESS;
    } else if (requestNo == SET_CONTROL_LINE_STATE) {
      return USB_SUCCESS;
    }
  }

  return USB_UNSUPPORT;
}

uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t length) {
  return Standard_GetDescriptorData(length, &Device_Descriptor);
}

uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t length) {
  return Standard_GetDescriptorData(length, &Config_Descriptor);
}

uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t length) {
  uint8_t wValue0 = pInformation->USBwValue0;
  if (wValue0 > 4) {
    return NULL;
  } else {
    return Standard_GetDescriptorData(length, &String_Descriptor[wValue0]);
  }
}

RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t interface, uint8_t alternateSetting) {
  if (alternateSetting > 0) {
    return USB_UNSUPPORT;
  } else if (interface > 1) {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}

uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t length) {
  if (length == 0) {
    pInformation->Ctrl_Info.Usb_wLength = sizeof (linecoding);
    return NULL;
  }
  return (uint8_t*)&linecoding;
}

uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t length) {
  if (length == 0) {
    pInformation->Ctrl_Info.Usb_wLength = sizeof (linecoding);
    return NULL;
  }
  return (uint8_t*)&linecoding;
}

