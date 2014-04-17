
#ifndef USB_H
#define	USB_H

#include "usb_lib.h"

#ifdef	__cplusplus
extern "C" {
#endif  

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define VIRTUAL_COM_PORT_DATA_SIZE              64
#define VIRTUAL_COM_PORT_INT_SIZE               8

#define VIRTUAL_COM_PORT_SIZ_DEVICE_DESC        18
#define VIRTUAL_COM_PORT_SIZ_CONFIG_DESC        67
#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
#define VIRTUAL_COM_PORT_SIZ_STRING_VENDOR      ((2 * 10) + 2)
#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     ((2 * 9) + 2)
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      (((2 * 5) + 2) + 14)

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

#define Virtual_Com_Port_GetConfiguration       NOP_Process
  //#define Virtual_Com_Port_SetConfiguration       NOP_Process
#define Virtual_Com_Port_GetInterface           NOP_Process
#define Virtual_Com_Port_SetInterface           NOP_Process
#define Virtual_Com_Port_GetStatus              NOP_Process
#define Virtual_Com_Port_ClearFeature           NOP_Process
#define Virtual_Com_Port_SetEndPointFeature     NOP_Process
#define Virtual_Com_Port_SetDeviceFeature       NOP_Process
  //#define Virtual_Com_Port_SetDeviceAddress       NOP_Process

#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

  typedef struct {
    uint32_t bitrate;
    uint8_t format;
    uint8_t paritytype;
    uint8_t datatype;
  } LINE_CODING;

  typedef enum _RESUME_STATE {
    RESUME_EXTERNAL,
    RESUME_INTERNAL,
    RESUME_LATER,
    RESUME_WAIT,
    RESUME_START,
    RESUME_ON,
    RESUME_OFF,
    RESUME_ESOF
  } RESUME_STATE;

  typedef enum _DEVICE_STATE {
    UNCONNECTED,
    ATTACHED,
    POWERED,
    SUSPENDED,
    ADDRESSED,
    CONFIGURED
  } DEVICE_STATE;

  extern __IO uint32_t g_usb_deviceState; /* USB device status */
  extern __IO bool g_usb_suspendEnabled; /* true when suspend is possible */
  extern uint8_t g_usb_initialized;

  void usb_setup();
  int usb_detect();
  extern void usb_on_rx(uint8_t* data, uint16_t len);
  void usb_write(const uint8_t* data, uint16_t len);
  void usb_write_str(const char* data);
  void usb_write_u8(uint8_t data);

  void USB_Istr(void);

  void EP1_IN_Callback(void);
  void EP2_IN_Callback(void);
  void EP3_IN_Callback(void);
  void EP4_IN_Callback(void);
  void EP5_IN_Callback(void);
  void EP6_IN_Callback(void);
  void EP7_IN_Callback(void);

  void EP1_OUT_Callback(void);
  void EP2_OUT_Callback(void);
  void EP3_OUT_Callback(void);
  void EP4_OUT_Callback(void);
  void EP5_OUT_Callback(void);
  void EP6_OUT_Callback(void);
  void EP7_OUT_Callback(void);

  extern const uint8_t Virtual_Com_Port_DeviceDescriptor[VIRTUAL_COM_PORT_SIZ_DEVICE_DESC];
  extern const uint8_t Virtual_Com_Port_ConfigDescriptor[VIRTUAL_COM_PORT_SIZ_CONFIG_DESC];

  extern const uint8_t Virtual_Com_Port_StringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID];
  extern const uint8_t Virtual_Com_Port_StringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR];
  extern const uint8_t Virtual_Com_Port_StringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT];
  extern uint8_t Virtual_Com_Port_StringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL];

  void Virtual_Com_Port_init(void);
  void Virtual_Com_Port_Reset(void);
  void Virtual_Com_Port_SetConfiguration(void);
  void Virtual_Com_Port_SetDeviceAddress(void);
  void Virtual_Com_Port_Status_In(void);
  void Virtual_Com_Port_Status_Out(void);
  RESULT Virtual_Com_Port_Data_Setup(uint8_t);
  RESULT Virtual_Com_Port_NoData_Setup(uint8_t);
  RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
  uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t);
  uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t);
  uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t);

  uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t Length);
  uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t Length);

  void Suspend(void);
  void Resume_Init(void);
  void Resume(RESUME_STATE eResumeSetVal);
  RESULT PowerOn(void);
  RESULT PowerOff(void);

#ifdef CTR_CALLBACK
  void CTR_Callback(void);
#endif

#ifdef DOVR_CALLBACK
  void DOVR_Callback(void);
#endif

#ifdef ERR_CALLBACK
  void ERR_Callback(void);
#endif

#ifdef WKUP_CALLBACK
  void WKUP_Callback(void);
#endif

#ifdef SUSP_CALLBACK
  void SUSP_Callback(void);
#endif

#ifdef RESET_CALLBACK
  void RESET_Callback(void);
#endif

#ifdef SOF_CALLBACK
  void SOF_Callback(void);
#endif

#ifdef ESOF_CALLBACK
  void ESOF_Callback(void);
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* USB_H */

