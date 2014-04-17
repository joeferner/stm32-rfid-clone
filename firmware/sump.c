
#include "sump.h"
#include "ring_buffer.h"
#include "usb.h"

#define SUMP_RESET                0x00
#define SUMP_ARM                  0x01
#define SUMP_QUERY                0x02
#define SUMP_SELF_TEST            0x03
#define SUMP_GET_METADATA         0x04
#define SUMP_XON                  0x11
#define SUMP_XOFF                 0x13
#define SUMP_TRIGGER_MASK         0xC0
#define SUMP_TRIGGER_VALUES       0xC1
#define SUMP_TRIGGER_CONFIG       0xC2
#define SUMP_SET_DIVIDER          0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS            0x82

#define INPUT_BUFFER_SIZE 100
uint8_t usbInputBuffer[INPUT_BUFFER_SIZE];
ring_buffer_u8 usbInputRingBuffer;

uint8_t _sump_get_command(uint8_t *cmd);

void sump_setup() {
  ring_buffer_u8_init(&usbInputRingBuffer, usbInputBuffer, INPUT_BUFFER_SIZE);

  g_sump_config.channel = 0;
  g_sump_config.delay = 0;
  g_sump_config.level = 0;
  g_sump_config.serial = 0;
  g_sump_config.start = 0;
}

void sump_loop() {
  uint8_t cmdBytes[5];

  while (ring_buffer_u8_available(&usbInputRingBuffer) > 0) {
    uint8_t d = ring_buffer_u8_peek(&usbInputRingBuffer);
    switch (d) {
      case SUMP_RESET:
        ring_buffer_u8_read_byte(&usbInputRingBuffer);
        // do nothing
        break;
      case SUMP_QUERY:
        ring_buffer_u8_read_byte(&usbInputRingBuffer);
        sump_tx('1');
        sump_tx('A');
        sump_tx('L');
        sump_tx('S');
        break;
      case SUMP_ARM:
        ring_buffer_u8_read_byte(&usbInputRingBuffer);
        g_sump_enabled = 1;
        break;
      case SUMP_TRIGGER_MASK:
        _sump_get_command(g_sump_trigger);
        break;
      case SUMP_TRIGGER_VALUES:
        _sump_get_command(g_sump_trigger_values);
        break;
      case SUMP_TRIGGER_CONFIG:
        if (_sump_get_command(cmdBytes)) {
          g_sump_config.delay = (uint16_t) cmdBytes[0] | ((uint16_t) cmdBytes[1] << 8);
          g_sump_config.channel = ((cmdBytes[2] >> 4) & 0x0f) | ((cmdBytes[3] << 4) & 0x10);
          g_sump_config.level = cmdBytes[2] & 0x03;
          g_sump_config.start = (cmdBytes[3] & 0x08) ? 1 : 0;
          g_sump_config.serial = (cmdBytes[3] & 0x04) ? 1 : 0;
        }
        break;
      case SUMP_SET_DIVIDER:
        if (_sump_get_command(cmdBytes)) {
          g_sump_divider = cmdBytes[2];
          g_sump_divider = g_sump_divider << 8;
          g_sump_divider |= cmdBytes[1];
          g_sump_divider = g_sump_divider << 8;
          g_sump_divider |= cmdBytes[0];
        }
        break;
      case SUMP_SET_READ_DELAY_COUNT:
        if (_sump_get_command(cmdBytes)) {
          g_sump_read_count = cmdBytes[1];
          g_sump_read_count = g_sump_read_count << 8;
          g_sump_read_count |= cmdBytes[0];

          g_sump_delay_count = cmdBytes[3];
          g_sump_delay_count = g_sump_delay_count << 8;
          g_sump_delay_count |= cmdBytes[2];
        }
        break;
      case SUMP_SET_FLAGS:
        if (_sump_get_command(cmdBytes)) {
          g_sump_flags.inverted = (cmdBytes[0] & 0x80) ? 1 : 0;
          g_sump_flags.external = (cmdBytes[0] & 0x40) ? 1 : 0;
          g_sump_flags.channelGroup0 = (cmdBytes[0] & 0x20) ? 1 : 0;
          g_sump_flags.channelGroup1 = (cmdBytes[0] & 0x10) ? 1 : 0;
          g_sump_flags.channelGroup2 = (cmdBytes[0] & 0x08) ? 1 : 0;
          g_sump_flags.channelGroup3 = (cmdBytes[0] & 0x04) ? 1 : 0;
          g_sump_flags.filter = (cmdBytes[0] & 0x02) ? 1 : 0;
          g_sump_flags.demux = (cmdBytes[0] & 0x01) ? 1 : 0;
          g_sump_flags.rle = (cmdBytes[1] & 0x80) ? 1 : 0;
        }
        break;
      case SUMP_GET_METADATA:
        // device name
        sump_tx(0x01);
        sump_tx('s');
        sump_tx('t');
        sump_tx('m');
        sump_tx('3');
        sump_tx('2');
        sump_tx(0x00);

        // version
        sump_tx(0x02);
        sump_tx('0');
        sump_tx('.');
        sump_tx('1');
        sump_tx(0x00);

        // protocol vertex 2
        sump_tx(0x41);
        sump_tx(0x02);

        // end of message
        sump_tx(0x00);
        break;
      case SUMP_SELF_TEST:
        // ignore
        break;
    }
  }
}

void sump_tx(uint8_t data) {
  usb_write_u8(data);
}

uint8_t _sump_get_command(uint8_t *cmd) {
  if (ring_buffer_u8_available(&usbInputRingBuffer) < 5) {
    return 0;
  }
  ring_buffer_u8_read_byte(&usbInputRingBuffer); // skip command byte
  cmd[0] = ring_buffer_u8_read_byte(&usbInputRingBuffer);
  cmd[1] = ring_buffer_u8_read_byte(&usbInputRingBuffer);
  cmd[2] = ring_buffer_u8_read_byte(&usbInputRingBuffer);
  cmd[3] = ring_buffer_u8_read_byte(&usbInputRingBuffer);
  return 1;
}

void usb_on_rx(uint8_t* data, uint16_t len) {
  ring_buffer_u8_write(&usbInputRingBuffer, data, len);
}
