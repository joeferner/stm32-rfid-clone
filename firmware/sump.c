
#include "sump.h"
#include "ring_buffer.h"
#include "usb.h"
#include "debug.h"
#include "delay.h"

#define SUMP_RESET                0x00
#define SUMP_ARM                  0x01
#define SUMP_QUERY                0x02
#define SUMP_SELF_TEST            0x03
#define SUMP_GET_METADATA         0x04
#define SUMP_XON                  0x11
#define SUMP_XOFF                 0x13
#define SUMP_TRIGGER_MASK_CH0     0xC0
#define SUMP_TRIGGER_MASK_CH1     0xC4
#define SUMP_TRIGGER_MASK_CH2     0xC8
#define SUMP_TRIGGER_MASK_CH3     0xCC
#define SUMP_TRIGGER_VALUES_CH0   0xC1
#define SUMP_TRIGGER_VALUES_CH1   0xC5
#define SUMP_TRIGGER_VALUES_CH2   0xC9
#define SUMP_TRIGGER_VALUES_CH3   0xCD
#define SUMP_TRIGGER_CONFIG_CH0   0xC2
#define SUMP_TRIGGER_CONFIG_CH1   0xC6
#define SUMP_TRIGGER_CONFIG_CH2   0xCA
#define SUMP_TRIGGER_CONFIG_CH3   0xCE
#define SUMP_SET_DIVIDER          0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS            0x82

#define INPUT_BUFFER_SIZE 8000
uint8_t _g_sump_usbInputBuffer[INPUT_BUFFER_SIZE];
ring_buffer_u8 _g_sump_usbInputRingBuffer;

uint8_t _g_sump_triggered;

uint8_t _sump_get_command(uint8_t *cmd);

void sump_setup() {
  ring_buffer_u8_init(&_g_sump_usbInputRingBuffer, _g_sump_usbInputBuffer, INPUT_BUFFER_SIZE);

  int ch;
  for (ch = 0; ch < 4; ch++) {
    g_sump_trigger[ch][0] = 0;
    g_sump_trigger[ch][1] = 0;
    g_sump_trigger[ch][2] = 0;
    g_sump_trigger[ch][3] = 0;
    g_sump_trigger_values[ch][0] = 0;
    g_sump_trigger_values[ch][1] = 0;
    g_sump_trigger_values[ch][2] = 0;
    g_sump_trigger_values[ch][3] = 0;
    g_sump_config[ch].channel = 0;
    g_sump_config[ch].delay = 0;
    g_sump_config[ch].level = 0;
    g_sump_config[ch].serial = 0;
    g_sump_config[ch].start = 0;
  }
}

void sump_loop() {
  uint8_t cmdBytes[5];

  while (ring_buffer_u8_available(&_g_sump_usbInputRingBuffer) > 0) {
    uint8_t d = ring_buffer_u8_peek(&_g_sump_usbInputRingBuffer);
    switch (d) {
      case SUMP_RESET:
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
        // do nothing
        break;
      case SUMP_QUERY:
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
        usb_write_u8('1');
        usb_write_u8('A');
        usb_write_u8('L');
        usb_write_u8('S');
        break;
      case SUMP_ARM:
        debug_write_line("SUMP: arm");
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
        g_sump_enabled = 1;
        _g_sump_triggered = 0;
        break;
      case SUMP_TRIGGER_MASK_CH0:
        _sump_get_command(g_sump_trigger[0]);
        break;
      case SUMP_TRIGGER_MASK_CH1:
        _sump_get_command(g_sump_trigger[1]);
        break;
      case SUMP_TRIGGER_MASK_CH2:
        _sump_get_command(g_sump_trigger[2]);
        break;
      case SUMP_TRIGGER_MASK_CH3:
        _sump_get_command(g_sump_trigger[3]);
        break;
      case SUMP_TRIGGER_VALUES_CH0:
        _sump_get_command(g_sump_trigger_values[0]);
        break;
      case SUMP_TRIGGER_VALUES_CH1:
        _sump_get_command(g_sump_trigger_values[1]);
        break;
      case SUMP_TRIGGER_VALUES_CH2:
        _sump_get_command(g_sump_trigger_values[2]);
        break;
      case SUMP_TRIGGER_VALUES_CH3:
        _sump_get_command(g_sump_trigger_values[3]);
        break;
      case SUMP_TRIGGER_CONFIG_CH0:
      case SUMP_TRIGGER_CONFIG_CH1:
      case SUMP_TRIGGER_CONFIG_CH2:
      case SUMP_TRIGGER_CONFIG_CH3:
        if (_sump_get_command(cmdBytes)) {
          int ch = 0;
          switch (d) {
            case SUMP_TRIGGER_CONFIG_CH0:
              ch = 0;
              break;
            case SUMP_TRIGGER_CONFIG_CH1:
              ch = 1;
              break;
            case SUMP_TRIGGER_CONFIG_CH2:
              ch = 2;
              break;
            case SUMP_TRIGGER_CONFIG_CH3:
              ch = 3;
              break;
          }
          g_sump_config[ch].delay = (uint16_t) cmdBytes[0] | ((uint16_t) cmdBytes[1] << 8);
          g_sump_config[ch].channel = ((cmdBytes[2] >> 4) & 0x0f) | ((cmdBytes[3] << 4) & 0x10);
          g_sump_config[ch].level = cmdBytes[2] & 0x03;
          g_sump_config[ch].start = (cmdBytes[3] & 0x08) ? 1 : 0;
          g_sump_config[ch].serial = (cmdBytes[3] & 0x04) ? 1 : 0;
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

          g_sump_tx_left = 20 * (g_sump_read_count + 1);
          debug_write("SUMP: read count: ");
          debug_write_u32(g_sump_tx_left, 10);
          debug_write_line("");
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
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);

        // device name
        usb_write_u8(0x01);
        usb_write_u8('s');
        usb_write_u8('t');
        usb_write_u8('m');
        usb_write_u8('3');
        usb_write_u8('2');
        usb_write_u8(0x00);

        // version
        usb_write_u8(0x02);
        usb_write_u8('0');
        usb_write_u8('.');
        usb_write_u8('1');
        usb_write_u8(0x00);

        // sample memory
        usb_write_u8(0x21);
        usb_write_u8((SUMP_SAMPLE_MEMORY >> 24) & 0xff);
        usb_write_u8((SUMP_SAMPLE_MEMORY >> 16) & 0xff);
        usb_write_u8((SUMP_SAMPLE_MEMORY >> 8) & 0xff);
        usb_write_u8((SUMP_SAMPLE_MEMORY >> 0) & 0xff);

        // sample rate
        usb_write_u8(0x23);
        usb_write_u8((SUMP_SAMPLE_RATE >> 24) & 0xff);
        usb_write_u8((SUMP_SAMPLE_RATE >> 16) & 0xff);
        usb_write_u8((SUMP_SAMPLE_RATE >> 8) & 0xff);
        usb_write_u8((SUMP_SAMPLE_RATE >> 0) & 0xff);

        // number of probes
        usb_write_u8(0x40);
        usb_write_u8(SUMP_NUMBER_OF_PROBES);

        // protocol vertex 2
        usb_write_u8(0x41);
        usb_write_u8(0x02);

        // end of message
        usb_write_u8(0x00);
        break;
      case SUMP_SELF_TEST:
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
        // ignore
        break;

      default:
        ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
        debug_write("SUMP: Invalid command 0x");
        debug_write_u8(d, 16);
        debug_write_line("");
        break;
    }
  }
}

void sump_trigger() {
  _g_sump_triggered = 1;
}

uint8_t last = 0x00;

void sump_tx(uint8_t data) {
  if (g_sump_enabled && _g_sump_triggered) {
    if (usb_write_free() < 1) {
      return;
    }
    last = ((last ^ 0xff) & 0xf0) | (data & 0x0f);
    usb_write_u8(last);
    g_sump_tx_left--;
    if (g_sump_tx_left <= 0) {
      g_sump_enabled = 0;
      debug_write_line("SUMP: dump complete");
    }
  }
}

uint8_t _sump_get_command(uint8_t *cmd) {
  if (ring_buffer_u8_available(&_g_sump_usbInputRingBuffer) < 5) {
    return 0;
  }
  ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer); // skip command byte
  cmd[0] = ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
  cmd[1] = ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
  cmd[2] = ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
  cmd[3] = ring_buffer_u8_read_byte(&_g_sump_usbInputRingBuffer);
  return 1;
}

void usb_on_rx(uint8_t* data, uint16_t len) {
  ring_buffer_u8_write(&_g_sump_usbInputRingBuffer, data, len);
}
