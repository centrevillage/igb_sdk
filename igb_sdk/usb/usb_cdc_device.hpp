#pragma once

// USB CDC-ACM (virtual COM port) device — MCU-independent protocol layer.
//
// Owns the EP0 control-transfer state machine, the standard requests
// needed for enumeration, the CDC-ACM descriptors and class requests
// (SET/GET_LINE_CODING and SET_CONTROL_LINE_STATE are stored but unused).
// Bulk IN is the primary data path; bulk OUT is received and handed to an
// optional callback (discarded by default).
//
// DriverT provides the endpoint hardware operations (see
// igb_stm32/periph/usb_otg_device.hpp for the STM32 OTG implementation;
// host tests substitute a mock):
//
//   void setAddress(uint8_t address);
//   void openInEp(uint8_t ep, uint8_t ep_type, uint16_t max_packet);
//   void openOutEp(uint8_t ep, uint8_t ep_type, uint16_t max_packet);
//   void transmit(uint8_t ep, const uint8_t* buf, uint16_t len); // EP0: 1 packet max
//   void receive(uint8_t ep, uint8_t* buf, uint16_t len);
//   void ep0StartSetup();
//   void stallEp0();
//   void stallInEp(uint8_t ep);   void stallOutEp(uint8_t ep);
//   void clearStallInEp(uint8_t ep); void clearStallOutEp(uint8_t ep);
//
// This struct is also the delegate for DriverT::handleIrq(): the driver
// calls back onReset / onEnumDone / onSetupStage / onDataInStage /
// onDataOutStage / onSuspend / onResume / onSof.

#include <cstdint>
#include <cstddef>
#include <functional>

#include <igb_util/macro.hpp>

namespace igb {
namespace sdk {

struct UsbSetupPacket {
  uint8_t bm_request_type = 0;
  uint8_t b_request = 0;
  uint16_t w_value = 0;
  uint16_t w_index = 0;
  uint16_t w_length = 0;

  static UsbSetupPacket parse(const uint8_t* p) {
    return UsbSetupPacket {
      .bm_request_type = p[0],
      .b_request = p[1],
      .w_value = (uint16_t)(p[2] | ((uint16_t)p[3] << 8)),
      .w_index = (uint16_t)(p[4] | ((uint16_t)p[5] << 8)),
      .w_length = (uint16_t)(p[6] | ((uint16_t)p[7] << 8)),
    };
  }
};

struct UsbCdcDeviceConf {
  uint16_t vid = 0x0483;         // STMicroelectronics (same as libDaisy CDC)
  uint16_t pid = 0x5740;         // Virtual COM Port
  uint16_t bcd_device = 0x0200;
  const char* manufacturer = "igb"; // must stay valid (string literals)
  const char* product = "igb usb cdc";
  const char* serial = "000000000001"; // placeholder, override recommended
};

template<typename DriverT>
struct UsbCdcDevice {
  DriverT driver;

  // EP layout: EP0 control / EP1 bulk IN+OUT (data) / EP2 interrupt IN (notify)
  constexpr static uint8_t ep_data = 1;
  constexpr static uint8_t ep_notify = 2;
  constexpr static uint16_t data_max_packet = 64;
  constexpr static uint16_t notify_max_packet = 8;
  constexpr static uint16_t ep0_max_packet = 64;
  constexpr static uint8_t ep_type_bulk = 2;
  constexpr static uint8_t ep_type_interrupt = 3;

  // called with received bulk OUT data (default: discard). Runs in IRQ context.
  std::function<void(const uint8_t* data, size_t size)> on_rx;

  enum class Ep0State : uint8_t {
    idle = 0,
    dataIn,    // sending data stage packets (incl. trailing ZLP)
    dataOut,   // receiving data stage (e.g. SET_LINE_CODING payload)
    statusIn,  // sending status ZLP
    statusOut, // receiving status ZLP
  };

  UsbCdcDeviceConf _conf;
  Ep0State _ep0_state = Ep0State::idle;
  const uint8_t* _ep0_tx_ptr = nullptr;
  uint16_t _ep0_tx_remaining = 0;
  bool _ep0_zlp_needed = false;
  bool _ep0_out_is_line_coding = false;
  alignas(4) uint8_t _ep0_rx_buf[ep0_max_packet] = {};
  alignas(4) uint8_t _ep0_small_buf[2] = {};
  alignas(4) uint8_t _str_buf[64] = {};
  alignas(4) uint8_t _device_desc[18] = {};
  alignas(4) uint8_t _rx_buf[data_max_packet] = {};

  // 115200 8N1 (dwDTERate LE, bCharFormat, bParityType, bDataBits)
  alignas(4) uint8_t _line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };

  bool _configured = false;
  bool _suspended = false;
  bool _dtr = false;
  bool _rts = false;
  bool _tx_busy = false;

  // CDC-ACM configuration descriptor: config + comm IF (header/call
  // mgmt/ACM/union functional) + notify EP + data IF + bulk OUT/IN EPs.
  constexpr static uint8_t config_desc[67] = {
    // configuration
    9, 0x02, 67, 0, 2, 1, 0, 0xC0 /* self powered */, 0x32,
    // interface 0: communications (class 0x02, subclass ACM 0x02, protocol AT 0x01)
    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    // header functional (CDC 1.10)
    5, 0x24, 0x00, 0x10, 0x01,
    // call management functional (no call mgmt, data IF 1)
    5, 0x24, 0x01, 0x00, 0x01,
    // ACM functional (line coding + serial state)
    4, 0x24, 0x02, 0x02,
    // union functional (control IF 0, subordinate IF 1)
    5, 0x24, 0x06, 0x00, 0x01,
    // EP2 IN interrupt (notify), 8 bytes, interval 16ms
    7, 0x05, 0x80 | ep_notify, 0x03, notify_max_packet, 0, 0x10,
    // interface 1: CDC data (class 0x0A)
    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    // EP1 OUT bulk 64B
    7, 0x05, ep_data, 0x02, data_max_packet, 0, 0,
    // EP1 IN bulk 64B
    7, 0x05, 0x80 | ep_data, 0x02, data_max_packet, 0, 0,
  };

  void init(const UsbCdcDeviceConf& conf) {
    _conf = conf;
    uint8_t* d = _device_desc;
    d[0] = 18;         // bLength
    d[1] = 0x01;       // bDescriptorType: device
    d[2] = 0x00; d[3] = 0x02;  // bcdUSB 2.00
    d[4] = 0x02;       // bDeviceClass: CDC
    d[5] = 0x02;       // bDeviceSubClass
    d[6] = 0x00;       // bDeviceProtocol
    d[7] = ep0_max_packet;
    d[8] = (uint8_t)(conf.vid & 0xFF); d[9] = (uint8_t)(conf.vid >> 8);
    d[10] = (uint8_t)(conf.pid & 0xFF); d[11] = (uint8_t)(conf.pid >> 8);
    d[12] = (uint8_t)(conf.bcd_device & 0xFF); d[13] = (uint8_t)(conf.bcd_device >> 8);
    d[14] = 1;         // iManufacturer
    d[15] = 2;         // iProduct
    d[16] = 3;         // iSerialNumber
    d[17] = 1;         // bNumConfigurations
    _ep0_state = Ep0State::idle;
    _configured = false;
    _suspended = false;
    _dtr = false;
    _rts = false;
    _tx_busy = false;
  }

  IGB_FAST_INLINE bool isConfigured() const { return _configured && !_suspended; }
  IGB_FAST_INLINE bool isDtr() const { return _dtr; }
  // "terminal open": host enumerated us and a program asserted DTR
  IGB_FAST_INLINE bool isTerminalOpen() const { return isConfigured() && _dtr; }
  IGB_FAST_INLINE bool isTxBusy() const { return _tx_busy; }

  // Non-blocking bulk IN write. buf must stay valid until the transfer
  // completes (isTxBusy() returns false). Guard against concurrent
  // handleIrq() when calling from thread context.
  bool writeData(const uint8_t* buf, uint16_t len) {
    if (!isConfigured() || _tx_busy) { return false; }
    _tx_busy = true;
    driver.transmit(ep_data, buf, len);
    return true;
  }

  IGB_FAST_INLINE void handleIrq() {
    driver.handleIrq(*this);
  }

  // ----- driver delegate -----

  void onReset() {
    _configured = false;
    _suspended = false;
    _dtr = false;
    _rts = false;
    _tx_busy = false;
    _ep0_state = Ep0State::idle;
  }

  void onEnumDone() {}

  void onSuspend() { _suspended = true; }
  void onResume() { _suspended = false; }
  void onSof() {}

  void onSetupStage(const uint8_t* setup8) {
    const auto sp = UsbSetupPacket::parse(setup8);
    _ep0_zlp_needed = false;
    _ep0_out_is_line_coding = false;
    const uint8_t req_type = (sp.bm_request_type >> 5) & 0x3;
    if (req_type == 0) {
      _handleStandardRequest(sp);
    } else if (req_type == 1) {
      _handleClassRequest(sp);
    } else {
      driver.stallEp0();
      _ep0_state = Ep0State::idle;
    }
  }

  void onDataInStage(uint8_t ep) {
    if (ep == 0) {
      switch (_ep0_state) {
        case Ep0State::dataIn:
          if (_ep0_tx_remaining > 0) {
            _ep0SendChunk();
          } else if (_ep0_zlp_needed) {
            _ep0_zlp_needed = false;
            driver.transmit(0, nullptr, 0);
          } else {
            // data stage done → status OUT (host sends ZLP)
            _ep0_state = Ep0State::statusOut;
            driver.receive(0, _ep0_rx_buf, 0);
          }
          break;
        case Ep0State::statusIn:
          _ep0_state = Ep0State::idle;
          driver.ep0StartSetup();
          break;
        default:
          break;
      }
    } else if (ep == ep_data) {
      _tx_busy = false;
    }
  }

  void onDataOutStage(uint8_t ep, uint16_t count) {
    if (ep == 0) {
      switch (_ep0_state) {
        case Ep0State::dataOut:
          if (_ep0_out_is_line_coding && count >= 7) {
            for (uint8_t i = 0; i < 7; ++i) { _line_coding[i] = _ep0_rx_buf[i]; }
          }
          _ep0_state = Ep0State::statusIn;
          driver.transmit(0, nullptr, 0);
          break;
        case Ep0State::statusOut:
          _ep0_state = Ep0State::idle;
          driver.ep0StartSetup();
          break;
        default:
          break;
      }
    } else if (ep == ep_data) {
      if (on_rx && count > 0) {
        on_rx(_rx_buf, count);
      }
      driver.receive(ep_data, _rx_buf, data_max_packet); // re-arm
    }
  }

  // ----- internal -----

  void _handleStandardRequest(const UsbSetupPacket& sp) {
    const uint8_t recipient = sp.bm_request_type & 0x1F;
    switch (sp.b_request) {
      case 0x06: // GET_DESCRIPTOR
        _handleGetDescriptor(sp);
        break;
      case 0x05: // SET_ADDRESS (address must be set before the status stage)
        driver.setAddress((uint8_t)(sp.w_value & 0x7F));
        _sendStatusIn();
        break;
      case 0x09: // SET_CONFIGURATION
        if (sp.w_value <= 1) {
          if (sp.w_value == 1 && !_configured) {
            _openEndpoints();
          }
          _configured = (sp.w_value == 1);
          _sendStatusIn();
        } else {
          driver.stallEp0();
          _ep0_state = Ep0State::idle;
        }
        break;
      case 0x08: // GET_CONFIGURATION
        _ep0_small_buf[0] = _configured ? 1 : 0;
        _startDataIn(_ep0_small_buf, 1, sp.w_length);
        break;
      case 0x00: // GET_STATUS
        _ep0_small_buf[0] = (recipient == 0) ? 0x01 : 0x00; // device: self powered
        _ep0_small_buf[1] = 0x00;
        _startDataIn(_ep0_small_buf, 2, sp.w_length);
        break;
      case 0x01: // CLEAR_FEATURE
        if (recipient == 2 && sp.w_value == 0) { // ENDPOINT_HALT
          const uint8_t ep = sp.w_index & 0x0F;
          if (sp.w_index & 0x80) { driver.clearStallInEp(ep); }
          else { driver.clearStallOutEp(ep); }
        }
        _sendStatusIn();
        break;
      case 0x03: // SET_FEATURE
        if (recipient == 2 && sp.w_value == 0) { // ENDPOINT_HALT
          const uint8_t ep = sp.w_index & 0x0F;
          if (sp.w_index & 0x80) { driver.stallInEp(ep); }
          else { driver.stallOutEp(ep); }
        }
        _sendStatusIn();
        break;
      case 0x0A: // GET_INTERFACE
        _ep0_small_buf[0] = 0;
        _startDataIn(_ep0_small_buf, 1, sp.w_length);
        break;
      case 0x0B: // SET_INTERFACE (no alternate settings)
        _sendStatusIn();
        break;
      default:
        driver.stallEp0();
        _ep0_state = Ep0State::idle;
        break;
    }
  }

  void _handleGetDescriptor(const UsbSetupPacket& sp) {
    const uint8_t desc_type = (uint8_t)(sp.w_value >> 8);
    const uint8_t desc_idx = (uint8_t)(sp.w_value & 0xFF);
    switch (desc_type) {
      case 0x01: // device
        _startDataIn(_device_desc, sizeof(_device_desc), sp.w_length);
        break;
      case 0x02: // configuration
        _startDataIn(config_desc, sizeof(config_desc), sp.w_length);
        break;
      case 0x03: { // string
        uint16_t len = _buildStringDescriptor(desc_idx);
        if (len == 0) {
          driver.stallEp0();
          _ep0_state = Ep0State::idle;
        } else {
          _startDataIn(_str_buf, len, sp.w_length);
        }
        break;
      }
      default: // device qualifier etc. → stall (FS-only device)
        driver.stallEp0();
        _ep0_state = Ep0State::idle;
        break;
    }
  }

  void _handleClassRequest(const UsbSetupPacket& sp) {
    switch (sp.b_request) {
      case 0x20: // SET_LINE_CODING (7 byte data OUT)
        if (sp.w_length >= 7 && sp.w_length <= sizeof(_ep0_rx_buf)) {
          _ep0_out_is_line_coding = true;
          _ep0_state = Ep0State::dataOut;
          driver.receive(0, _ep0_rx_buf, sp.w_length);
        } else {
          driver.stallEp0();
          _ep0_state = Ep0State::idle;
        }
        break;
      case 0x21: // GET_LINE_CODING
        _startDataIn(_line_coding, 7, sp.w_length);
        break;
      case 0x22: // SET_CONTROL_LINE_STATE (wValue bit0=DTR, bit1=RTS)
        _dtr = sp.w_value & 0x1;
        _rts = sp.w_value & 0x2;
        _sendStatusIn();
        break;
      case 0x23: // SEND_BREAK
        _sendStatusIn();
        break;
      default:
        if (sp.w_length == 0) {
          _sendStatusIn(); // tolerate unknown no-data class requests
        } else {
          driver.stallEp0();
          _ep0_state = Ep0State::idle;
        }
        break;
    }
  }

  void _openEndpoints() {
    driver.openOutEp(ep_data, ep_type_bulk, data_max_packet);
    driver.openInEp(ep_data, ep_type_bulk, data_max_packet);
    driver.openInEp(ep_notify, ep_type_interrupt, notify_max_packet);
    driver.receive(ep_data, _rx_buf, data_max_packet);
    _tx_busy = false;
  }

  void _startDataIn(const uint8_t* ptr, uint16_t len, uint16_t w_length) {
    if (len > w_length) { len = w_length; }
    // exact multiple of the packet size AND shorter than requested
    // → a trailing ZLP tells the host the transfer is over
    _ep0_zlp_needed = (len > 0) && (len < w_length) && ((len % ep0_max_packet) == 0);
    _ep0_tx_ptr = ptr;
    _ep0_tx_remaining = len;
    _ep0_state = Ep0State::dataIn;
    _ep0SendChunk();
  }

  void _ep0SendChunk() {
    uint16_t chunk = _ep0_tx_remaining;
    if (chunk > ep0_max_packet) { chunk = ep0_max_packet; }
    const uint8_t* p = _ep0_tx_ptr;
    _ep0_tx_ptr += chunk;
    _ep0_tx_remaining -= chunk;
    driver.transmit(0, p, chunk);
  }

  void _sendStatusIn() {
    _ep0_state = Ep0State::statusIn;
    driver.transmit(0, nullptr, 0);
  }

  // returns descriptor length (0 = unknown index)
  uint16_t _buildStringDescriptor(uint8_t idx) {
    if (idx == 0) { // LANGID: US English
      _str_buf[0] = 4;
      _str_buf[1] = 0x03;
      _str_buf[2] = 0x09;
      _str_buf[3] = 0x04;
      return 4;
    }
    const char* s = nullptr;
    switch (idx) {
      case 1: s = _conf.manufacturer; break;
      case 2: s = _conf.product; break;
      case 3: s = _conf.serial; break;
      default: break;
    }
    if (!s) { return 0; }
    // ASCII → UTF-16LE, capped to the scratch buffer (31 chars)
    uint16_t pos = 2;
    while (*s && pos + 2 <= sizeof(_str_buf)) {
      _str_buf[pos++] = (uint8_t)*s++;
      _str_buf[pos++] = 0;
    }
    _str_buf[0] = (uint8_t)pos;
    _str_buf[1] = 0x03;
    return pos;
  }
};

}
}
