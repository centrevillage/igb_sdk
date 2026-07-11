#pragma once

// USB OTG device-mode driver (Synopsys DWC2 core, STM32H7).
//
// FS embedded PHY / device mode / non-DMA (slave mode FIFO) only.
// Register access follows the usart.hpp idiom: compile-time Reg?? members
// for configuration bits + CMSIS TypeDef pointers for per-EP runtime access.
//
// The protocol layer (EP0 state machine, descriptors) lives above this
// driver (see igb_sdk/usb/usb_cdc_device.hpp) and receives stage events
// through a delegate passed to handleIrq():
//
//   struct Delegate {
//     void onReset();                                  // USBRST
//     void onEnumDone();                               // ENUMDNE (speed fixed)
//     void onSetupStage(const uint8_t* setup8);        // SETUP phase done
//     void onDataOutStage(uint8_t ep, uint16_t count); // OUT xfer complete
//     void onDataInStage(uint8_t ep);                  // IN xfer complete
//     void onSuspend();
//     void onResume();
//     void onSof();
//   };
//
// Init sequence, FIFO handling and interrupt decode are ported from
// stm32h7xx_ll_usb.c / stm32h7xx_hal_pcd.c (proven on the same hardware
// via libDaisy). All TX FIFO writes happen inside handleIrq() (TXFE);
// thread-side code only arms transfers. Callers must guard transmit()/
// receive() against concurrent handleIrq() (e.g. NVIC disable) when
// calling from thread context.

#include <stddef.h>
#include <array>
#include <optional>

#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_stm32/periph/nvic.hpp>

#if defined(USB1_OTG_HS_PERIPH_BASE) && defined(USB2_OTG_FS_PERIPH_BASE)

namespace igb {
namespace stm32 {

enum class UsbOtgType : uint8_t {
  otg1Hs = 0,  // USB1 OTG_HS @ 0x40040000 (FS embedded PHY on PB14/PB15)
  otg2Fs,      // USB2 OTG_FS @ 0x40080000 (FS embedded PHY on PA11/PA12)
};

struct UsbOtgDeviceConf {
  std::optional<GpioPinType> dm_pin_type = std::nullopt;
  std::optional<GpioPinType> dp_pin_type = std::nullopt;
  // AF10 = OTG2 FS pins (PA11/PA12), AF12 = OTG1 HS FS pins (PB14/PB15)
  GpioAf pin_alternate_func = GpioAf::af10;

  // false: VBUS sensing off + B-session valid override (BVALOEN/BVALOVAL).
  // Works regardless of PA9 wiring; the usual choice for custom boards.
  bool vbus_sensing = false;
  bool enable_it_sof = false;

  // FIFO sizes in 32bit words, total <= 0x400 (4KB shared FIFO RAM).
  uint16_t rx_fifo_words = 0x80;
  // index = IN EP number (TXFNUM is mapped 1:1); 0 = unused EP
  std::array<uint16_t, 4> tx_fifo_words = { 0x40, 0x80, 0x10, 0 };

  uint8_t interrupt_priority = 12;
};

#define IGB_USB_OTG ((USB_OTG_GlobalTypeDef*)addr)
#define IGB_USB_OTG_REG_ADDR(member) (addr + offsetof(USB_OTG_GlobalTypeDef, member))
#define IGB_USB_OTG_DEV ((USB_OTG_DeviceTypeDef*)(addr + USB_OTG_DEVICE_BASE))
#define IGB_USB_OTG_DEV_REG_ADDR(member) (addr + USB_OTG_DEVICE_BASE + offsetof(USB_OTG_DeviceTypeDef, member))

template<UsbOtgType USB_TYPE>
struct UsbOtgDevice {
  constexpr static auto type = USB_TYPE;
  constexpr static uint32_t addr =
    (USB_TYPE == UsbOtgType::otg1Hs) ? USB1_OTG_HS_PERIPH_BASE : USB2_OTG_FS_PERIPH_BASE;
  constexpr static IRQn_Type irqn =
    (USB_TYPE == UsbOtgType::otg1Hs) ? OTG_HS_IRQn : OTG_FS_IRQn;
  constexpr static uint32_t bus_enr_bit =
    (USB_TYPE == UsbOtgType::otg1Hs) ? RCC_AHB1ENR_USB1OTGHSEN : RCC_AHB1ENR_USB2OTGFSEN;
  constexpr static uint8_t max_ep_count = 9;

  constexpr static auto addr_GOTGCTL = IGB_USB_OTG_REG_ADDR(GOTGCTL);
  constexpr static auto addr_GAHBCFG = IGB_USB_OTG_REG_ADDR(GAHBCFG);
  constexpr static auto addr_GUSBCFG = IGB_USB_OTG_REG_ADDR(GUSBCFG);
  constexpr static auto addr_GRSTCTL = IGB_USB_OTG_REG_ADDR(GRSTCTL);
  constexpr static auto addr_GINTSTS = IGB_USB_OTG_REG_ADDR(GINTSTS);
  constexpr static auto addr_GINTMSK = IGB_USB_OTG_REG_ADDR(GINTMSK);
  constexpr static auto addr_GRXSTSP = IGB_USB_OTG_REG_ADDR(GRXSTSP);
  constexpr static auto addr_GRXFSIZ = IGB_USB_OTG_REG_ADDR(GRXFSIZ);
  constexpr static auto addr_DIEPTXF0 = IGB_USB_OTG_REG_ADDR(DIEPTXF0_HNPTXFSIZ);
  constexpr static auto addr_GCCFG = IGB_USB_OTG_REG_ADDR(GCCFG);
  constexpr static auto addr_DCFG = IGB_USB_OTG_DEV_REG_ADDR(DCFG);
  constexpr static auto addr_DCTL = IGB_USB_OTG_DEV_REG_ADDR(DCTL);
  constexpr static auto addr_DSTS = IGB_USB_OTG_DEV_REG_ADDR(DSTS);
  constexpr static auto addr_DIEPMSK = IGB_USB_OTG_DEV_REG_ADDR(DIEPMSK);
  constexpr static auto addr_DOEPMSK = IGB_USB_OTG_DEV_REG_ADDR(DOEPMSK);
  constexpr static auto addr_DAINT = IGB_USB_OTG_DEV_REG_ADDR(DAINT);
  constexpr static auto addr_DAINTMSK = IGB_USB_OTG_DEV_REG_ADDR(DAINTMSK);
  constexpr static auto addr_DIEPEMPMSK = IGB_USB_OTG_DEV_REG_ADDR(DIEPEMPMSK);

  RegFlag<addr_GAHBCFG, USB_OTG_GAHBCFG_GINT> enableGlobalIt;
  RegFlag<addr_GUSBCFG, USB_OTG_GUSBCFG_PHYSEL> selectFsPhy;
  RegFlag<addr_GUSBCFG, USB_OTG_GUSBCFG_FDMOD> forceDeviceMode;
  RegFlag<addr_GUSBCFG, USB_OTG_GUSBCFG_FHMOD> forceHostMode;
  RegValue<addr_GUSBCFG, USB_OTG_GUSBCFG_TRDT_Msk, USB_OTG_GUSBCFG_TRDT_Pos> turnaroundTime;
  RegFlag<addr_GCCFG, USB_OTG_GCCFG_PWRDWN> powerUpTransceiver; // 1 = powered up
  RegFlag<addr_GCCFG, USB_OTG_GCCFG_VBDEN> enableVbusDetect;
  RegFlag<addr_GOTGCTL, USB_OTG_GOTGCTL_BVALOEN> enableBValidOverride;
  RegFlag<addr_GOTGCTL, USB_OTG_GOTGCTL_BVALOVAL> bValidOverrideValue;
  RegFlag<addr_DCTL, USB_OTG_DCTL_SDIS> softDisconnect;
  RegValue<addr_DCFG, USB_OTG_DCFG_DAD_Msk, USB_OTG_DCFG_DAD_Pos> deviceAddress;
  RegValue<addr_DCFG, USB_OTG_DCFG_DSPD_Msk, USB_OTG_DCFG_DSPD_Pos> deviceSpeed;
  Reg<addr_GINTSTS> reg_GINTSTS;
  Reg<addr_GINTMSK> reg_GINTMSK;
  RegRO<addr_GRXSTSP> reg_GRXSTSP; // pop-on-read
  Reg<addr_GRXFSIZ> reg_GRXFSIZ;
  Reg<addr_DIEPTXF0> reg_DIEPTXF0;
  Reg<addr_DAINTMSK> reg_DAINTMSK;
  Reg<addr_DIEPMSK> reg_DIEPMSK;
  Reg<addr_DOEPMSK> reg_DOEPMSK;
  Reg<addr_DIEPEMPMSK> reg_DIEPEMPMSK;

  IGB_FAST_INLINE static USB_OTG_INEndpointTypeDef* p_in_ep(uint8_t ep) {
    return (USB_OTG_INEndpointTypeDef*)(addr + USB_OTG_IN_ENDPOINT_BASE + (uint32_t)ep * USB_OTG_EP_REG_SIZE);
  }
  IGB_FAST_INLINE static USB_OTG_OUTEndpointTypeDef* p_out_ep(uint8_t ep) {
    return (USB_OTG_OUTEndpointTypeDef*)(addr + USB_OTG_OUT_ENDPOINT_BASE + (uint32_t)ep * USB_OTG_EP_REG_SIZE);
  }
  IGB_FAST_INLINE static volatile uint32_t* p_fifo(uint8_t ep) {
    return (volatile uint32_t*)(addr + USB_OTG_FIFO_BASE + (uint32_t)ep * USB_OTG_FIFO_SIZE);
  }
  IGB_FAST_INLINE static volatile uint32_t* p_pcgcctl() {
    return (volatile uint32_t*)(addr + USB_OTG_PCGCCTL_BASE);
  }

  // GRXSTSP.PKTSTS (device mode)
  constexpr static uint32_t pktsts_data_updt = 2;
  constexpr static uint32_t pktsts_setup_updt = 6;

  // EPTYP encoding == descriptor bmAttributes: 0=control,1=iso,2=bulk,3=interrupt
  struct EpState {
    uint8_t* buf = nullptr;
    uint16_t len = 0;         // requested transfer length
    uint16_t count = 0;       // bytes transferred so far
    uint16_t max_packet = 0;
    uint8_t ep_type = 0;
    bool active = false;
  };
  std::array<EpState, max_ep_count> in_eps;
  std::array<EpState, max_ep_count> out_eps;
  alignas(4) uint8_t setup_buf[8] = {};
  uint8_t _dev_ep_count = 9;

  IGB_FAST_INLINE void enableBusClock() {
    RCC->AHB1ENR |= bus_enr_bit;
    __IO auto tmp IGB_UNUSED = RCC->AHB1ENR; // delay until clock enabled
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type, GpioAf af_idx) {
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::low);
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(af_idx);
  }

  // ----- FIFO -----

  IGB_FAST_INLINE static void writePacket(uint8_t fifo_num, const uint8_t* src, uint16_t len) {
    volatile uint32_t* fifo = p_fifo(fifo_num);
    uint32_t count32 = ((uint32_t)len + 3) / 4;
    for (uint32_t i = 0; i < count32; ++i) {
      uint32_t v;
      __builtin_memcpy(&v, src, 4); // unaligned-safe
      *fifo = v;
      src += 4;
    }
  }

  IGB_FAST_INLINE static void readPacket(uint8_t* dest, uint16_t len) {
    volatile uint32_t* fifo = p_fifo(0);
    uint32_t count32 = (uint32_t)len >> 2;
    for (uint32_t i = 0; i < count32; ++i) {
      uint32_t v = *fifo;
      __builtin_memcpy(dest, &v, 4);
      dest += 4;
    }
    uint16_t remain = len & 3;
    if (remain) {
      uint32_t v = *fifo;
      for (uint16_t i = 0; i < remain; ++i) {
        dest[i] = (uint8_t)(v >> (8 * i));
      }
    }
  }

  // num = fifo number, 0x10 = all
  bool flushTxFifo(uint32_t num) {
    if (!_waitAhbIdle()) { return false; }
    IGB_USB_OTG->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (num << USB_OTG_GRSTCTL_TXFNUM_Pos);
    uint32_t count = 0;
    while (IGB_USB_OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) {
      if (++count > 200000UL) { return false; }
    }
    return true;
  }

  bool flushRxFifo() {
    if (!_waitAhbIdle()) { return false; }
    IGB_USB_OTG->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    uint32_t count = 0;
    while (IGB_USB_OTG->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) {
      if (++count > 200000UL) { return false; }
    }
    return true;
  }

  // ----- EP operations -----

  IGB_FAST_INLINE void setAddress(uint8_t address) {
    deviceAddress(address);
  }

  void openInEp(uint8_t ep, uint8_t ep_type, uint16_t max_packet) {
    auto& e = in_eps[ep];
    e.max_packet = max_packet;
    e.ep_type = ep_type;
    e.active = false;
    IGB_USB_OTG_DEV->DAINTMSK |= (1UL << ep) & USB_OTG_DAINTMSK_IEPM;
    auto* in = p_in_ep(ep);
    if (!(in->DIEPCTL & USB_OTG_DIEPCTL_USBAEP)) {
      in->DIEPCTL |= ((uint32_t)max_packet & USB_OTG_DIEPCTL_MPSIZ) |
                     ((uint32_t)ep_type << 18) | ((uint32_t)ep << 22) |
                     USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                     USB_OTG_DIEPCTL_USBAEP;
    }
  }

  void openOutEp(uint8_t ep, uint8_t ep_type, uint16_t max_packet) {
    auto& e = out_eps[ep];
    e.max_packet = max_packet;
    e.ep_type = ep_type;
    e.active = false;
    IGB_USB_OTG_DEV->DAINTMSK |= ((1UL << ep) << 16) & USB_OTG_DAINTMSK_OEPM;
    auto* out = p_out_ep(ep);
    if (!(out->DOEPCTL & USB_OTG_DOEPCTL_USBAEP)) {
      out->DOEPCTL |= ((uint32_t)max_packet & USB_OTG_DOEPCTL_MPSIZ) |
                      ((uint32_t)ep_type << 18) |
                      USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                      USB_OTG_DOEPCTL_USBAEP;
    }
  }

  // Arm an IN transfer. Data is pushed into the FIFO from handleIrq()
  // (TXFE), so buf must stay valid until onDataInStage(ep). EP0 sends at
  // most one packet per call (chunking is up to the protocol layer).
  void transmit(uint8_t ep, const uint8_t* buf, uint16_t len) {
    auto& e = in_eps[ep];
    if (ep == 0 && len > e.max_packet) { len = e.max_packet; }
    e.buf = (uint8_t*)buf;
    e.len = len;
    e.count = 0;
    e.active = true;
    auto* in = p_in_ep(ep);
    uint32_t tsiz = 0;
    if (len == 0) {
      tsiz = (1UL << 19); // PKTCNT=1, XFRSIZ=0 (ZLP)
    } else {
      uint32_t pktcnt = (ep == 0) ? 1 : (((uint32_t)len + e.max_packet - 1) / e.max_packet);
      tsiz = (pktcnt << 19) | len;
    }
    in->DIEPTSIZ = tsiz;
    in->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    if (len > 0) {
      IGB_USB_OTG_DEV->DIEPEMPMSK |= (1UL << ep);
    }
  }

  // Arm an OUT transfer into buf (len 0 = status/ZLP reception).
  void receive(uint8_t ep, uint8_t* buf, uint16_t len) {
    auto& e = out_eps[ep];
    e.buf = buf;
    e.count = 0;
    e.active = true;
    auto* out = p_out_ep(ep);
    uint32_t tsiz;
    if (ep == 0 || len == 0) {
      e.len = (ep == 0) ? e.max_packet : 0;
      tsiz = (1UL << 19) | e.max_packet; // PKTCNT=1, XFRSIZ=maxpacket
    } else {
      uint32_t pktcnt = ((uint32_t)len + e.max_packet - 1) / e.max_packet;
      e.len = len;
      tsiz = (pktcnt << 19) | (pktcnt * e.max_packet);
    }
    out->DOEPTSIZ = tsiz;
    out->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
  }

  // Re-arm EP0 OUT for the next SETUP packet (no EPENA needed in slave mode).
  IGB_FAST_INLINE void ep0StartSetup() {
    auto* out = p_out_ep(0);
    out->DOEPTSIZ = (1UL << 19) | (3U * 8U) | USB_OTG_DOEPTSIZ_STUPCNT;
  }

  void stallInEp(uint8_t ep) {
    auto* in = p_in_ep(ep);
    if (!(in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && ep != 0) {
      in->DIEPCTL &= ~USB_OTG_DIEPCTL_EPDIS;
    }
    in->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    in_eps[ep].active = false;
  }

  void stallOutEp(uint8_t ep) {
    auto* out = p_out_ep(ep);
    if (!(out->DOEPCTL & USB_OTG_DOEPCTL_EPENA) && ep != 0) {
      out->DOEPCTL &= ~USB_OTG_DOEPCTL_EPDIS;
    }
    out->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    out_eps[ep].active = false;
  }

  // Protocol stall on EP0 (both directions). The core clears EP0 STALL
  // automatically when the next SETUP arrives.
  void stallEp0() {
    stallInEp(0);
    stallOutEp(0);
    ep0StartSetup();
  }

  void clearStallInEp(uint8_t ep) {
    auto* in = p_in_ep(ep);
    in->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    uint8_t t = in_eps[ep].ep_type;
    if (t == 2 || t == 3) {
      in->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    }
  }

  void clearStallOutEp(uint8_t ep) {
    auto* out = p_out_ep(ep);
    out->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    uint8_t t = out_eps[ep].ep_type;
    if (t == 2 || t == 3) {
      out->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
    }
  }

  // ----- init / connect -----

  bool init(auto&& conf) {
    // H7: USB transceivers need the VDD33USB voltage detector.
    PWR->CR3 |= PWR_CR3_USB33DEN;

    enableBusClock();

    if (conf.dm_pin_type) { prepareGpio(conf.dm_pin_type.value(), conf.pin_alternate_func); }
    if (conf.dp_pin_type) { prepareGpio(conf.dp_pin_type.value(), conf.pin_alternate_func); }

    NvicCtrl::setPriority(irqn, conf.interrupt_priority);
    NvicCtrl::enable(irqn);

    enableGlobalIt(false);

    // Core init: FS embedded PHY → core soft reset → transceiver power up
    selectFsPhy(true);
    if (!_coreReset()) { return false; }
    powerUpTransceiver(true);

    // Force device mode (CMOD: 0=device). Takes up to ~25ms.
    forceHostMode(false);
    forceDeviceMode(true);
    {
      uint32_t count = 0;
      while (reg_GINTSTS() & USB_OTG_GINTSTS_CMOD) {
        if (++count > 20000000UL) { return false; }
      }
    }

    for (uint32_t i = 0; i < 15; ++i) {
      IGB_USB_OTG->DIEPTXF[i] = 0;
    }

    if (conf.vbus_sensing) {
      enableVbusDetect(true);
    } else {
      softDisconnect(true);
      enableVbusDetect(false);
      enableBValidOverride(true);
      bValidOverrideValue(true);
    }

    // Restart PHY clock
    *p_pcgcctl() = 0;

    deviceSpeed(3); // full speed, internal PHY

    if (!flushTxFifo(0x10)) { return false; }
    if (!flushRxFifo()) { return false; }

    reg_DIEPMSK(0);
    reg_DOEPMSK(0);
    reg_DAINTMSK(0);

    for (uint8_t i = 0; i < _dev_ep_count; ++i) {
      auto* in = p_in_ep(i);
      if (in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        in->DIEPCTL = (i == 0) ? USB_OTG_DIEPCTL_SNAK
                               : (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
      } else {
        in->DIEPCTL = 0;
      }
      in->DIEPTSIZ = 0;
      in->DIEPINT = 0xFB7FU;

      auto* out = p_out_ep(i);
      if (out->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
        out->DOEPCTL = (i == 0) ? USB_OTG_DOEPCTL_SNAK
                                : (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
      } else {
        out->DOEPCTL = 0;
      }
      out->DOEPTSIZ = 0;
      out->DOEPINT = 0xFB7FU;
    }

    reg_GINTMSK(0);
    reg_GINTSTS(0xBFFFFFFFU);

    uint32_t mask = USB_OTG_GINTMSK_RXFLVLM |
                    USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                    USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                    USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IISOIXFRM |
                    USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;
    if (conf.enable_it_sof) {
      mask |= USB_OTG_GINTMSK_SOFM;
    }
    if (conf.vbus_sensing) {
      mask |= USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;
    }
    reg_GINTMSK(mask);

    // FIFO layout: RX first, then per-IN-EP TX FIFOs
    reg_GRXFSIZ(conf.rx_fifo_words);
    uint32_t offset = conf.rx_fifo_words;
    reg_DIEPTXF0(((uint32_t)conf.tx_fifo_words[0] << 16) | offset);
    offset += conf.tx_fifo_words[0];
    for (uint32_t i = 1; i < conf.tx_fifo_words.size(); ++i) {
      if (conf.tx_fifo_words[i] == 0) { continue; }
      IGB_USB_OTG->DIEPTXF[i - 1] = ((uint32_t)conf.tx_fifo_words[i] << 16) | offset;
      offset += conf.tx_fifo_words[i];
    }

    softDisconnect(true);
    enableGlobalIt(true);
    return true;
  }

  IGB_FAST_INLINE void connect() {
    *p_pcgcctl() &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    softDisconnect(false);
  }

  IGB_FAST_INLINE void disconnect() {
    *p_pcgcctl() &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    softDisconnect(true);
  }

  // ----- interrupt handling -----

  template<typename DelegateT>
  void handleIrq(DelegateT& d) {
    if (reg_GINTSTS() & USB_OTG_GINTSTS_CMOD) { return; } // host mode: not ours
    uint32_t ints = reg_GINTSTS() & reg_GINTMSK();
    if (!ints) { return; }

    if (ints & USB_OTG_GINTSTS_MMIS) {
      reg_GINTSTS(USB_OTG_GINTSTS_MMIS);
    }

    if (ints & USB_OTG_GINTSTS_RXFLVL) {
      IGB_USB_OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
      uint32_t sts = reg_GRXSTSP();
      uint8_t ep = sts & USB_OTG_GRXSTSP_EPNUM;
      uint16_t bcnt = (sts & USB_OTG_GRXSTSP_BCNT) >> 4;
      uint32_t pktsts = (sts & USB_OTG_GRXSTSP_PKTSTS) >> 17;
      if (pktsts == pktsts_data_updt) {
        if (bcnt) {
          auto& e = out_eps[ep];
          if (e.buf) {
            readPacket(e.buf + e.count, bcnt);
            e.count += bcnt;
          } else {
            // 未アーム EP へのデータ: FIFO を読み捨てて詰まりを防ぐ
            uint32_t words = ((uint32_t)bcnt + 3) / 4;
            while (words--) { (void)*p_fifo(0); }
          }
        }
      } else if (pktsts == pktsts_setup_updt) {
        readPacket(setup_buf, 8);
      }
      // other pktsts (xfer complete / setup complete / global out nak): no data
      IGB_USB_OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    }

    if (ints & USB_OTG_GINTSTS_OEPINT) {
      uint32_t ep_ints = ((IGB_USB_OTG_DEV->DAINT & IGB_USB_OTG_DEV->DAINTMSK) >> 16) & 0xFFFFU;
      uint8_t ep = 0;
      while (ep_ints) {
        if (ep_ints & 1) {
          auto* out = p_out_ep(ep);
          uint32_t epint = out->DOEPINT & reg_DOEPMSK();
          if (epint & USB_OTG_DOEPINT_XFRC) {
            out->DOEPINT = USB_OTG_DOEPINT_XFRC;
            // H7 core (SNPSiD 310A): STPKTRX marks "setup packet received",
            // not a data-out completion — clear and skip the stage callback.
            if (out->DOEPINT & USB_OTG_DOEPINT_STPKTRX) {
              out->DOEPINT = USB_OTG_DOEPINT_STPKTRX;
            } else {
              if (out->DOEPINT & USB_OTG_DOEPINT_OTEPSPR) {
                out->DOEPINT = USB_OTG_DOEPINT_OTEPSPR;
              }
              auto& e = out_eps[ep];
              e.active = false;
              d.onDataOutStage(ep, e.count);
            }
          }
          if (epint & USB_OTG_DOEPINT_STUP) {
            out->DOEPINT = USB_OTG_DOEPINT_STUP;
            if (out->DOEPINT & USB_OTG_DOEPINT_STPKTRX) {
              out->DOEPINT = USB_OTG_DOEPINT_STPKTRX;
            }
            d.onSetupStage(setup_buf);
          }
          if (epint & USB_OTG_DOEPINT_OTEPDIS) {
            out->DOEPINT = USB_OTG_DOEPINT_OTEPDIS;
          }
          if (epint & USB_OTG_DOEPINT_EPDISD) {
            out->DOEPINT = USB_OTG_DOEPINT_EPDISD;
          }
          if (epint & USB_OTG_DOEPINT_OTEPSPR) {
            out->DOEPINT = USB_OTG_DOEPINT_OTEPSPR;
          }
          if (epint & USB_OTG_DOEPINT_NAK) {
            out->DOEPINT = USB_OTG_DOEPINT_NAK;
          }
        }
        ++ep;
        ep_ints >>= 1;
      }
    }

    if (ints & USB_OTG_GINTSTS_IEPINT) {
      uint32_t ep_ints = IGB_USB_OTG_DEV->DAINT & IGB_USB_OTG_DEV->DAINTMSK & 0xFFFFU;
      uint8_t ep = 0;
      while (ep_ints) {
        if (ep_ints & 1) {
          auto* in = p_in_ep(ep);
          uint32_t msk = reg_DIEPMSK() |
                         (((reg_DIEPEMPMSK() >> ep) & 1U) << 7); // TXFE
          uint32_t epint = in->DIEPINT & msk;
          if (epint & USB_OTG_DIEPINT_XFRC) {
            IGB_USB_OTG_DEV->DIEPEMPMSK &= ~(1UL << ep);
            in->DIEPINT = USB_OTG_DIEPINT_XFRC;
            in_eps[ep].active = false;
            d.onDataInStage(ep);
          }
          if (epint & USB_OTG_DIEPINT_TOC) {
            in->DIEPINT = USB_OTG_DIEPINT_TOC;
          }
          if (epint & USB_OTG_DIEPINT_ITTXFE) {
            in->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
          }
          if (epint & USB_OTG_DIEPINT_INEPNE) {
            in->DIEPINT = USB_OTG_DIEPINT_INEPNE;
          }
          if (epint & USB_OTG_DIEPINT_EPDISD) {
            flushTxFifo(ep);
            in->DIEPINT = USB_OTG_DIEPINT_EPDISD;
          }
          if (epint & USB_OTG_DIEPINT_TXFE) {
            _writeEmptyTxFifo(ep);
          }
        }
        ++ep;
        ep_ints >>= 1;
      }
    }

    if (ints & USB_OTG_GINTSTS_WKUINT) {
      IGB_USB_OTG_DEV->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      d.onResume();
      reg_GINTSTS(USB_OTG_GINTSTS_WKUINT);
    }

    if (ints & USB_OTG_GINTSTS_USBSUSP) {
      if (IGB_USB_OTG_DEV->DSTS & USB_OTG_DSTS_SUSPSTS) {
        d.onSuspend();
      }
      reg_GINTSTS(USB_OTG_GINTSTS_USBSUSP);
    }

    if (ints & USB_OTG_GINTSTS_USBRST) {
      IGB_USB_OTG_DEV->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      flushTxFifo(0x10);
      for (uint8_t i = 0; i < _dev_ep_count; ++i) {
        auto* in = p_in_ep(i);
        in->DIEPINT = 0xFB7FU;
        in->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        auto* out = p_out_ep(i);
        out->DOEPINT = 0xFB7FU;
        out->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        out->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        in_eps[i].active = false;
        out_eps[i].active = false;
      }
      IGB_USB_OTG_DEV->DAINTMSK |= 0x10001U;
      IGB_USB_OTG_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM |
                                  USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM |
                                  USB_OTG_DOEPMSK_NAKM;
      IGB_USB_OTG_DEV->DIEPMSK |= USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM |
                                  USB_OTG_DIEPMSK_EPDM;
      deviceAddress(0);
      ep0StartSetup();
      d.onReset();
      reg_GINTSTS(USB_OTG_GINTSTS_USBRST);
    }

    if (ints & USB_OTG_GINTSTS_ENUMDNE) {
      // EP0 max packet 64B (MPSIZ=00) + clear global IN NAK
      p_in_ep(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
      IGB_USB_OTG_DEV->DCTL |= USB_OTG_DCTL_CGINAK;
      in_eps[0].max_packet = 64;
      out_eps[0].max_packet = 64;
      // FS with HCLK >= 32MHz → TRDT = 6 (RM0433)
      turnaroundTime(6);
      d.onEnumDone();
      reg_GINTSTS(USB_OTG_GINTSTS_ENUMDNE);
    }

    if (ints & USB_OTG_GINTSTS_SOF) {
      d.onSof();
      reg_GINTSTS(USB_OTG_GINTSTS_SOF);
    }

    if (ints & USB_OTG_GINTSTS_IISOIXFR) {
      reg_GINTSTS(USB_OTG_GINTSTS_IISOIXFR);
    }
    if (ints & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) {
      reg_GINTSTS(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
    }
    if (ints & USB_OTG_GINTSTS_SRQINT) {
      reg_GINTSTS(USB_OTG_GINTSTS_SRQINT);
    }
    if (ints & USB_OTG_GINTSTS_OTGINT) {
      uint32_t otg = IGB_USB_OTG->GOTGINT;
      IGB_USB_OTG->GOTGINT = otg;
    }
  }

  // ----- internal -----

  bool _waitAhbIdle() {
    uint32_t count = 0;
    while (!(IGB_USB_OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)) {
      if (++count > 200000UL) { return false; }
    }
    return true;
  }

  bool _coreReset() {
    if (!_waitAhbIdle()) { return false; }
    IGB_USB_OTG->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    uint32_t count = 0;
    while (IGB_USB_OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST) {
      if (++count > 200000UL) { return false; }
    }
    return true;
  }

  // TXFE: keep writing packets while the FIFO has room (HAL same logic)
  void _writeEmptyTxFifo(uint8_t ep) {
    auto& e = in_eps[ep];
    if (e.count > e.len) { return; }
    auto* in = p_in_ep(ep);
    uint16_t len = e.len - e.count;
    if (len > e.max_packet) { len = e.max_packet; }
    uint32_t len32 = ((uint32_t)len + 3) / 4;
    while ((in->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32 &&
           e.count < e.len && e.len != 0) {
      len = e.len - e.count;
      if (len > e.max_packet) { len = e.max_packet; }
      len32 = ((uint32_t)len + 3) / 4;
      writePacket(ep, e.buf + e.count, len);
      e.count += len;
    }
    if (e.len <= e.count) {
      IGB_USB_OTG_DEV->DIEPEMPMSK &= ~(1UL << ep);
    }
  }
};

#undef IGB_USB_OTG_DEV_REG_ADDR
#undef IGB_USB_OTG_DEV
#undef IGB_USB_OTG_REG_ADDR
#undef IGB_USB_OTG

}
}

#endif // USB1_OTG_HS_PERIPH_BASE && USB2_OTG_FS_PERIPH_BASE
