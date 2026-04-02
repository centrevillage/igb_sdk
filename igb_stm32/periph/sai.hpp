#pragma once

#if defined(STM32H7)

#ifdef STM32_PERIPHGRP_SAI_EXISTS

#include <stddef.h>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

// ============================================================
// SAI enums
// ============================================================

// CR1: MODE[1:0] — audio block mode
enum class SaiBlockMode : uint32_t {
  masterTransmit = 0,
  masterReceive  = 1,
  slaveTransmit  = 2,
  slaveReceive   = 3,
};

// CR1: PRTCFG[1:0] — protocol configuration
enum class SaiProtocol : uint32_t {
  free  = 0,
  spdif = 1,
  ac97  = 2,
};

// CR1: DS[2:0] — data size (field value; 0,1 reserved)
enum class SaiDataSize : uint32_t {
  _8bit  = 2,
  _10bit = 3,
  _16bit = 4,
  _20bit = 5,
  _24bit = 6,
  _32bit = 7,
};

// CR1: SYNCEN[1:0] — synchronization mode
enum class SaiSyncMode : uint32_t {
  async    = 0,
  internal = 1,   // synchronous with the other sub-block (A↔B)
  external = 2,   // synchronous with external SAI via GCR.SYNCIN
};

// CR2: FTH[2:0] — FIFO threshold
enum class SaiFifoThreshold : uint32_t {
  empty        = 0,
  quarter      = 1,
  half         = 2,
  threeQuarter = 3,
  full         = 4,
};

// CR2: COMP[1:0] — companding mode
enum class SaiCompanding : uint32_t {
  none  = 0,
  muLaw = 2,
  aLaw  = 3,
};

// SLOTR: SLOTSZ[1:0] — slot size
enum class SaiSlotSize : uint32_t {
  dataSize = 0,
  _16bit   = 1,
  _32bit   = 2,
};

// SR: FLVL[2:0] — FIFO level
enum class SaiFifoLevel : uint32_t {
  empty      = 0,
  lt_quarter = 1,
  quarter    = 2,
  lt_half    = 3,
  half       = 4,
  lt_full    = 5,
  full       = 6,
};

// GCR: SYNCIN[1:0] — external sync input selection
enum class SaiSyncIn : uint32_t {
  input0 = 0,
  input1 = 1,
  input2 = 2,
  input3 = 3,
};

// GCR: SYNCOUT[1:0] — synchronization output
enum class SaiSyncOut : uint32_t {
  none   = 0,
  blockA = 1,
  blockB = 2,
};

// ============================================================
// SAI conf structs
// ============================================================

struct SaiBlockConf {
  SaiBlockMode mode        = SaiBlockMode::masterTransmit;
  SaiProtocol  protocol    = SaiProtocol::free;
  SaiDataSize  dataSize    = SaiDataSize::_16bit;
  bool lsbFirst            = false;  // false = MSB first (standard)
  bool clockStrobing       = false;  // false = data latched on rising SCK edge
  SaiSyncMode  syncMode    = SaiSyncMode::async;
  bool mono                = false;
  bool outputDrive         = false;  // true = drive output immediately on SAIEN
  bool noDiv               = false;  // true = bypass MCKDIV (SCK = SAI kernel clock)
  uint8_t mckDiv           = 0;      // master clock divider: 6-bit value (0–63)
  bool mckEnable           = false;  // enable MCLK output pin
  bool osr                 = false;  // oversampling: false=256*fs, true=512*fs (master Rx only)
};

struct SaiFrameConf {
  uint8_t frameLength  = 63;    // written to FRL; actual frame = FRL+1 SCK cycles
  uint8_t activeLength = 31;    // written to FSALL; active FS level = FSALL+1 SCK cycles
  bool fsDefinition    = false; // false = FS marks start-of-frame; true = start-of-channel
  bool fsPol           = false; // false = FS active low; true = FS active high
  bool fsOffset        = false; // false = FS on 1st bit of slot 0; true = 1 bit before
};

struct SaiSlotConf {
  uint8_t     firstBitOffset = 0;
  SaiSlotSize slotSize       = SaiSlotSize::dataSize;
  uint8_t     numSlots       = 1;       // written to NBSLOT; actual count = NBSLOT+1
  uint16_t    slotEnable     = 0xFFFF;  // 16-bit bitmap; bit n enables slot n (slots 0–15)
};

// ============================================================
// SaiBlock: SAI sub-block (A or B) register wrapper
//
// Usage example:
//   Sai<SaiType::sai1> sai1;
//   sai1.blockA.initBlock(conf);
//   sai1.blockA.initFrame(frcr);
//   sai1.blockA.initSlot(slotr);
//   sai1.blockA.enable();
// ============================================================

template<uint32_t BLOCK_BASE_ADDR>
struct SaiBlock {
  constexpr static uint32_t addr_CR1   = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, CR1);
  constexpr static uint32_t addr_CR2   = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, CR2);
  constexpr static uint32_t addr_FRCR  = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, FRCR);
  constexpr static uint32_t addr_SLOTR = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, SLOTR);
  constexpr static uint32_t addr_IMR   = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, IMR);
  constexpr static uint32_t addr_SR    = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, SR);
  constexpr static uint32_t addr_CLRFR = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, CLRFR);
  constexpr static uint32_t addr_DR    = BLOCK_BASE_ADDR + offsetof(SAI_Block_TypeDef, DR);

  // ----- CR1 register fields -----
  RegEnum<addr_CR1,  SAI_xCR1_MODE,    SaiBlockMode,  SAI_xCR1_MODE_Pos>    mode;
  RegEnum<addr_CR1,  SAI_xCR1_PRTCFG,  SaiProtocol,  SAI_xCR1_PRTCFG_Pos> protocol;
  RegEnum<addr_CR1,  SAI_xCR1_DS,      SaiDataSize,   SAI_xCR1_DS_Pos>      dataSize;
  RegFlag<addr_CR1,  SAI_xCR1_LSBFIRST>                                     lsbFirst;
  RegFlag<addr_CR1,  SAI_xCR1_CKSTR>                                        clockStrobing;
  RegEnum<addr_CR1,  SAI_xCR1_SYNCEN,  SaiSyncMode,   SAI_xCR1_SYNCEN_Pos> syncMode;
  RegFlag<addr_CR1,  SAI_xCR1_MONO>                           mono;
  RegFlag<addr_CR1,  SAI_xCR1_OUTDRIV>                        outputDrive;
  RegFlag<addr_CR1,  SAI_xCR1_SAIEN>                          enableBlock;
  RegFlag<addr_CR1,  SAI_xCR1_DMAEN>                          dmaEnable;
  RegFlag<addr_CR1,  SAI_xCR1_NODIV>                          noDiv;
  RegValue<addr_CR1, SAI_xCR1_MCKDIV,  SAI_xCR1_MCKDIV_Pos>  mckDiv;
  RegFlag<addr_CR1,  SAI_xCR1_MCKEN>                          mckEnable;
  RegFlag<addr_CR1,  SAI_xCR1_OSR>                            osr;

  // ----- CR2 register fields -----
  RegEnum<addr_CR2,  SAI_xCR2_FTH,     SaiFifoThreshold, SAI_xCR2_FTH_Pos>  fifoThreshold;
  RegFlag<addr_CR2,  SAI_xCR2_FFLUSH>                                        fifoFlush;
  RegFlag<addr_CR2,  SAI_xCR2_TRIS>                                          triState;
  RegFlag<addr_CR2,  SAI_xCR2_MUTE>                                          mute;
  RegFlag<addr_CR2,  SAI_xCR2_MUTEVAL>                                       muteValue;
  RegValue<addr_CR2, SAI_xCR2_MUTECNT, SAI_xCR2_MUTECNT_Pos>                muteCount;
  RegFlag<addr_CR2,  SAI_xCR2_CPL>                                           complement;
  RegEnum<addr_CR2,  SAI_xCR2_COMP,    SaiCompanding,    SAI_xCR2_COMP_Pos>  companding;

  // ----- FRCR register fields -----
  RegValue<addr_FRCR, SAI_xFRCR_FRL,   SAI_xFRCR_FRL_Pos>    frameLength;
  RegValue<addr_FRCR, SAI_xFRCR_FSALL, SAI_xFRCR_FSALL_Pos>  activeLength;
  RegFlag<addr_FRCR,  SAI_xFRCR_FSDEF>                        fsDefinition;
  RegFlag<addr_FRCR,  SAI_xFRCR_FSPOL>                        fsPol;
  RegFlag<addr_FRCR,  SAI_xFRCR_FSOFF>                        fsOffset;

  // ----- SLOTR register fields -----
  RegValue<addr_SLOTR, SAI_xSLOTR_FBOFF,  SAI_xSLOTR_FBOFF_Pos>  firstBitOffset;
  RegEnum<addr_SLOTR,  SAI_xSLOTR_SLOTSZ, SaiSlotSize, SAI_xSLOTR_SLOTSZ_Pos> slotSize;
  RegValue<addr_SLOTR, SAI_xSLOTR_NBSLOT, SAI_xSLOTR_NBSLOT_Pos> numSlots;
  RegValue<addr_SLOTR, SAI_xSLOTR_SLOTEN, SAI_xSLOTR_SLOTEN_Pos> slotEnable;

  // ----- IMR register fields (interrupt mask) -----
  RegFlag<addr_IMR, SAI_xIMR_OVRUDRIE>   enableItOverrun;
  RegFlag<addr_IMR, SAI_xIMR_MUTEDETIE>  enableItMuteDet;
  RegFlag<addr_IMR, SAI_xIMR_WCKCFGIE>   enableItWrongClk;
  RegFlag<addr_IMR, SAI_xIMR_FREQIE>     enableItFifoReq;
  RegFlag<addr_IMR, SAI_xIMR_CNRDYIE>    enableItCodecNotReady;
  RegFlag<addr_IMR, SAI_xIMR_AFSDETIE>   enableItAnticipatedFs;
  RegFlag<addr_IMR, SAI_xIMR_LFSDETIE>   enableItLateFs;

  // ----- SR register fields (status, read-only) -----
  RegFlagRO<addr_SR, SAI_xSR_OVRUDR>   isOverrun;
  RegFlagRO<addr_SR, SAI_xSR_MUTEDET>  isMuteDet;
  RegFlagRO<addr_SR, SAI_xSR_WCKCFG>   isWrongClk;
  RegFlagRO<addr_SR, SAI_xSR_FREQ>     isFifoReq;
  RegFlagRO<addr_SR, SAI_xSR_CNRDY>    isCodecNotReady;
  RegFlagRO<addr_SR, SAI_xSR_AFSDET>   isAnticipatedFs;
  RegFlagRO<addr_SR, SAI_xSR_LFSDET>   isLateFs;
  RegEnumRO<addr_SR, SAI_xSR_FLVL, SaiFifoLevel, SAI_xSR_FLVL_Pos>  fifoLevel;

  // ----- CLRFR register fields (clear flag, write-only) -----
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_COVRUDR>   clearOverrun;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CMUTEDET>  clearMuteDet;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CWCKCFG>   clearWrongClk;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CFREQ>     clearFifoReq;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CCNRDY>    clearCodecNotReady;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CAFSDET>   clearAnticipatedFs;
  RegFlagWO<addr_CLRFR, SAI_xCLRFR_CLFSDET>   clearLateFs;

  // ----- DR register (data, read-write) -----
  Reg<addr_DR>  data;

  // ----- High-level init helpers -----

  IGB_FAST_INLINE void initBlock(const SaiBlockConf& conf) {
    (mode.val(conf.mode)
      | protocol.val(conf.protocol)
      | dataSize.val(conf.dataSize)
      | lsbFirst.val(conf.lsbFirst)
      | clockStrobing.val(conf.clockStrobing)
      | syncMode.val(conf.syncMode)
      | mono.val(conf.mono)
      | outputDrive.val(conf.outputDrive)
      | noDiv.val(conf.noDiv)
      | mckDiv.val(static_cast<uint32_t>(conf.mckDiv))
      | mckEnable.val(conf.mckEnable)
      | osr.val(conf.osr)
    ).update();
  }

  IGB_FAST_INLINE void initFrame(const SaiFrameConf& conf) {
    (frameLength.val(static_cast<uint32_t>(conf.frameLength))
      | activeLength.val(static_cast<uint32_t>(conf.activeLength))
      | fsDefinition.val(conf.fsDefinition)
      | fsPol.val(conf.fsPol)
      | fsOffset.val(conf.fsOffset)
    ).update();
  }

  IGB_FAST_INLINE void initSlot(const SaiSlotConf& conf) {
    (firstBitOffset.val(static_cast<uint32_t>(conf.firstBitOffset))
      | slotSize.val(conf.slotSize)
      | numSlots.val(static_cast<uint32_t>(conf.numSlots))
      | slotEnable.val(static_cast<uint32_t>(conf.slotEnable))
    ).update();
  }

  IGB_FAST_INLINE void enable()    { enableBlock(true); }
  IGB_FAST_INLINE void disable()   { enableBlock(false); }
  IGB_FAST_INLINE void flushFifo() { fifoFlush(true); }

  IGB_FAST_INLINE void     write(uint32_t value) { data(value); }
  IGB_FAST_INLINE uint32_t read()                { return data(); }
};

// ============================================================
// Sai: SAI peripheral-level wrapper
// Manages GCR, PDMCR, PDMDLY, bus clock, and sub-blocks A/B.
//
// Usage example:
//   Sai<SaiType::sai1> sai1;
//   sai1.enableBusClock();
//   sai1.syncOut(SaiSyncOut::blockA);
//   sai1.blockA.initBlock(conf);
//   sai1.blockB.initBlock(conf);
// ============================================================

template<SaiType SAI_TYPE>
struct Sai {
  constexpr static auto type = SAI_TYPE;
  constexpr static auto info = STM32_PERIPH_INFO.sai[to_idx(type)];
  constexpr static auto addr = info.addr;

  constexpr static uint32_t addr_GCR    = addr + offsetof(SAI_TypeDef, GCR);
  constexpr static uint32_t addr_PDMCR  = addr + offsetof(SAI_TypeDef, PDMCR);
  constexpr static uint32_t addr_PDMDLY = addr + offsetof(SAI_TypeDef, PDMDLY);

  // ----- Sub-blocks -----
  SaiBlock<info.block_a.addr> blockA;
  SaiBlock<info.block_b.addr> blockB;

  // ----- GCR register fields -----
  RegEnum<addr_GCR, SAI_GCR_SYNCIN,  SaiSyncIn,  SAI_GCR_SYNCIN_Pos>   syncIn;
  RegEnum<addr_GCR, SAI_GCR_SYNCOUT, SaiSyncOut, SAI_GCR_SYNCOUT_Pos>  syncOut;

  // ----- PDMCR register fields (PDM control; only for SAI1/SAI4 sub-block A) -----
  RegFlag<addr_PDMCR,  SAI_PDMCR_PDMEN>                           pdmEnable;
  RegValue<addr_PDMCR, SAI_PDMCR_MICNBR, SAI_PDMCR_MICNBR_Pos>  micNumber;
  RegFlag<addr_PDMCR,  SAI_PDMCR_CKEN1>                           pdmClkEn1;
  RegFlag<addr_PDMCR,  SAI_PDMCR_CKEN2>                           pdmClkEn2;
  RegFlag<addr_PDMCR,  SAI_PDMCR_CKEN3>                           pdmClkEn3;
  RegFlag<addr_PDMCR,  SAI_PDMCR_CKEN4>                           pdmClkEn4;

  // ----- PDMDLY register fields -----
  RegValue<addr_PDMDLY, SAI_PDMDLY_DLYM1L, SAI_PDMDLY_DLYM1L_Pos> dlyMic1L;
  RegValue<addr_PDMDLY, SAI_PDMDLY_DLYM1R, SAI_PDMDLY_DLYM1R_Pos> dlyMic1R;
  RegValue<addr_PDMDLY, SAI_PDMDLY_DLYM2L, SAI_PDMDLY_DLYM2L_Pos> dlyMic2L;
  RegValue<addr_PDMDLY, SAI_PDMDLY_DLYM2R, SAI_PDMDLY_DLYM2R_Pos> dlyMic2R;

  // ----- Bus clock management -----
  IGB_FAST_INLINE static void enableBusClock() {
    info.bus.enableBusClock();
  }
  IGB_FAST_INLINE static void disableBusClock() {
    info.bus.disableBusClock();
  }
  IGB_FAST_INLINE static void forceResetBusClock() {
    info.bus.forceResetBusClock();
  }
  IGB_FAST_INLINE static void releaseResetBusClock() {
    info.bus.releaseResetBusClock();
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(result.value());
  }

  // TODO: init()
};

} // namespace stm32
} // namespace igb

#endif // STM32_PERIPHGRP_SAI_EXISTS

#endif // STM32H7
