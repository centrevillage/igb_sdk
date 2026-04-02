#pragma once

#if defined(STM32H7)

#ifdef STM32_PERIPH_QUADSPI_EXISTS

#include <stddef.h>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

// ============================================================
// QUADSPI enums
// ============================================================

// CCR: FMODE[1:0] -- functional mode
enum class QspiFunctionalMode : uint32_t {
  indirectWrite = 0,
  indirectRead  = 1,
  autoPolling   = 2,
  memoryMapped  = 3,
};

// CCR: IMODE/ADMODE/ABMODE/DMODE -- line width
enum class QspiLineMode : uint32_t {
  none   = 0,
  single = 1,
  dual   = 2,
  quad   = 3,
};

// CCR: ADSIZE[1:0] -- address size
enum class QspiAddressSize : uint32_t {
  _8bit  = 0,
  _16bit = 1,
  _24bit = 2,
  _32bit = 3,
};

// CCR: ABSIZE[1:0] -- alternate bytes size
enum class QspiAlternateBytesSize : uint32_t {
  _8bit  = 0,
  _16bit = 1,
  _24bit = 2,
  _32bit = 3,
};

// CR: PMM -- polling match mode
enum class QspiPollingMatchMode : uint32_t {
  andMatch = 0,
  orMatch  = 1,
};

// DCR: CKMODE -- clock mode
enum class QspiClockMode : uint32_t {
  mode0 = 0,  // CLK low while NCS high
  mode3 = 1,  // CLK high while NCS high
};

// DCR: CSHT[2:0] -- chip select high time
enum class QspiCsHighTime : uint32_t {
  _1cycle = 0,
  _2cycle = 1,
  _3cycle = 2,
  _4cycle = 3,
  _5cycle = 4,
  _6cycle = 5,
  _7cycle = 6,
  _8cycle = 7,
};

// CR: FSEL -- flash memory selection (single-flash mode)
enum class QspiFlashSelect : uint32_t {
  flash1 = 0,
  flash2 = 1,
};

// ============================================================
// QUADSPI conf structs
// ============================================================

struct QspiConf {
  uint8_t prescaler                   = 0;    // 0-255: F_CLK = F_ker_ck / (prescaler+1)
  QspiPollingMatchMode pollMatchMode  = QspiPollingMatchMode::andMatch;
  bool autoPollStop                   = false;
  uint8_t fifoThreshold               = 0;    // 0-31: threshold is (value+1) bytes
  QspiFlashSelect flashSelect         = QspiFlashSelect::flash1;
  bool dualFlashMode                  = false;
  bool sampleShift                    = false; // must be false in DDR mode
  bool timeoutCounterEnable           = false; // memory-mapped mode only
  bool enableItTransferError          = false;
  bool enableItTransferComplete       = false;
  bool enableItFifoThreshold          = false;
  bool enableItStatusMatch            = false;
  bool enableItTimeout                = false;
};

struct QspiDeviceConf {
  uint8_t flashSize                   = 0;    // 0-31: capacity = 2^(flashSize+1) bytes
  QspiCsHighTime csHighTime           = QspiCsHighTime::_1cycle;
  QspiClockMode clockMode             = QspiClockMode::mode0;
};

struct QspiCommandConf {
  uint8_t instruction                 = 0;
  QspiLineMode instructionMode        = QspiLineMode::none;
  QspiLineMode addressMode            = QspiLineMode::none;
  QspiAddressSize addressSize         = QspiAddressSize::_8bit;
  QspiLineMode alternateBytesMode     = QspiLineMode::none;
  QspiAlternateBytesSize alternateBytesSize = QspiAlternateBytesSize::_8bit;
  uint8_t dummyCycles                 = 0;    // 0-31 CLK cycles
  QspiLineMode dataMode               = QspiLineMode::none;
  QspiFunctionalMode functionalMode   = QspiFunctionalMode::indirectWrite;
  bool sendInstructionOnce            = false;
  bool ddrHoldHalfCycle               = false;
  bool ddrMode                        = false;
};

struct QspiAutoPollingConf {
  uint32_t mask     = 0;
  uint32_t match    = 0;
  uint16_t interval = 0;  // CLK cycles between reads
};

// ============================================================
// Qspi: QUADSPI peripheral register wrapper
// ============================================================

struct Qspi {
  constexpr static auto info = STM32_PERIPH_INFO.quadspi[0];

  constexpr static uint32_t addr_CR    = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, CR);
  constexpr static uint32_t addr_DCR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, DCR);
  constexpr static uint32_t addr_SR    = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, SR);
  constexpr static uint32_t addr_FCR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, FCR);
  constexpr static uint32_t addr_DLR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, DLR);
  constexpr static uint32_t addr_CCR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, CCR);
  constexpr static uint32_t addr_AR    = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, AR);
  constexpr static uint32_t addr_ABR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, ABR);
  constexpr static uint32_t addr_DR    = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, DR);
  constexpr static uint32_t addr_PSMKR = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, PSMKR);
  constexpr static uint32_t addr_PSMAR = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, PSMAR);
  constexpr static uint32_t addr_PIR   = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, PIR);
  constexpr static uint32_t addr_LPTR  = QSPI_R_BASE + offsetof(QUADSPI_TypeDef, LPTR);

  // ----- CR register fields -----
  RegFlag<addr_CR, QUADSPI_CR_EN>               enable;
  RegFlag<addr_CR, QUADSPI_CR_ABORT>            abort;
  RegFlag<addr_CR, QUADSPI_CR_TCEN>             timeoutCounterEnable;
  RegFlag<addr_CR, QUADSPI_CR_SSHIFT>           sampleShift;
  RegEnum<addr_CR, QUADSPI_CR_DFM,
          QspiFlashSelect, QUADSPI_CR_DFM_Pos>  dualFlashMode; // note: DFM is a flag, not FlashSelect
  RegFlag<addr_CR, QUADSPI_CR_DFM>              enableDualFlash;
  RegEnum<addr_CR, QUADSPI_CR_FSEL,
          QspiFlashSelect, QUADSPI_CR_FSEL_Pos> flashSelect;
  RegValue<addr_CR, QUADSPI_CR_FTHRES,
           QUADSPI_CR_FTHRES_Pos>               fifoThreshold;
  RegFlag<addr_CR, QUADSPI_CR_TEIE>             enableItTransferError;
  RegFlag<addr_CR, QUADSPI_CR_TCIE>             enableItTransferComplete;
  RegFlag<addr_CR, QUADSPI_CR_FTIE>             enableItFifoThreshold;
  RegFlag<addr_CR, QUADSPI_CR_SMIE>             enableItStatusMatch;
  RegFlag<addr_CR, QUADSPI_CR_TOIE>             enableItTimeout;
  RegFlag<addr_CR, QUADSPI_CR_APMS>             autoPollStop;
  RegEnum<addr_CR, QUADSPI_CR_PMM,
          QspiPollingMatchMode, QUADSPI_CR_PMM_Pos> pollMatchMode;
  RegValue<addr_CR, QUADSPI_CR_PRESCALER,
           QUADSPI_CR_PRESCALER_Pos>            prescaler;

  // ----- DCR register fields -----
  RegFlag<addr_DCR, QUADSPI_DCR_CKMODE>         clockMode3;
  RegEnum<addr_DCR, QUADSPI_DCR_CSHT,
          QspiCsHighTime, QUADSPI_DCR_CSHT_Pos> csHighTime;
  RegValue<addr_DCR, QUADSPI_DCR_FSIZE,
           QUADSPI_DCR_FSIZE_Pos>               flashSize;

  // ----- SR register fields (read-only) -----
  RegFlagRO<addr_SR, QUADSPI_SR_TEF>            isTransferError;
  RegFlagRO<addr_SR, QUADSPI_SR_TCF>            isTransferComplete;
  RegFlagRO<addr_SR, QUADSPI_SR_FTF>            isFifoThreshold;
  RegFlagRO<addr_SR, QUADSPI_SR_SMF>            isStatusMatch;
  RegFlagRO<addr_SR, QUADSPI_SR_TOF>            isTimeout;
  RegFlagRO<addr_SR, QUADSPI_SR_BUSY>           isBusy;
  RegValueRO<addr_SR, QUADSPI_SR_FLEVEL,
             QUADSPI_SR_FLEVEL_Pos>             fifoLevel;

  // ----- FCR register fields (write-only, clear flags) -----
  RegFlagWO<addr_FCR, QUADSPI_FCR_CTEF>         clearTransferError;
  RegFlagWO<addr_FCR, QUADSPI_FCR_CTCF>         clearTransferComplete;
  RegFlagWO<addr_FCR, QUADSPI_FCR_CSMF>         clearStatusMatch;
  RegFlagWO<addr_FCR, QUADSPI_FCR_CTOF>         clearTimeout;

  // ----- DLR register (data length) -----
  Reg<addr_DLR>                                  dataLength;

  // ----- CCR register fields -----
  RegValue<addr_CCR, QUADSPI_CCR_INSTRUCTION,
           QUADSPI_CCR_INSTRUCTION_Pos>         instruction;
  RegEnum<addr_CCR, QUADSPI_CCR_IMODE,
          QspiLineMode, QUADSPI_CCR_IMODE_Pos>  instructionMode;
  RegEnum<addr_CCR, QUADSPI_CCR_ADMODE,
          QspiLineMode, QUADSPI_CCR_ADMODE_Pos> addressMode;
  RegEnum<addr_CCR, QUADSPI_CCR_ADSIZE,
          QspiAddressSize, QUADSPI_CCR_ADSIZE_Pos> addressSize;
  RegEnum<addr_CCR, QUADSPI_CCR_ABMODE,
          QspiLineMode, QUADSPI_CCR_ABMODE_Pos> alternateBytesMode;
  RegEnum<addr_CCR, QUADSPI_CCR_ABSIZE,
          QspiAlternateBytesSize, QUADSPI_CCR_ABSIZE_Pos> alternateBytesSize;
  RegValue<addr_CCR, QUADSPI_CCR_DCYC,
           QUADSPI_CCR_DCYC_Pos>                dummyCycles;
  RegEnum<addr_CCR, QUADSPI_CCR_DMODE,
          QspiLineMode, QUADSPI_CCR_DMODE_Pos>  dataMode;
  RegEnum<addr_CCR, QUADSPI_CCR_FMODE,
          QspiFunctionalMode, QUADSPI_CCR_FMODE_Pos> functionalMode;
  RegFlag<addr_CCR, QUADSPI_CCR_SIOO>           sendInstructionOnce;
  RegFlag<addr_CCR, QUADSPI_CCR_DHHC>           ddrHoldHalfCycle;
  RegFlag<addr_CCR, QUADSPI_CCR_DDRM>           ddrMode;

  // ----- AR register (address) -----
  Reg<addr_AR>                                   address;

  // ----- ABR register (alternate bytes) -----
  Reg<addr_ABR>                                  alternateBytes;

  // ----- DR register (data) -----
  Reg<addr_DR>                                   data;

  // ----- PSMKR register (polling status mask) -----
  Reg<addr_PSMKR>                                pollingMask;

  // ----- PSMAR register (polling status match) -----
  Reg<addr_PSMAR>                                pollingMatch;

  // ----- PIR register (polling interval) -----
  RegValue<addr_PIR, QUADSPI_PIR_INTERVAL,
           QUADSPI_PIR_INTERVAL_Pos>            pollingInterval;

  // ----- LPTR register (low-power timeout) -----
  RegValue<addr_LPTR, QUADSPI_LPTR_TIMEOUT,
           QUADSPI_LPTR_TIMEOUT_Pos>            timeout;

  // ----- High-level init helpers -----

  IGB_FAST_INLINE void initControl(const QspiConf& conf) {
    (prescaler.val(static_cast<uint32_t>(conf.prescaler))
      | pollMatchMode.val(conf.pollMatchMode)
      | autoPollStop.val(conf.autoPollStop)
      | fifoThreshold.val(static_cast<uint32_t>(conf.fifoThreshold))
      | flashSelect.val(conf.flashSelect)
      | enableDualFlash.val(conf.dualFlashMode)
      | sampleShift.val(conf.sampleShift)
      | timeoutCounterEnable.val(conf.timeoutCounterEnable)
      | enableItTransferError.val(conf.enableItTransferError)
      | enableItTransferComplete.val(conf.enableItTransferComplete)
      | enableItFifoThreshold.val(conf.enableItFifoThreshold)
      | enableItStatusMatch.val(conf.enableItStatusMatch)
      | enableItTimeout.val(conf.enableItTimeout)
    ).update();
  }

  IGB_FAST_INLINE void initDevice(const QspiDeviceConf& conf) {
    (flashSize.val(static_cast<uint32_t>(conf.flashSize))
      | csHighTime.val(conf.csHighTime)
      | clockMode3.val(conf.clockMode == QspiClockMode::mode3)
    ).update();
  }

  IGB_FAST_INLINE void initCommand(const QspiCommandConf& conf) {
    (instruction.val(static_cast<uint32_t>(conf.instruction))
      | instructionMode.val(conf.instructionMode)
      | addressMode.val(conf.addressMode)
      | addressSize.val(conf.addressSize)
      | alternateBytesMode.val(conf.alternateBytesMode)
      | alternateBytesSize.val(conf.alternateBytesSize)
      | dummyCycles.val(static_cast<uint32_t>(conf.dummyCycles))
      | dataMode.val(conf.dataMode)
      | functionalMode.val(conf.functionalMode)
      | sendInstructionOnce.val(conf.sendInstructionOnce)
      | ddrHoldHalfCycle.val(conf.ddrHoldHalfCycle)
      | ddrMode.val(conf.ddrMode)
    ).update();
  }

  IGB_FAST_INLINE void initAutoPolling(const QspiAutoPollingConf& conf) {
    pollingMask(conf.mask);
    pollingMatch(conf.match);
    pollingInterval(static_cast<uint32_t>(conf.interval));
  }

  IGB_FAST_INLINE void setTimeout(uint16_t value) {
    timeout(static_cast<uint32_t>(value));
  }

  IGB_FAST_INLINE void setDataLength(uint32_t numBytes) {
    dataLength(numBytes - 1U);
  }

  IGB_FAST_INLINE void setAddress(uint32_t addr) {
    address(addr);
  }

  IGB_FAST_INLINE void setAlternateBytes(uint32_t value) {
    alternateBytes(value);
  }

  IGB_FAST_INLINE void     write(uint32_t value) { data(value); }
  IGB_FAST_INLINE uint32_t read()                { return data(); }

  IGB_FAST_INLINE void clearAllFlags() {
    clearTransferError(true);
    clearTransferComplete(true);
    clearStatusMatch(true);
    clearTimeout(true);
  }

  IGB_FAST_INLINE void requestAbort() { abort(true); }

  // ----- Bus clock management -----
  IGB_FAST_INLINE static void enableBusClock() {
    STM32_PERIPH_INFO.quadspi[0].bus.enableBusClock();
  }
  IGB_FAST_INLINE static void disableBusClock() {
    STM32_PERIPH_INFO.quadspi[0].bus.disableBusClock();
  }
  IGB_FAST_INLINE static void forceResetBusClock() {
    STM32_PERIPH_INFO.quadspi[0].bus.forceResetBusClock();
  }
  IGB_FAST_INLINE static void releaseResetBusClock() {
    STM32_PERIPH_INFO.quadspi[0].bus.releaseResetBusClock();
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto result = get_af_idx(PeriphType::quadspi, pin_type);
    if (!result) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(result.value());
  }
};

} // namespace stm32
} // namespace igb

#endif // STM32_PERIPH_QUADSPI_EXISTS

#endif // STM32H7
