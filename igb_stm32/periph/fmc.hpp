#pragma once

#if defined(STM32H7)

#ifdef STM32_PERIPH_FMC_EXISTS

#include <igb_stm32/base.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

#define IGB_FMC_BANK1    ((FMC_Bank1_TypeDef*)FMC_Bank1_R_BASE)
#define IGB_FMC_BANK1E   ((FMC_Bank1E_TypeDef*)FMC_Bank1E_R_BASE)
#define IGB_FMC_BANK3    ((FMC_Bank3_TypeDef*)FMC_Bank3_R_BASE)
#define IGB_FMC_BANK5_6  ((FMC_Bank5_6_TypeDef*)FMC_Bank5_6_R_BASE)


// ============================================================
// NOR/SRAM (Bank 1-4) enums and conf
// ============================================================

enum class FmcNorSramMemType : uint32_t {
  sram  = 0,
  psram = FMC_BCRx_MTYP_0,
  nor   = FMC_BCRx_MTYP_1,
};

enum class FmcNorSramDataWidth : uint32_t {
  _8bit  = 0,
  _16bit = FMC_BCRx_MWID_0,
  _32bit = FMC_BCRx_MWID_1,
};

enum class FmcNorSramPageSize : uint32_t {
  none   = 0,
  _128b  = FMC_BCRx_CPSIZE_0,
  _256b  = FMC_BCRx_CPSIZE_1,
  _512b  = FMC_BCRx_CPSIZE_0 | FMC_BCRx_CPSIZE_1,
  _1024b = FMC_BCRx_CPSIZE_2,
};

enum class FmcNorSramAccessMode : uint32_t {
  a = 0,
  b = FMC_BTRx_ACCMOD_0,
  c = FMC_BTRx_ACCMOD_1,
  d = FMC_BTRx_ACCMOD_0 | FMC_BTRx_ACCMOD_1,
};

struct FmcNorSramConf {
  bool enableDataAddressMux           = false;
  FmcNorSramMemType memType           = FmcNorSramMemType::sram;
  FmcNorSramDataWidth dataWidth       = FmcNorSramDataWidth::_16bit;
  bool enableFlashAccess              = false;
  bool enableBurstAccess              = false;
  bool waitPolHigh                    = false;
  bool waitDuringAccess               = false;
  bool enableWrite                    = true;
  bool enableWaitSignal               = false;
  bool enableExtendedMode             = false;
  bool enableAsyncWait                = false;
  FmcNorSramPageSize pageSize         = FmcNorSramPageSize::none;
  bool enableWriteBurst               = false;
  bool enableContinuousClock          = false; // BCR1 (bank 0) only
  bool disableWriteFifo               = false; // BCR1 (bank 0) only
};

struct FmcNorSramTimingConf {
  uint8_t addressSetup    = 15;  // 0–15 HCLK cycles
  uint8_t addressHold     = 15;  // 1–15 HCLK cycles (ignored for synchronous)
  uint8_t dataSetup       = 255; // 1–255 HCLK cycles
  uint8_t busTurnAround   = 15;  // 0–15 HCLK cycles
  uint8_t clkDivision     = 15;  // 1–15 (encoded as value-1: 0x0=div2 .. 0xF=div16)
  uint8_t dataLatency     = 15;  // 0–15 (encoded as value-2: 0x0=2 .. 0xF=17)
  FmcNorSramAccessMode accessMode = FmcNorSramAccessMode::a;
};

// ============================================================
// NAND (Bank 3) enums and conf
// ============================================================

enum class FmcNandDataWidth : uint32_t {
  _8bit  = 0,
  _16bit = FMC_PCR_PWID_0,
};

enum class FmcNandEccPageSize : uint32_t {
  _256b  = 0,
  _512b  = FMC_PCR_ECCPS_0,
  _1024b = FMC_PCR_ECCPS_1,
  _2048b = FMC_PCR_ECCPS_0 | FMC_PCR_ECCPS_1,
  _4096b = FMC_PCR_ECCPS_2,
  _8192b = FMC_PCR_ECCPS_0 | FMC_PCR_ECCPS_2,
};

enum class FmcNandItFlag : uint32_t {
  risingEdge  = FMC_SR_IREN,
  level       = FMC_SR_ILEN,
  fallingEdge = FMC_SR_IFEN,
};

enum class FmcNandStatusFlag : uint32_t {
  risingEdge  = FMC_SR_IRS,
  level       = FMC_SR_ILS,
  fallingEdge = FMC_SR_IFS,
  fifoEmpty   = FMC_SR_FEMPT,
};

struct FmcNandConf {
  bool enableWaitFeature              = false;
  FmcNandDataWidth dataWidth          = FmcNandDataWidth::_8bit;
  bool enableEcc                      = false;
  uint8_t tclr                        = 0;   // CLE-to-RE delay: 0–15 HCLK cycles
  uint8_t tar                         = 0;   // ALE-to-RE delay: 0–15 HCLK cycles
  FmcNandEccPageSize eccPageSize      = FmcNandEccPageSize::_256b;
};

struct FmcNandTimingConf {
  uint8_t setup = 254; // 0–254 HCLK cycles
  uint8_t wait  = 254; // 0–254 HCLK cycles
  uint8_t hold  = 254; // 0–254 HCLK cycles
  uint8_t hiz   = 254; // 0–254 HCLK cycles
};

// ============================================================
// SDRAM (Bank 5/6) enums and conf
// ============================================================

enum class FmcSdramColumnBits : uint32_t {
  _8bit  = 0,
  _9bit  = FMC_SDCRx_NC_0,
  _10bit = FMC_SDCRx_NC_1,
  _11bit = FMC_SDCRx_NC_0 | FMC_SDCRx_NC_1,
};

enum class FmcSdramRowBits : uint32_t {
  _11bit = 0,
  _12bit = FMC_SDCRx_NR_0,
  _13bit = FMC_SDCRx_NR_1,
};

enum class FmcSdramDataWidth : uint32_t {
  _8bit  = 0,
  _16bit = FMC_SDCRx_MWID_0,
  _32bit = FMC_SDCRx_MWID_1,
};

enum class FmcSdramInternalBanks : uint32_t {
  _2banks = 0,
  _4banks = FMC_SDCRx_NB,
};

enum class FmcSdramCasLatency : uint32_t {
  _1clk = FMC_SDCRx_CAS_0,
  _2clk = FMC_SDCRx_CAS_1,
  _3clk = FMC_SDCRx_CAS_0 | FMC_SDCRx_CAS_1,
};

enum class FmcSdramClockPeriod : uint32_t {
  disable = 0,
  _2clk   = FMC_SDCRx_SDCLK_1,
  _3clk   = FMC_SDCRx_SDCLK_0 | FMC_SDCRx_SDCLK_1,
};

enum class FmcSdramReadPipeDelay : uint32_t {
  _0clk = 0,
  _1clk = FMC_SDCRx_RPIPE_0,
  _2clk = FMC_SDCRx_RPIPE_1,
};

// SDCMR MODE field: values 0–6 (not individual bits)
enum class FmcSdramCommandMode : uint32_t {
  normal       = 0,
  clockEnable  = 1,
  pall         = 2,
  autoRefresh  = 3,
  loadModeReg  = 4,
  selfRefresh  = 5,
  powerDown    = 6,
};

enum class FmcSdramCommandTarget : uint32_t {
  bank1 = FMC_SDCMR_CTB1,
  bank2 = FMC_SDCMR_CTB2,
  both  = FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2,
};

// Field values (0–2), not raw bit positions — works with RegEnumRO on both MODES1 and MODES2
enum class FmcSdramModeStatus : uint32_t {
  normal      = 0,
  selfRefresh = 1,
  powerDown   = 2,
};

struct FmcSdramConf {
  FmcSdramColumnBits columnBits           = FmcSdramColumnBits::_8bit;
  FmcSdramRowBits rowBits                 = FmcSdramRowBits::_11bit;
  FmcSdramDataWidth dataWidth             = FmcSdramDataWidth::_16bit;
  FmcSdramInternalBanks internalBanks     = FmcSdramInternalBanks::_4banks;
  FmcSdramCasLatency casLatency           = FmcSdramCasLatency::_3clk;
  bool enableWriteProtect                 = false;
  FmcSdramClockPeriod clockPeriod         = FmcSdramClockPeriod::_2clk;
  bool enableReadBurst                    = false;
  FmcSdramReadPipeDelay readPipeDelay     = FmcSdramReadPipeDelay::_0clk;
};

struct FmcSdramTimingConf {
  uint8_t loadToActive    = 16; // 1–16 memory clock cycles
  uint8_t exitSelfRefresh = 16; // 1–16 memory clock cycles
  uint8_t selfRefreshTime = 16; // 1–16 memory clock cycles
  uint8_t rowCycleDelay   = 16; // 1–16 memory clock cycles
  uint8_t writeRecovery   = 16; // 1–16 memory clock cycles
  uint8_t rpDelay         = 16; // 1–16 memory clock cycles
  uint8_t rcdDelay        = 16; // 1–16 memory clock cycles
};

// ============================================================
// NOR/SRAM bank struct  (BANK_IDX: 0–3 → Bank1–Bank4)
// ============================================================

template<uint8_t BANK_IDX>
struct FmcNorSram {
  static_assert(BANK_IDX < 4, "FmcNorSram: BANK_IDX must be 0–3");

  // BCR/BTR are interleaved in BTCR[8]: index 0,2,4,6 = BCRx, index 1,3,5,7 = BTRx
  constexpr static uint32_t addr_BCR  = FMC_Bank1_R_BASE  + BANK_IDX * 8U;
  constexpr static uint32_t addr_BTR  = FMC_Bank1_R_BASE  + BANK_IDX * 8U + 4U;
  // Extended write-timing: BWTR[7] in Bank1E, stored at even indices (0,2,4,6)
  constexpr static uint32_t addr_BWTR = FMC_Bank1E_R_BASE + BANK_IDX * 8U;

  // ----- BCR register fields -----
  RegFlag<addr_BCR, FMC_BCRx_MBKEN>          enableMemBank;
  RegFlag<addr_BCR, FMC_BCRx_MUXEN>          enableDataAddressMux;
  RegEnum<addr_BCR, FMC_BCRx_MTYP,
          FmcNorSramMemType, FMC_BCRx_MTYP_Pos>   memType;
  RegEnum<addr_BCR, FMC_BCRx_MWID,
          FmcNorSramDataWidth, FMC_BCRx_MWID_Pos>  dataWidth;
  RegFlag<addr_BCR, FMC_BCRx_FACCEN>         enableFlashAccess;
  RegFlag<addr_BCR, FMC_BCRx_BURSTEN>        enableBurstAccess;
  RegFlag<addr_BCR, FMC_BCRx_WAITPOL>        waitPolHigh;
  RegFlag<addr_BCR, FMC_BCRx_WAITCFG>        waitDuringAccess;
  RegFlag<addr_BCR, FMC_BCRx_WREN>           enableWrite;
  RegFlag<addr_BCR, FMC_BCRx_WAITEN>         enableWaitSignal;
  RegFlag<addr_BCR, FMC_BCRx_EXTMOD>         enableExtendedMode;
  RegFlag<addr_BCR, FMC_BCRx_ASYNCWAIT>      enableAsyncWait;
  RegEnum<addr_BCR, FMC_BCRx_CPSIZE,
          FmcNorSramPageSize, FMC_BCRx_CPSIZE_Pos> pageSize;
  RegFlag<addr_BCR, FMC_BCRx_CBURSTRW>       enableWriteBurst;
  // BCR1-only fields (only valid when BANK_IDX == 0)
  RegFlag<addr_BCR, FMC_BCR1_CCLKEN>         enableContinuousClock;
  RegFlag<addr_BCR, FMC_BCR1_WFDIS>          disableWriteFifo;

  // ----- BTR (read/write timing) register fields -----
  RegValue<addr_BTR, FMC_BTRx_ADDSET,
           FMC_BTRx_ADDSET_Pos>  addressSetup;
  RegValue<addr_BTR, FMC_BTRx_ADDHLD,
           FMC_BTRx_ADDHLD_Pos>  addressHold;
  RegValue<addr_BTR, FMC_BTRx_DATAST,
           FMC_BTRx_DATAST_Pos>  dataSetup;
  RegValue<addr_BTR, FMC_BTRx_BUSTURN,
           FMC_BTRx_BUSTURN_Pos> busTurnAround;
  RegValue<addr_BTR, FMC_BTRx_CLKDIV,
           FMC_BTRx_CLKDIV_Pos>  clkDivision;
  RegValue<addr_BTR, FMC_BTRx_DATLAT,
           FMC_BTRx_DATLAT_Pos>  dataLatency;
  RegEnum<addr_BTR, FMC_BTRx_ACCMOD,
          FmcNorSramAccessMode, FMC_BTRx_ACCMOD_Pos> accessMode;

  // ----- BWTR (extended write timing) register fields -----
  // Only meaningful when enableExtendedMode is set
  RegValue<addr_BWTR, FMC_BWTRx_ADDSET,
           FMC_BWTRx_ADDSET_Pos>  writeAddressSetup;
  RegValue<addr_BWTR, FMC_BWTRx_ADDHLD,
           FMC_BWTRx_ADDHLD_Pos>  writeAddressHold;
  RegValue<addr_BWTR, FMC_BWTRx_DATAST,
           FMC_BWTRx_DATAST_Pos>  writeDataSetup;
  RegValue<addr_BWTR, FMC_BWTRx_BUSTURN,
           FMC_BWTRx_BUSTURN_Pos> writeBusTurnAround;
  RegEnum<addr_BWTR, FMC_BWTRx_ACCMOD,
          FmcNorSramAccessMode, FMC_BWTRx_ACCMOD_Pos> writeAccessMode;

  // ----- High-level init -----
  IGB_FAST_INLINE void initControl(const FmcNorSramConf& conf) {
    auto reg = enableDataAddressMux.val(conf.enableDataAddressMux)
      | memType.val(conf.memType)
      | dataWidth.val(conf.dataWidth)
      | enableFlashAccess.val(conf.enableFlashAccess)
      | enableBurstAccess.val(conf.enableBurstAccess)
      | waitPolHigh.val(conf.waitPolHigh)
      | waitDuringAccess.val(conf.waitDuringAccess)
      | enableWrite.val(conf.enableWrite)
      | enableWaitSignal.val(conf.enableWaitSignal)
      | enableExtendedMode.val(conf.enableExtendedMode)
      | enableAsyncWait.val(conf.enableAsyncWait)
      | pageSize.val(conf.pageSize)
      | enableWriteBurst.val(conf.enableWriteBurst)
      ;
    if constexpr (BANK_IDX == 0) {
      (reg
        | enableContinuousClock.val(conf.enableContinuousClock)
        | disableWriteFifo.val(conf.disableWriteFifo)
      ).update();
    } else {
      reg.update();
    }
  }

  IGB_FAST_INLINE void initTiming(const FmcNorSramTimingConf& conf) {
    (addressSetup.val(conf.addressSetup)
      | addressHold.val(conf.addressHold)
      | dataSetup.val(conf.dataSetup)
      | busTurnAround.val(conf.busTurnAround)
      | clkDivision.val(conf.clkDivision)
      | dataLatency.val(conf.dataLatency)
      | accessMode.val(conf.accessMode)
    ).update();
  }

  IGB_FAST_INLINE void initWriteTiming(const FmcNorSramTimingConf& conf) {
    (writeAddressSetup.val(conf.addressSetup)
      | writeAddressHold.val(conf.addressHold)
      | writeDataSetup.val(conf.dataSetup)
      | writeBusTurnAround.val(conf.busTurnAround)
      | writeAccessMode.val(conf.accessMode)
    ).update();
  }

  IGB_FAST_INLINE void enable()  { enableMemBank(true); }
  IGB_FAST_INLINE void disable() { enableMemBank(false); }
};

// ============================================================
// NAND bank struct  (Bank 3 only on STM32H7)
// ============================================================

struct FmcNand {
  constexpr static uint32_t addr_PCR  = FMC_Bank3_R_BASE + offsetof(FMC_Bank3_TypeDef, PCR);
  constexpr static uint32_t addr_SR   = FMC_Bank3_R_BASE + offsetof(FMC_Bank3_TypeDef, SR);
  constexpr static uint32_t addr_PMEM = FMC_Bank3_R_BASE + offsetof(FMC_Bank3_TypeDef, PMEM);
  constexpr static uint32_t addr_PATT = FMC_Bank3_R_BASE + offsetof(FMC_Bank3_TypeDef, PATT);
  constexpr static uint32_t addr_ECCR = FMC_Bank3_R_BASE + offsetof(FMC_Bank3_TypeDef, ECCR);

  // ----- PCR register fields -----
  RegFlag<addr_PCR, FMC_PCR_PWAITEN>        enableWait;
  RegFlag<addr_PCR, FMC_PCR_PBKEN>          enableMemBank;
  RegEnum<addr_PCR, FMC_PCR_PWID,
          FmcNandDataWidth, FMC_PCR_PWID_Pos>   dataWidth;
  RegFlag<addr_PCR, FMC_PCR_ECCEN>          enableEcc;
  RegValue<addr_PCR, FMC_PCR_TCLR,
           FMC_PCR_TCLR_Pos>                tclr;
  RegValue<addr_PCR, FMC_PCR_TAR,
           FMC_PCR_TAR_Pos>                 tar;
  RegEnum<addr_PCR, FMC_PCR_ECCPS,
          FmcNandEccPageSize, FMC_PCR_ECCPS_Pos> eccPageSize;

  // ----- SR register fields -----
  RegFlag<addr_SR, FMC_SR_IRS>              statusRisingEdge;
  RegFlag<addr_SR, FMC_SR_ILS>              statusLevel;
  RegFlag<addr_SR, FMC_SR_IFS>             statusFallingEdge;
  RegFlag<addr_SR, FMC_SR_IREN>             enableItRisingEdge;
  RegFlag<addr_SR, FMC_SR_ILEN>             enableItLevel;
  RegFlag<addr_SR, FMC_SR_IFEN>             enableItFallingEdge;
  RegFlagRO<addr_SR, FMC_SR_FEMPT>          fifoEmpty;

  // ----- ECC result (read-only) -----
  RegRO<addr_ECCR>                           eccResult;

  // ----- PMEM (common memory timing) register fields -----
  RegValue<addr_PMEM, FMC_PMEM_MEMSET,
           FMC_PMEM_MEMSET_Pos>   commonSetup;
  RegValue<addr_PMEM, FMC_PMEM_MEMWAIT,
           FMC_PMEM_MEMWAIT_Pos>  commonWait;
  RegValue<addr_PMEM, FMC_PMEM_MEMHOLD,
           FMC_PMEM_MEMHOLD_Pos>  commonHold;
  RegValue<addr_PMEM, FMC_PMEM_MEMHIZ,
           FMC_PMEM_MEMHIZ_Pos>   commonHiz;

  // ----- PATT (attribute memory timing) register fields -----
  RegValue<addr_PATT, FMC_PATT_ATTSET,
           FMC_PATT_ATTSET_Pos>   attribSetup;
  RegValue<addr_PATT, FMC_PATT_ATTWAIT,
           FMC_PATT_ATTWAIT_Pos>  attribWait;
  RegValue<addr_PATT, FMC_PATT_ATTHOLD,
           FMC_PATT_ATTHOLD_Pos>  attribHold;
  RegValue<addr_PATT, FMC_PATT_ATTHIZ,
           FMC_PATT_ATTHIZ_Pos>   attribHiz;

  // ----- High-level init -----
  IGB_FAST_INLINE void initControl(const FmcNandConf& conf) {
    (enableWait.val(conf.enableWaitFeature)
      | dataWidth.val(conf.dataWidth)
      | enableEcc.val(conf.enableEcc)
      | tclr.val(conf.tclr)
      | tar.val(conf.tar)
      | eccPageSize.val(conf.eccPageSize)
    ).update();
  }

  IGB_FAST_INLINE void initCommonTiming(const FmcNandTimingConf& conf) {
    (commonSetup.val(conf.setup)
      | commonWait.val(conf.wait)
      | commonHold.val(conf.hold)
      | commonHiz.val(conf.hiz)
    ).update();
  }

  IGB_FAST_INLINE void initAttribTiming(const FmcNandTimingConf& conf) {
    (attribSetup.val(conf.setup)
      | attribWait.val(conf.wait)
      | attribHold.val(conf.hold)
      | attribHiz.val(conf.hiz)
    ).update();
  }

  IGB_FAST_INLINE void enable()  { enableMemBank(true); }
  IGB_FAST_INLINE void disable() { enableMemBank(false); }

  IGB_FAST_INLINE bool is(FmcNandStatusFlag flag) {
    switch (flag) {
      case FmcNandStatusFlag::risingEdge:  return statusRisingEdge();
      case FmcNandStatusFlag::level:       return statusLevel();
      case FmcNandStatusFlag::fallingEdge: return statusFallingEdge();
      case FmcNandStatusFlag::fifoEmpty:   return fifoEmpty();
      default: return false;
    }
  }
  IGB_FAST_INLINE void clearStatus(FmcNandStatusFlag flag) {
    switch (flag) {
      case FmcNandStatusFlag::risingEdge:  statusRisingEdge(false); break;
      case FmcNandStatusFlag::level:       statusLevel(false); break;
      case FmcNandStatusFlag::fallingEdge: statusFallingEdge(false); break;
      default: break;
    }
  }
  IGB_FAST_INLINE void enable(FmcNandItFlag flag) {
    switch (flag) {
      case FmcNandItFlag::risingEdge:  enableItRisingEdge(true); break;
      case FmcNandItFlag::level:       enableItLevel(true); break;
      case FmcNandItFlag::fallingEdge: enableItFallingEdge(true); break;
      default: break;
    }
  }
  IGB_FAST_INLINE void disable(FmcNandItFlag flag) {
    switch (flag) {
      case FmcNandItFlag::risingEdge:  enableItRisingEdge(false); break;
      case FmcNandItFlag::level:       enableItLevel(false); break;
      case FmcNandItFlag::fallingEdge: enableItFallingEdge(false); break;
      default: break;
    }
  }
  IGB_FAST_INLINE uint32_t getEccValue() { return eccResult(); }
};

// ============================================================
// SDRAM bank struct  (BANK_IDX: 0 = Bank5, 1 = Bank6)
// ============================================================

template<uint8_t BANK_IDX>
struct FmcSdram {
  static_assert(BANK_IDX < 2, "FmcSdram: BANK_IDX must be 0 or 1");

  constexpr static uint32_t addr_SDCR  = FMC_Bank5_6_R_BASE + offsetof(FMC_Bank5_6_TypeDef, SDCR) + BANK_IDX * 4U;
  constexpr static uint32_t addr_SDTR  = FMC_Bank5_6_R_BASE + offsetof(FMC_Bank5_6_TypeDef, SDTR) + BANK_IDX * 4U;
  // SDCMR, SDRTR, SDSR are shared between both SDRAM banks
  constexpr static uint32_t addr_SDCMR = FMC_Bank5_6_R_BASE + offsetof(FMC_Bank5_6_TypeDef, SDCMR);
  constexpr static uint32_t addr_SDRTR = FMC_Bank5_6_R_BASE + offsetof(FMC_Bank5_6_TypeDef, SDRTR);
  constexpr static uint32_t addr_SDSR  = FMC_Bank5_6_R_BASE + offsetof(FMC_Bank5_6_TypeDef, SDSR);

  // ----- SDCR register fields -----
  RegEnum<addr_SDCR, FMC_SDCRx_NC,
          FmcSdramColumnBits, FMC_SDCRx_NC_Pos>       columnBits;
  RegEnum<addr_SDCR, FMC_SDCRx_NR,
          FmcSdramRowBits, FMC_SDCRx_NR_Pos>           rowBits;
  RegEnum<addr_SDCR, FMC_SDCRx_MWID,
          FmcSdramDataWidth, FMC_SDCRx_MWID_Pos>       dataWidth;
  RegEnum<addr_SDCR, FMC_SDCRx_NB,
          FmcSdramInternalBanks, FMC_SDCRx_NB_Pos>     internalBanks;
  RegEnum<addr_SDCR, FMC_SDCRx_CAS,
          FmcSdramCasLatency, FMC_SDCRx_CAS_Pos>       casLatency;
  RegFlag<addr_SDCR, FMC_SDCRx_WP>                     enableWriteProtect;
  RegEnum<addr_SDCR, FMC_SDCRx_SDCLK,
          FmcSdramClockPeriod, FMC_SDCRx_SDCLK_Pos>    clockPeriod;
  RegFlag<addr_SDCR, FMC_SDCRx_RBURST>                 enableReadBurst;
  RegEnum<addr_SDCR, FMC_SDCRx_RPIPE,
          FmcSdramReadPipeDelay, FMC_SDCRx_RPIPE_Pos>  readPipeDelay;

  // ----- SDTR register fields -----
  RegValue<addr_SDTR, FMC_SDTRx_TMRD,
           FMC_SDTRx_TMRD_Pos>  loadToActive;
  RegValue<addr_SDTR, FMC_SDTRx_TXSR,
           FMC_SDTRx_TXSR_Pos>  exitSelfRefresh;
  RegValue<addr_SDTR, FMC_SDTRx_TRAS,
           FMC_SDTRx_TRAS_Pos>  selfRefreshTime;
  RegValue<addr_SDTR, FMC_SDTRx_TRC,
           FMC_SDTRx_TRC_Pos>   rowCycleDelay;
  RegValue<addr_SDTR, FMC_SDTRx_TWR,
           FMC_SDTRx_TWR_Pos>   writeRecoveryTime;
  RegValue<addr_SDTR, FMC_SDTRx_TRP,
           FMC_SDTRx_TRP_Pos>   rpDelay;
  RegValue<addr_SDTR, FMC_SDTRx_TRCD,
           FMC_SDTRx_TRCD_Pos>  rcdDelay;

  // ----- SDCMR register fields (shared) -----
  RegEnum<addr_SDCMR, FMC_SDCMR_MODE,
          FmcSdramCommandMode, FMC_SDCMR_MODE_Pos>  commandMode;
  RegFlag<addr_SDCMR, FMC_SDCMR_CTB1>  commandTargetBank1;
  RegFlag<addr_SDCMR, FMC_SDCMR_CTB2>  commandTargetBank2;
  RegValue<addr_SDCMR, FMC_SDCMR_NRFS,
           FMC_SDCMR_NRFS_Pos>  autoRefreshNumber;
  RegValue<addr_SDCMR, FMC_SDCMR_MRD,
           FMC_SDCMR_MRD_Pos>   modeRegister;

  // ----- SDRTR register fields (shared) -----
  RegFlag<addr_SDRTR, FMC_SDRTR_CRE>                   clearRefreshError;
  RegValue<addr_SDRTR, FMC_SDRTR_COUNT,
           FMC_SDRTR_COUNT_Pos> refreshCount;
  RegFlag<addr_SDRTR, FMC_SDRTR_REIE>                  enableRefreshErrorIt;

  // ----- SDSR register fields (read-only, shared) -----
  RegFlagRO<addr_SDSR, FMC_SDSR_RE>    refreshError;
  RegEnumRO<addr_SDSR, FMC_SDSR_MODES1,
            FmcSdramModeStatus, FMC_SDSR_MODES1_Pos>  bank1ModeStatus;
  RegEnumRO<addr_SDSR, FMC_SDSR_MODES2,
            FmcSdramModeStatus, FMC_SDSR_MODES2_Pos>  bank2ModeStatus;

  // ----- High-level init -----
  IGB_FAST_INLINE void initControl(const FmcSdramConf& conf) {
    auto reg = columnBits.val(conf.columnBits)
      | rowBits.val(conf.rowBits)
      | dataWidth.val(conf.dataWidth)
      | internalBanks.val(conf.internalBanks)
      | casLatency.val(conf.casLatency)
      | enableWriteProtect.val(conf.enableWriteProtect)
      ;
    // SDCLK, RBURST, RPIPE are only effective in SDCR1 (bank 0);
    // writing them to SDCR2 has no effect per RM0433, so skip to avoid touching reserved state
    if constexpr (BANK_IDX == 0) {
      (reg
        | clockPeriod.val(conf.clockPeriod)
        | enableReadBurst.val(conf.enableReadBurst)
        | readPipeDelay.val(conf.readPipeDelay)
      ).update();
    } else {
      reg.update();
    }
  }

  IGB_FAST_INLINE void initTiming(const FmcSdramTimingConf& conf) {
    auto reg = loadToActive.val(conf.loadToActive - 1U)
      | exitSelfRefresh.val(conf.exitSelfRefresh - 1U)
      | selfRefreshTime.val(conf.selfRefreshTime - 1U)
      | writeRecoveryTime.val(conf.writeRecovery - 1U)
      | rcdDelay.val(conf.rcdDelay - 1U)
      ;
    // TRC and TRP are only effective in SDTR1 (bank 0);
    // writing them to SDTR2 has no effect per RM0433, so skip to avoid touching reserved state
    if constexpr (BANK_IDX == 0) {
      (reg
        | rowCycleDelay.val(conf.rowCycleDelay - 1U)
        | rpDelay.val(conf.rpDelay - 1U)
      ).update();
    } else {
      reg.update();
    }
  }

  // Send an SDRAM command to this bank's target
  IGB_FAST_INLINE void sendCommand(
      FmcSdramCommandMode mode,
      uint8_t autoRefreshNum = 1,
      uint16_t modeReg = 0)
  {
    (commandMode.val(mode)
      | commandTargetBank1.val(BANK_IDX == 0)
      | commandTargetBank2.val(BANK_IDX != 0)
      | autoRefreshNumber.val(autoRefreshNum - 1U)
      | modeRegister.val(modeReg)
    ).update();
  }

  // Send a command targeting both SDRAM banks simultaneously
  IGB_FAST_INLINE static void sendCommandBoth(
      FmcSdramCommandMode mode,
      uint8_t autoRefreshNum = 1,
      uint16_t modeReg = 0)
  {
    FmcSdram<0> bank {};
    (bank.commandMode.val(mode)
      | bank.commandTargetBank1.val(true)
      | bank.commandTargetBank2.val(true)
      | bank.autoRefreshNumber.val(autoRefreshNum - 1U)
      | bank.modeRegister.val(modeReg)
    ).update();
  }

  // Set SDRAM refresh rate.
  // sdclk_hz  : SDRAM clock frequency in Hz (e.g. 120'000'000 for 120 MHz)
  // num_rows  : number of rows in the SDRAM array (e.g. 8192)
  // rate_hz   : desired refresh rate in Hz — total array refreshes per second
  //             (e.g. 1.0f / 0.064f ≈ 15.625f for a 64 ms refresh period)
  // Formula: COUNT = sdclk_hz / (rate_hz * num_rows) - 20  (RM0433 §32.7.5)
  IGB_FAST_INLINE void setRefreshRate(uint32_t sdclk_hz, uint16_t num_rows, float rate_hz) {
    refreshCount(static_cast<uint32_t>(sdclk_hz / (rate_hz * num_rows) - 20.0f));
  }

  IGB_FAST_INLINE bool hasRefreshError() { return refreshError(); }

  IGB_FAST_INLINE FmcSdramModeStatus getModeStatus() {
    if constexpr (BANK_IDX == 0) {
      return bank1ModeStatus();
    } else {
      return bank2ModeStatus();
    }
  }
};

// ============================================================
// Top-level FMC controller
// ============================================================

struct Fmc {
  constexpr static uint32_t addr_BCR1 = FMC_Bank1_R_BASE;
  RegFlag<addr_BCR1, FMC_BCR1_FMCEN> fmcEnable;

  // Bus clock management (AHB3)
  IGB_FAST_INLINE static void enableBusClock() {
    STM32_PERIPH_INFO.fmc[0].bus.enableBusClock();
  }
  IGB_FAST_INLINE static void disableBusClock() {
    STM32_PERIPH_INFO.fmc[0].bus.disableBusClock();
  }
  IGB_FAST_INLINE static void forceResetBusClock() {
    STM32_PERIPH_INFO.fmc[0].bus.forceResetBusClock();
  }
  IGB_FAST_INLINE static void releaseResetBusClock() {
    STM32_PERIPH_INFO.fmc[0].bus.releaseResetBusClock();
  }

  // FMC global enable/disable (via BCR1 FMCEN bit)
  IGB_FAST_INLINE static void enable()     { Fmc{}.fmcEnable(true); }
  IGB_FAST_INLINE static void disable()    { Fmc{}.fmcEnable(false); }
  IGB_FAST_INLINE static bool isEnabled()  { return Fmc{}.fmcEnable(); }

  // Convenience type aliases for sub-controller structs
  template<uint8_t BANK_IDX>
  using NorSram = FmcNorSram<BANK_IDX>;

  using Nand = FmcNand;

  template<uint8_t BANK_IDX>
  using Sdram = FmcSdram<BANK_IDX>;
};

#undef IGB_FMC_BANK1
#undef IGB_FMC_BANK1E
#undef IGB_FMC_BANK3
#undef IGB_FMC_BANK5_6

} /* stm32 */
} /* igb */

#endif /* STM32_PERIPH_FMC_EXISTS */

#endif // defined(STM32H7)
