#pragma once

#if defined(STM32H7)

#ifdef STM32_PERIPH_MDMA_EXISTS

#include <stddef.h>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

// ============================================================
// MDMA (master direct memory access) — RM0433 §14
//
// A D1-domain master with BOTH a 64-bit AXI system-bus port and a 32-bit
// AHB/TCM port, which makes it the only DMA on this device that can write
// the CPU's tightly-coupled memories (through the Cortex-M7 AHBS slave).
// 16 independent channels share one NVIC line (MDMA_IRQn).
//
// Transfer hierarchy (RM0433 §14.3.4-14.3.11), smallest first:
//   burst  : the beats that cannot be interrupted at bus-arbitration level
//   buffer : TLEN+1 bytes — the unit after which MDMA re-arbitrates channels
//   block  : BNDT bytes — one linked-list entry / one repeat unit
//   repeat : BRC+1 blocks, with SAR/DAR stepped by SUV/DUV between blocks
//   channel: the whole linked list, until CLAR == 0
// TRGM selects WHICH of those levels one request (hardware or SWRQ) carries.
// ============================================================

// CxCR: PL[1:0] — priority level (arbitration between MDMA channels only;
// ties break toward the lower channel index).
enum class MdmaPriority : uint32_t {
  low      = 0,
  medium   = 1,
  high     = 2,
  veryHigh = 3,
};

// CxTCR: SSIZE[1:0] / DSIZE[1:0] — data size of one beat.
enum class MdmaDataSize : uint32_t {
  byte       = 0,
  halfWord   = 1,
  word       = 2,
  doubleWord = 3,
};

// CxTCR: SINC[1:0] / DINC[1:0] — address stepping. 01 is reserved.
enum class MdmaIncrementMode : uint32_t {
  fixed     = 0,
  increment = 2,
  decrement = 3,
};

// CxTCR: SINCOS[1:0] / DINCOS[1:0] — the step taken per beat. Must be >=
// the matching data size (SINCOS < SSIZE is undefined behaviour per the RM).
enum class MdmaIncrementOffset : uint32_t {
  byte       = 0,
  halfWord   = 1,
  word       = 2,
  doubleWord = 3,
};

// CxTCR: SBURST[2:0] / DBURST[2:0] — N encodes a burst of 2^N beats. The
// burst size must stay BELOW the buffer transfer length (TLEN+1), and the
// RM adds bus-specific constraints (see MdmaChannelConf::srcBurst).
enum class MdmaBurst : uint32_t {
  single    = 0,
  beats2    = 1,
  beats4    = 2,
  beats8    = 3,
  beats16   = 4,
  beats32   = 5,
  beats64   = 6,
  beats128  = 7,
};

// CxTCR: PAM[1:0] — how a source beat is fitted into a differently sized
// destination beat. Ignored when PKE = 1 or SSIZE == DSIZE.
enum class MdmaPaddingMode : uint32_t {
  rightZeroPad   = 0,
  rightSignExtend = 1,
  left           = 2,
};

// CxTCR: TRGM[1:0] — how much data ONE request transfers.
enum class MdmaTriggerMode : uint32_t {
  buffer        = 0,
  block         = 1,
  repeatedBlock = 2,
  wholeTransfer = 3,
};

// CxTBR: SBUS / DBUS — which master port serves this side of the transfer.
// TCM addresses MUST use ahbTcm; everything else normally uses system.
enum class MdmaBusSelect : uint32_t {
  system = 0,   // 64-bit AXI
  ahbTcm = 1,   // 32-bit AHB (Cortex-M7 AHBS → ITCM/DTCM)
};

// CxBNDTR: BRSUM / BRDUM — sign of the between-block address update. The
// update is applied to the CURRENT address (already past the block), so the
// next block starts at (block start + BNDT +/- SUV).
enum class MdmaBlockAddrUpdate : uint32_t {
  increment = 0,
  decrement = 1,
};

// CxTBR: TSEL — hardware trigger source. Ignored while SWRM = 1.
// RM0433 Rev 8 documents TSEL as 6 bits; the CMSIS mask is 8 bits wide.
enum class MdmaRequest : uint32_t {
  dma1Stream0Tc      = 0x00,
  dma1Stream1Tc      = 0x01,
  dma1Stream2Tc      = 0x02,
  dma1Stream3Tc      = 0x03,
  dma1Stream4Tc      = 0x04,
  dma1Stream5Tc      = 0x05,
  dma1Stream6Tc      = 0x06,
  dma1Stream7Tc      = 0x07,
  dma2Stream0Tc      = 0x08,
  dma2Stream1Tc      = 0x09,
  dma2Stream2Tc      = 0x0A,
  dma2Stream3Tc      = 0x0B,
  dma2Stream4Tc      = 0x0C,
  dma2Stream5Tc      = 0x0D,
  dma2Stream6Tc      = 0x0E,
  dma2Stream7Tc      = 0x0F,
  ltdcLineIt         = 0x10,
  jpegInFifoTh       = 0x11,
  jpegInFifoNotFull  = 0x12,
  jpegOutFifoTh      = 0x13,
  jpegOutFifoNotEmpty = 0x14,
  jpegEndOfConversion = 0x15,
  quadspiFifoTh      = 0x16,
  quadspiTc          = 0x17,
  dma2dClutTc        = 0x18,
  dma2dTc            = 0x19,
  dma2dTw            = 0x1A,
  dsiTearingEffect   = 0x1B,
  dsiEndOfRefresh    = 0x1C,
  sdmmc1EndOfData    = 0x1D,
  sdmmc1DmaEndBuffer = 0x1E,
  sdmmc1CommandEnd   = 0x1F,
  // Placeholder for software-triggered channels: TSEL is ignored when
  // MdmaChannelConf::softwareRequestMode is set, so the value never reaches
  // a hardware line.
  software           = 0x00,
};

// CxISR / CxIFCR flag positions (the two registers share bit numbering; the
// IFCR names in CMSIS are prefixed with C, e.g. CTEIF clears TEIF).
enum class MdmaStatus : uint32_t {
  transferError               = MDMA_CISR_TEIF,
  channelTransferComplete     = MDMA_CISR_CTCIF,
  blockRepeatTransferComplete = MDMA_CISR_BRTIF,
  blockTransferComplete       = MDMA_CISR_BTIF,
  bufferTransferComplete      = MDMA_CISR_TCIF,
};

// CxCR interrupt enables.
enum class MdmaInterrupt : uint32_t {
  transferError               = MDMA_CCR_TEIE,
  channelTransferComplete     = MDMA_CCR_CTCIE,
  blockRepeatTransferComplete = MDMA_CCR_BRTIE,
  blockTransferComplete       = MDMA_CCR_BTIE,
  bufferTransferComplete      = MDMA_CCR_TCIE,
};

// CxESR: TED — which side of the transfer faulted.
enum class MdmaErrorDirection : uint32_t {
  read  = 0,   // error on the source (read) access
  write = 1,   // error on the destination (write) access
};

// ============================================================
// Configuration structs
//
// Split the way the hardware is: the CHANNEL config is the write-protected
// part (CxTCR / CxCR / CxTBR — "can be written only if EN = 0"), the BLOCK
// config is what changes per kick (addresses, counts, block stepping).
// ============================================================

struct MdmaChannelConf {
  // --- CxTCR ---
  MdmaDataSize        srcSize       = MdmaDataSize::word;
  MdmaDataSize        dstSize       = MdmaDataSize::word;
  MdmaIncrementMode   srcInc        = MdmaIncrementMode::increment;
  MdmaIncrementMode   dstInc        = MdmaIncrementMode::increment;
  MdmaIncrementOffset srcIncOffset  = MdmaIncrementOffset::word;
  MdmaIncrementOffset dstIncOffset  = MdmaIncrementOffset::word;
  // RM0433 §14.5.5: the burst must be smaller than the buffer transfer
  // length, and single (000) is MANDATORY when the side is on the AHB/TCM
  // bus with incOffset == doubleWord, a fixed address, or incOffset !=
  // dataSize. On the AXI side with a fixed address the cap is beats16.
  MdmaBurst           srcBurst      = MdmaBurst::single;
  MdmaBurst           dstBurst      = MdmaBurst::single;
  // Buffer transfer length in BYTES (1..128) — written as TLEN = value - 1.
  // This is the re-arbitration granularity: smaller keeps other channels
  // (and other bus masters) waiting for less time.
  uint16_t            bufferBytes   = 8;
  bool                packEnable    = false;   // PKE
  MdmaPaddingMode     paddingMode   = MdmaPaddingMode::rightZeroPad;
  MdmaTriggerMode     triggerMode   = MdmaTriggerMode::repeatedBlock;
  bool                softwareRequestMode = true;   // SWRM: ignore hardware TSEL
  bool                bufferableWrite     = false;  // BWM (destination writes are never cacheable)

  // --- CxCR ---
  MdmaPriority        priority      = MdmaPriority::low;
  bool                byteEndiannessExchange     = false;  // BEX
  bool                halfWordEndiannessExchange = false;  // HEX
  bool                wordEndiannessExchange     = false;  // WEX
  bool interruptTransferError               = false;  // TEIE
  bool interruptChannelTransferComplete     = false;  // CTCIE
  bool interruptBlockRepeatTransferComplete = false;  // BRTIE
  bool interruptBlockTransferComplete       = false;  // BTIE
  bool interruptBufferTransferComplete      = false;  // TCIE

  // --- CxTBR ---
  MdmaRequest         request       = MdmaRequest::software;
  MdmaBusSelect       srcBus        = MdmaBusSelect::system;
  MdmaBusSelect       dstBus        = MdmaBusSelect::system;
};

struct MdmaBlockConf {
  uint32_t srcAddress = 0;
  uint32_t dstAddress = 0;
  // BNDT: bytes in ONE block (1..65536). Need not be a multiple of the
  // buffer length — the last buffer of a block is simply shorter.
  uint32_t blockBytes = 0;
  // BRC: number of ADDITIONAL blocks (0..4095), i.e. total = value + 1.
  uint16_t blockRepeatCount = 0;
  // SUV/DUV with sign: applied to the current (post-block) address, so a
  // strided gather of `stride` bytes with `blockBytes` per element uses
  // srcBlockOffset = stride - blockBytes. Magnitude must fit 16 bits and be
  // a multiple of the matching data size; must be 0 when that side's
  // increment mode is `fixed`.
  int32_t  srcBlockOffset = 0;
  int32_t  dstBlockOffset = 0;
  // CLAR: address of the next descriptor (0 = last block; the channel then
  // disables itself and raises CTCIF). Must live on the AXI system bus.
  uint32_t linkAddress = 0;
  // CMAR/CMDR: the hardware-request acknowledge write (write MDR to MAR).
  // Leave MAR at 0 for software-triggered memory-to-memory transfers.
  uint32_t maskAddress = 0;
  uint32_t maskData = 0;
};

// ============================================================
// MdmaChannel<CH> — one channel's full register map
//
// Usage (software-triggered gather):
//   MdmaChannel<0> ch;
//   Mdma::enableBusClock();
//   ch.abort(); ch.clearAllFlags();
//   ch.configure(conf);          // EN must be 0: every field is protected
//   ch.setBlock(block);
//   ch.start();                  // EN = 1
//   ch.trigger();                // SWRQ = 1
//   ... poll ch.isChannelTransferComplete() ...
// ============================================================

template<uint8_t CH>
struct MdmaChannel {
  static_assert(CH < 16, "MDMA channel index must be 0-15");

  constexpr static uint8_t  index = CH;
  constexpr static uint32_t addr =
      MDMA_Channel0_BASE + (uint32_t)CH * 0x40UL;

  constexpr static uint32_t addr_CISR   = addr + offsetof(MDMA_Channel_TypeDef, CISR);
  constexpr static uint32_t addr_CIFCR  = addr + offsetof(MDMA_Channel_TypeDef, CIFCR);
  constexpr static uint32_t addr_CESR   = addr + offsetof(MDMA_Channel_TypeDef, CESR);
  constexpr static uint32_t addr_CCR    = addr + offsetof(MDMA_Channel_TypeDef, CCR);
  constexpr static uint32_t addr_CTCR   = addr + offsetof(MDMA_Channel_TypeDef, CTCR);
  constexpr static uint32_t addr_CBNDTR = addr + offsetof(MDMA_Channel_TypeDef, CBNDTR);
  constexpr static uint32_t addr_CSAR   = addr + offsetof(MDMA_Channel_TypeDef, CSAR);
  constexpr static uint32_t addr_CDAR   = addr + offsetof(MDMA_Channel_TypeDef, CDAR);
  constexpr static uint32_t addr_CBRUR  = addr + offsetof(MDMA_Channel_TypeDef, CBRUR);
  constexpr static uint32_t addr_CLAR   = addr + offsetof(MDMA_Channel_TypeDef, CLAR);
  constexpr static uint32_t addr_CTBR   = addr + offsetof(MDMA_Channel_TypeDef, CTBR);
  constexpr static uint32_t addr_CMAR   = addr + offsetof(MDMA_Channel_TypeDef, CMAR);
  constexpr static uint32_t addr_CMDR   = addr + offsetof(MDMA_Channel_TypeDef, CMDR);

  // ----- CxISR: interrupt / status (read-only) -----
  RegFlagRO<addr_CISR, MDMA_CISR_TEIF>    isTransferError;
  RegFlagRO<addr_CISR, MDMA_CISR_CTCIF>   isChannelTransferComplete;
  RegFlagRO<addr_CISR, MDMA_CISR_BRTIF>   isBlockRepeatTransferComplete;
  RegFlagRO<addr_CISR, MDMA_CISR_BTIF>    isBlockTransferComplete;
  RegFlagRO<addr_CISR, MDMA_CISR_TCIF>    isBufferTransferComplete;
  RegFlagRO<addr_CISR, MDMA_CISR_CRQA>    isRequestActive;

  // ----- CxIFCR: flag clear (write-only) -----
  RegFlagWO<addr_CIFCR, MDMA_CIFCR_CTEIF>  clearTransferError;
  RegFlagWO<addr_CIFCR, MDMA_CIFCR_CCTCIF> clearChannelTransferComplete;
  RegFlagWO<addr_CIFCR, MDMA_CIFCR_CBRTIF> clearBlockRepeatTransferComplete;
  RegFlagWO<addr_CIFCR, MDMA_CIFCR_CBTIF>  clearBlockTransferComplete;
  RegFlagWO<addr_CIFCR, MDMA_CIFCR_CLTCIF> clearBufferTransferComplete;

  // ----- CxESR: error status (read-only) -----
  RegValueRO<addr_CESR, MDMA_CESR_TEA_Msk, MDMA_CESR_TEA_Pos> transferErrorAddress;
  RegEnumRO<addr_CESR, MDMA_CESR_TED_Msk,
            MdmaErrorDirection, MDMA_CESR_TED_Pos>            transferErrorDirection;
  RegFlagRO<addr_CESR, MDMA_CESR_TELD>  isTransferErrorLinkData;
  RegFlagRO<addr_CESR, MDMA_CESR_TEMD>  isTransferErrorMaskData;
  RegFlagRO<addr_CESR, MDMA_CESR_ASE>   isAddressSizeError;
  RegFlagRO<addr_CESR, MDMA_CESR_BSE>   isBlockSizeError;

  // ----- CxCR: control -----
  // EN reads back low once the channel has actually stopped (RM: "flag
  // channel ready when read low") — abort() writes 0 without waiting.
  RegFlag<addr_CCR, MDMA_CCR_EN>    enable;
  RegFlag<addr_CCR, MDMA_CCR_TEIE>  enableItTransferError;
  RegFlag<addr_CCR, MDMA_CCR_CTCIE> enableItChannelTransferComplete;
  RegFlag<addr_CCR, MDMA_CCR_BRTIE> enableItBlockRepeatTransferComplete;
  RegFlag<addr_CCR, MDMA_CCR_BTIE>  enableItBlockTransferComplete;
  RegFlag<addr_CCR, MDMA_CCR_TCIE>  enableItBufferTransferComplete;
  RegEnum<addr_CCR, MDMA_CCR_PL_Msk, MdmaPriority, MDMA_CCR_PL_Pos> priority;
  RegFlag<addr_CCR, MDMA_CCR_BEX>   byteEndiannessExchange;
  RegFlag<addr_CCR, MDMA_CCR_HEX>   halfWordEndiannessExchange;
  RegFlag<addr_CCR, MDMA_CCR_WEX>   wordEndiannessExchange;
  RegFlagWO<addr_CCR, MDMA_CCR_SWRQ> softwareRequest;

  // ----- CxTCR: transfer configuration (all fields need EN = 0) -----
  RegEnum<addr_CTCR, MDMA_CTCR_SINC_Msk,
          MdmaIncrementMode, MDMA_CTCR_SINC_Pos>       srcIncrementMode;
  RegEnum<addr_CTCR, MDMA_CTCR_DINC_Msk,
          MdmaIncrementMode, MDMA_CTCR_DINC_Pos>       dstIncrementMode;
  RegEnum<addr_CTCR, MDMA_CTCR_SSIZE_Msk,
          MdmaDataSize, MDMA_CTCR_SSIZE_Pos>           srcDataSize;
  RegEnum<addr_CTCR, MDMA_CTCR_DSIZE_Msk,
          MdmaDataSize, MDMA_CTCR_DSIZE_Pos>           dstDataSize;
  RegEnum<addr_CTCR, MDMA_CTCR_SINCOS_Msk,
          MdmaIncrementOffset, MDMA_CTCR_SINCOS_Pos>   srcIncrementOffset;
  RegEnum<addr_CTCR, MDMA_CTCR_DINCOS_Msk,
          MdmaIncrementOffset, MDMA_CTCR_DINCOS_Pos>   dstIncrementOffset;
  RegEnum<addr_CTCR, MDMA_CTCR_SBURST_Msk,
          MdmaBurst, MDMA_CTCR_SBURST_Pos>             srcBurst;
  RegEnum<addr_CTCR, MDMA_CTCR_DBURST_Msk,
          MdmaBurst, MDMA_CTCR_DBURST_Pos>             dstBurst;
  RegValue<addr_CTCR, MDMA_CTCR_TLEN_Msk,
           MDMA_CTCR_TLEN_Pos>                         bufferTransferLength;  // bytes - 1
  RegFlag<addr_CTCR, MDMA_CTCR_PKE>                    packEnable;
  RegEnum<addr_CTCR, MDMA_CTCR_PAM_Msk,
          MdmaPaddingMode, MDMA_CTCR_PAM_Pos>          paddingMode;
  RegEnum<addr_CTCR, MDMA_CTCR_TRGM_Msk,
          MdmaTriggerMode, MDMA_CTCR_TRGM_Pos>         triggerMode;
  RegFlag<addr_CTCR, MDMA_CTCR_SWRM>                   softwareRequestMode;
  RegFlag<addr_CTCR, MDMA_CTCR_BWM>                    bufferableWrite;

  // ----- CxBNDTR: block size / repeat -----
  // BNDT is read-only while the channel is enabled: it then reports the
  // bytes still outstanding in the current block.
  RegValue<addr_CBNDTR, MDMA_CBNDTR_BNDT_Msk,
           MDMA_CBNDTR_BNDT_Pos>                       blockDataBytes;
  RegEnum<addr_CBNDTR, MDMA_CBNDTR_BRSUM_Msk,
          MdmaBlockAddrUpdate, MDMA_CBNDTR_BRSUM_Pos>  blockRepeatSrcUpdate;
  RegEnum<addr_CBNDTR, MDMA_CBNDTR_BRDUM_Msk,
          MdmaBlockAddrUpdate, MDMA_CBNDTR_BRDUM_Pos>  blockRepeatDstUpdate;
  RegValue<addr_CBNDTR, MDMA_CBNDTR_BRC_Msk,
           MDMA_CBNDTR_BRC_Pos>                        blockRepeatCount;

  // ----- CxSAR / CxDAR / CxBRUR / CxLAR -----
  Reg<addr_CSAR>  srcAddress;
  Reg<addr_CDAR>  dstAddress;
  RegValue<addr_CBRUR, MDMA_CBRUR_SUV_Msk, MDMA_CBRUR_SUV_Pos> srcUpdateValue;
  RegValue<addr_CBRUR, MDMA_CBRUR_DUV_Msk, MDMA_CBRUR_DUV_Pos> dstUpdateValue;
  Reg<addr_CLAR> linkAddress;

  // ----- CxTBR: trigger / bus selection -----
  RegValue<addr_CTBR, MDMA_CTBR_TSEL_Msk, MDMA_CTBR_TSEL_Pos>  triggerSelection;
  RegEnum<addr_CTBR, MDMA_CTBR_SBUS_Msk,
          MdmaBusSelect, MDMA_CTBR_SBUS_Pos>                   srcBus;
  RegEnum<addr_CTBR, MDMA_CTBR_DBUS_Msk,
          MdmaBusSelect, MDMA_CTBR_DBUS_Pos>                   dstBus;

  // ----- CxMAR / CxMDR: hardware-request acknowledge write -----
  Reg<addr_CMAR> maskAddress;
  Reg<addr_CMDR> maskData;

  // ----- NVIC (one line shared by all 16 channels) -----
  constexpr static IRQn_Type irqn = MDMA_IRQn;

  // ============================================================
  // High-level helpers
  // ============================================================

  // Write the protected configuration. EN must already be 0 (RM0433 §14.5:
  // "These bits are protected and can be written only if EN is 0"), and the
  // RM additionally requires waiting for CTCIF before reprogramming a
  // channel that was running — see abort() / isIdle().
  IGB_FAST_INLINE void configure(const MdmaChannelConf& conf) {
    (srcIncrementMode.val(conf.srcInc)
      | dstIncrementMode.val(conf.dstInc)
      | srcDataSize.val(conf.srcSize)
      | dstDataSize.val(conf.dstSize)
      | srcIncrementOffset.val(conf.srcIncOffset)
      | dstIncrementOffset.val(conf.dstIncOffset)
      | srcBurst.val(conf.srcBurst)
      | dstBurst.val(conf.dstBurst)
      | bufferTransferLength.val((uint32_t)conf.bufferBytes - 1u)
      | packEnable.val(conf.packEnable)
      | paddingMode.val(conf.paddingMode)
      | triggerMode.val(conf.triggerMode)
      | softwareRequestMode.val(conf.softwareRequestMode)
      | bufferableWrite.val(conf.bufferableWrite)
    ).update();

    (triggerSelection.val(static_cast<uint32_t>(conf.request))
      | srcBus.val(conf.srcBus)
      | dstBus.val(conf.dstBus)
    ).update();

    // EN is deliberately NOT part of this chain: start() owns it, so a
    // configure() can never launch a transfer by accident.
    (priority.val(conf.priority)
      | byteEndiannessExchange.val(conf.byteEndiannessExchange)
      | halfWordEndiannessExchange.val(conf.halfWordEndiannessExchange)
      | wordEndiannessExchange.val(conf.wordEndiannessExchange)
      | enableItTransferError.val(conf.interruptTransferError)
      | enableItChannelTransferComplete.val(conf.interruptChannelTransferComplete)
      | enableItBlockRepeatTransferComplete.val(conf.interruptBlockRepeatTransferComplete)
      | enableItBlockTransferComplete.val(conf.interruptBlockTransferComplete)
      | enableItBufferTransferComplete.val(conf.interruptBufferTransferComplete)
    ).update();
  }

  // Addresses, sizes and the between-block stepping. Also EN = 0 only.
  IGB_FAST_INLINE void setBlock(const MdmaBlockConf& conf) {
    srcAddress(conf.srcAddress);
    dstAddress(conf.dstAddress);

    const bool s_dec = conf.srcBlockOffset < 0;
    const bool d_dec = conf.dstBlockOffset < 0;
    const uint32_t suv =
        (uint32_t)(s_dec ? -conf.srcBlockOffset : conf.srcBlockOffset);
    const uint32_t duv =
        (uint32_t)(d_dec ? -conf.dstBlockOffset : conf.dstBlockOffset);
    (srcUpdateValue.val(suv) | dstUpdateValue.val(duv)).update();

    (blockDataBytes.val(conf.blockBytes)
      | blockRepeatCount.val((uint32_t)conf.blockRepeatCount)
      | blockRepeatSrcUpdate.val(s_dec ? MdmaBlockAddrUpdate::decrement
                                       : MdmaBlockAddrUpdate::increment)
      | blockRepeatDstUpdate.val(d_dec ? MdmaBlockAddrUpdate::decrement
                                       : MdmaBlockAddrUpdate::increment)
    ).update();

    linkAddress(conf.linkAddress);
    maskAddress(conf.maskAddress);
    maskData(conf.maskData);
  }

  IGB_FAST_INLINE void start()   { enable(true); }
  IGB_FAST_INLINE void trigger() { softwareRequest(true); }

  // Request a stop. RM0433 §14.3.14: clearing EN does NOT stop the channel
  // immediately — the buffer transfer in flight completes first, and CTCIF
  // is then raised. Callers on a real-time path must NOT spin here; poll
  // isIdle() later instead.
  IGB_FAST_INLINE void abort() { enable(false); }

  // True once the hardware has actually released the channel.
  IGB_FAST_INLINE bool isIdle() { return !enable(); }

  // Bytes still outstanding in the current block (BNDT is read-only while
  // the channel runs).
  IGB_FAST_INLINE uint32_t remainingBytes() { return blockDataBytes(); }

  IGB_FAST_INLINE bool isAnyError() {
    return isTransferError() || isAddressSizeError() || isBlockSizeError();
  }

  // CxIFCR is write-1-to-clear only: a plain store is correct (and avoids a
  // pointless read-modify-write of a register that reads as 0).
  IGB_FAST_INLINE void clearAllFlags() {
    (*(volatile uint32_t*)addr_CIFCR) =
        MDMA_CIFCR_CTEIF | MDMA_CIFCR_CCTCIF | MDMA_CIFCR_CBRTIF
        | MDMA_CIFCR_CBTIF | MDMA_CIFCR_CLTCIF;
  }

  IGB_FAST_INLINE void enableIt(MdmaInterrupt interrupt) {
    IGB_SET_BIT(*(volatile uint32_t*)addr_CCR, static_cast<uint32_t>(interrupt));
  }

  IGB_FAST_INLINE void disableIt(MdmaInterrupt interrupt) {
    IGB_CLEAR_BIT(*(volatile uint32_t*)addr_CCR, static_cast<uint32_t>(interrupt));
  }

  IGB_FAST_INLINE bool is(MdmaStatus status) {
    return (*(volatile uint32_t*)addr_CISR) & static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void clear(MdmaStatus status) {
    // CxIFCR mirrors CxISR bit for bit (CTEIF clears TEIF, and so on).
    (*(volatile uint32_t*)addr_CIFCR) = static_cast<uint32_t>(status);
  }
};

// ============================================================
// Mdma — controller-level access
// ============================================================

struct Mdma {
  constexpr static uint32_t addr = MDMA_BASE;
  constexpr static uint32_t addr_GISR0 = addr + offsetof(MDMA_TypeDef, GISR0);

  // Bit x = channel x has a pending (unmasked) interrupt.
  RegRO<addr_GISR0> globalInterruptStatus;

  constexpr static IRQn_Type irqn = MDMA_IRQn;

  // MDMA sits on AHB3 (RCC_AHB3ENR bit 0). Not present in the generated
  // STM32_PERIPH_INFO table, so the bus descriptor is spelled out here — the
  // same shortcut dma_stream.hpp takes for the stream/DMAMUX addresses.
  constexpr static PeriphBusInfo bus { BusType::ahb3, RCC_AHB3ENR_MDMAEN };

  IGB_FAST_INLINE static void enableBusClock()       { bus.enableBusClock(); }
  IGB_FAST_INLINE static void disableBusClock()      { bus.disableBusClock(); }
  IGB_FAST_INLINE static void forceResetBusClock()   { bus.forceResetBusClock(); }
  IGB_FAST_INLINE static void releaseResetBusClock() { bus.releaseResetBusClock(); }

  IGB_FAST_INLINE static bool isChannelInterrupt(uint8_t ch) {
    return MDMA->GISR0 & (1UL << ch);
  }

  IGB_FAST_INLINE static void enableNvic(uint32_t priority = 1) {
    NvicCtrl::setPriority(irqn, priority);
    NvicCtrl::enable(irqn);
  }

  IGB_FAST_INLINE static void disableNvic() { NvicCtrl::disable(irqn); }
};

} // namespace stm32
} // namespace igb

#endif // STM32_PERIPH_MDMA_EXISTS

#endif // STM32H7
