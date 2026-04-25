#pragma once

// SD card driver over STM32H7 SDMMC1 (single-instance, IDMA + interrupt-driven
// data-phase completion).
//
// GPIO: PC8(D0), PC9(D1), PC10(D2), PC11(D3), PC12(CK), PD2(CMD) — AF12
// Clock: PLL2R (200 MHz kernel), CLKDIV sets card clock
// Transfer: IDMA (Internal DMA) with non-cacheable AXI SRAM buffer
//
// IDMA transfers to/from a caller-provided 4 KB buffer in non-cacheable AXI
// SRAM (must be allocated by the application, e.g. via linker section). Data
// is memcpy'd between the DMA buffer and caller-provided application buffers.

#include <cstdint>
#include <cstring>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/sdmmc.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/systick.hpp>
#include <igb_util/macro.hpp>

namespace igb::sdk {

constexpr uint32_t SDMMC_DMA_MAX_BLOCKS = 8;

// SD card command indices
namespace SdCmd {
  constexpr uint8_t GO_IDLE        = 0;
  constexpr uint8_t SEND_IF_COND   = 8;
  constexpr uint8_t SEND_CSD       = 9;
  constexpr uint8_t SEND_CID       = 2;
  constexpr uint8_t SEND_STATUS    = 13;
  constexpr uint8_t SET_BLOCKLEN   = 16;
  constexpr uint8_t READ_SINGLE    = 17;
  constexpr uint8_t READ_MULTI     = 18;
  constexpr uint8_t WRITE_SINGLE   = 24;
  constexpr uint8_t WRITE_MULTI    = 25;
  constexpr uint8_t APP_CMD        = 55;
  constexpr uint8_t SET_REL_ADDR   = 3;
  constexpr uint8_t SEL_DESEL      = 7;
  constexpr uint8_t STOP_TRANS     = 12;
}
namespace SdAcmd {
  constexpr uint8_t SET_BUSWIDTH   = 6;
  constexpr uint8_t SD_APP_OP_COND = 41;
}

struct SdCardInfo {
  uint32_t cardType    = 0; // 1=SDSC, 2=SDHC/SDXC
  uint32_t rca         = 0; // relative card address
  uint32_t blockCount  = 0;
  uint32_t blockSize   = 512;
};

// dma_buf must point to a 4 KB-aligned buffer in non-cacheable memory
// (8 sectors × 512 B = 1024 × uint32_t).
struct SdCard {
  igb::stm32::Sdmmc<igb::stm32::SdmmcType::sdmmc1> sd;
  SdCardInfo info;
  uint32_t* const dma_buf;

  explicit SdCard(uint32_t* dma_buf_) : dma_buf(dma_buf_) {}

  // ---- Async transfer state ----
  // begin* → isTransferComplete() → endTransfer() pattern.
  // Data phase completion (DATAEND) is interrupt-driven; command response
  // and waitReady remain synchronous short busy-waits (microseconds) for simplicity.
  enum class TransferState : uint8_t {
    idle,
    busy_data,    // IDMA data phase in progress, DATAEND IT pending
    complete,
    error,
  };
  volatile TransferState _xfer_state = TransferState::idle;
  volatile uint32_t _xfer_err_flags = 0;  // snapshot of STA error bits on error
  uint32_t _xfer_start_msec = 0;
  bool _xfer_is_multi = false;
  bool _xfer_is_read = false;
  uint32_t* _xfer_read_dest = nullptr;    // caller buffer for read memcpy on complete
  uint32_t _xfer_bytes = 0;

  // ---- GPIO ----
  void initGpio(igb::stm32::GpioSpeedMode speed) {
    constexpr igb::stm32::GpioPinType pins[] = {
      igb::stm32::GpioPinType::pc8,  // D0
      igb::stm32::GpioPinType::pc9,  // D1
      igb::stm32::GpioPinType::pc10, // D2
      igb::stm32::GpioPinType::pc11, // D3
      igb::stm32::GpioPinType::pc12, // CK
      igb::stm32::GpioPinType::pd2,  // CMD
    };
    for (auto pt : pins) {
      sd.prepareGpio(pt);
      auto pin = igb::stm32::GpioPin::newPin(pt);
      pin.setSpeedMode(speed);
    }
  }

  // ---- Low-level command ----
  bool sendCommand(uint8_t idx, uint32_t arg, igb::stm32::SdmmcWaitResp resp, uint32_t timeout_ms = 1000) {
    sd.clearAllFlags();
    sd.setArg(arg);
    sd.sendCommand({
      .cmdIndex = idx,
      .waitResp = resp,
      .cpsmEn   = true,
    });

    uint32_t start = current_msec();
    if (resp == igb::stm32::SdmmcWaitResp::none) {
      while (!sd.isCmdSent()) {
        if (current_msec() - start > timeout_ms) return false;
      }
    } else {
      while (true) {
        if (sd.isCmdRend())  return true;
        if (sd.isCTimeout()) return false;
        if (sd.isCCrcFail()) {
          return (resp == igb::stm32::SdmmcWaitResp::shortNoCrc);
        }
        if (current_msec() - start > timeout_ms) return false;
      }
    }
    return true;
  }

  bool sendAppCommand(uint8_t acmd, uint32_t arg, igb::stm32::SdmmcWaitResp resp) {
    if (!sendCommand(SdCmd::APP_CMD, info.rca << 16, igb::stm32::SdmmcWaitResp::shortCrc))
      return false;
    return sendCommand(acmd, arg, resp);
  }

  // ---- Card initialization ----
  bool init() {
    sd.enableBusClock();

    // Clear residual SDMMC state (bootloader may have left it configured)
    sd.disableIdma();
    sd.idmaDoubleBuffer(false);
    sd.idmaBase0(0);
    sd.idmaBase1(0);
    sd.clearAllFlags();
    sd.disableAllInterrupts();
    NVIC_DisableIRQ(SDMMC1_IRQn);
    NVIC_ClearPendingIRQ(SDMMC1_IRQn);

    // Start with slow clock (400 kHz) for initialization
    // SDMMC_CK = kernel_clk / (2 * CLKDIV) = 200MHz / (2*250) = 400kHz
    initGpio(igb::stm32::GpioSpeedMode::low);
    sd.powerOn();
    delay_msec(2);
    sd.initClock({ .clkDiv = 250, .busWidth = igb::stm32::SdmmcBusWidth::_1bit });
    delay_msec(2);

    // CMD0: GO_IDLE_STATE
    if (!sendCommand(SdCmd::GO_IDLE, 0, igb::stm32::SdmmcWaitResp::none))
      return false;

    // CMD8: SEND_IF_COND (voltage check, pattern 0xAA)
    bool v2card = sendCommand(SdCmd::SEND_IF_COND, 0x000001AA, igb::stm32::SdmmcWaitResp::shortCrc);
    if (v2card) {
      uint32_t r7 = sd.getResp1();
      if ((r7 & 0xFF) != 0xAA) return false;
    }

    // ACMD41 loop: negotiate operating conditions
    uint32_t ocr_arg = 0x80100000; // 3.2-3.4V window
    if (v2card) ocr_arg |= 0x40000000; // HCS (High Capacity Support)
    info.rca = 0;

    for (uint32_t i = 0; i < 0xFFFF; ++i) {
      if (!sendAppCommand(SdAcmd::SD_APP_OP_COND, ocr_arg, igb::stm32::SdmmcWaitResp::shortNoCrc))
        return false;
      uint32_t ocr = sd.getResp1();
      if (ocr & 0x80000000) { // Card ready
        info.cardType = (ocr & 0x40000000) ? 2 : 1; // SDHC=2, SDSC=1
        break;
      }
      if (i == 0xFFFE) return false; // timeout
    }

    // CMD2: ALL_SEND_CID
    if (!sendCommand(SdCmd::SEND_CID, 0, igb::stm32::SdmmcWaitResp::longResp))
      return false;

    // CMD3: SET_RELATIVE_ADDR
    if (!sendCommand(SdCmd::SET_REL_ADDR, 0, igb::stm32::SdmmcWaitResp::shortCrc))
      return false;
    info.rca = (sd.getResp1() >> 16) & 0xFFFF;

    // CMD9: SEND_CSD (get card capacity)
    if (!sendCommand(SdCmd::SEND_CSD, info.rca << 16, igb::stm32::SdmmcWaitResp::longResp))
      return false;
    parseCsd();

    // CMD7: SELECT_CARD
    if (!sendCommand(SdCmd::SEL_DESEL, info.rca << 16, igb::stm32::SdmmcWaitResp::shortCrc))
      return false;

    // Switch to 4-bit bus
    if (!sendAppCommand(SdAcmd::SET_BUSWIDTH, 0x00000002, igb::stm32::SdmmcWaitResp::shortCrc))
      return false;

    // Increase clock speed: 200MHz / (2*4) = 25 MHz
    initGpio(igb::stm32::GpioSpeedMode::veryHigh);
    sd.initClock({
      .clkDiv   = 4,
      .busWidth = igb::stm32::SdmmcBusWidth::_4bit,
      .hwFlowCtrl = true,
    });

    // Set block length to 512 (required for SDSC; SDHC ignores)
    if (info.cardType == 1) {
      sendCommand(SdCmd::SET_BLOCKLEN, 512, igb::stm32::SdmmcWaitResp::shortCrc);
    }

    // Enable SDMMC1 NVIC line for async transfer completion.
    // Peripheral-level IT mask is toggled per-transfer by _beginDataTransfer().
    sd.enableNvic(2);

    return true;
  }

  // ---- Parse CSD register for capacity ----
  void parseCsd() {
    uint32_t r1 = sd.getResp1(), r2 = sd.getResp2();
    uint32_t r3 = sd.getResp3();
    uint8_t csd_struct = (r1 >> 30) & 0x03;
    if (csd_struct == 1) {
      uint32_t c_size = ((r2 & 0x3F) << 16) | ((r3 >> 16) & 0xFFFF);
      info.blockCount = (c_size + 1) * 1024;
      info.blockSize  = 512;
    } else {
      uint32_t read_bl_len = (r2 >> 16) & 0x0F;
      uint32_t c_size = ((r2 & 0x03FF) << 2) | ((r3 >> 30) & 0x03);
      uint32_t c_size_mult = (r3 >> 15) & 0x07;
      uint32_t capacity = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << read_bl_len);
      info.blockCount = capacity / 512;
      info.blockSize  = 512;
    }
  }

  // ---- Wait for card ready (not busy) ----
  bool waitReady(uint32_t timeout_ms = 5000) {
    uint32_t start = current_msec();
    while (current_msec() - start < timeout_ms) {
      if (!sendCommand(SdCmd::SEND_STATUS, info.rca << 16, igb::stm32::SdmmcWaitResp::shortCrc))
        continue;
      uint32_t status = sd.getResp1();
      uint32_t state = (status >> 9) & 0x0F;
      if (state == 4) return true; // TRAN state
      if (state == 5 || state == 6) continue; // DATA/RCV — still busy
    }
    return false;
  }

  // ---- Async transfer: issue data command, return immediately ----
  // DATAEND completion is delivered via SDMMC1 IRQ (see handleIrq()).
  // Returns false on immediate failure (bad args, state, or cmd response timeout).
  bool _beginDataTransfer(uint32_t card_addr, uint32_t count, bool is_read) {
    if (_xfer_state != TransferState::idle) return false;
    if (count == 0 || count > SDMMC_DMA_MAX_BLOCKS) return false;

    _xfer_is_multi = (count > 1);
    _xfer_is_read = is_read;
    _xfer_err_flags = 0;
    _xfer_bytes = count * 512;
    _xfer_start_msec = current_msec();

    sd.clearAllFlags();
    sd.disableAllInterrupts();
    sd.dataTimer(0xFFFFFFFF);
    sd.dataLength(_xfer_bytes);
    sd.idmaBase0(reinterpret_cast<uint32_t>(dma_buf));
    *reinterpret_cast<volatile uint32_t*>(sd.addr_IDMACTRL) = SDMMC_IDMA_IDMAEN;
    *reinterpret_cast<volatile uint32_t*>(sd.addr_DCTRL) =
        (is_read ? SDMMC_DCTRL_DTDIR : 0)
      | (9U << SDMMC_DCTRL_DBLOCKSIZE_Pos);

    uint8_t cmd_idx = is_read
      ? (_xfer_is_multi ? SdCmd::READ_MULTI  : SdCmd::READ_SINGLE)
      : (_xfer_is_multi ? SdCmd::WRITE_MULTI : SdCmd::WRITE_SINGLE);
    sd.setArg(card_addr);
    sd.sendCommand({
      .cmdIndex = cmd_idx,
      .cmdTrans = true,
      .waitResp = igb::stm32::SdmmcWaitResp::shortCrc,
      .cpsmEn   = true,
    });

    // Cmd response wait (synchronous, typically microseconds).
    while (!sd.isCmdRend() && !sd.isCTimeout() && !sd.isCCrcFail()) {
      if (current_msec() - _xfer_start_msec > 1000) {
        sd.disableIdma();
        _xfer_state = TransferState::error;
        return false;
      }
    }
    if (sd.isCTimeout() || sd.isCCrcFail()) {
      sd.disableIdma();
      _xfer_state = TransferState::error;
      return false;
    }

    _xfer_state = TransferState::busy_data;

    // Enable DATAEND + data-phase error interrupts. DATAEND is level-sticky;
    // if the transfer already completed before we get here, the IT fires
    // immediately on enable.
    sd.enableIt(igb::stm32::SdmmcInterruptType::dataEnd);
    sd.enableIt(igb::stm32::SdmmcInterruptType::dCrcFail);
    sd.enableIt(igb::stm32::SdmmcInterruptType::dTimeout);
    if (is_read) {
      sd.enableIt(igb::stm32::SdmmcInterruptType::rxOverr);
    } else {
      sd.enableIt(igb::stm32::SdmmcInterruptType::txUnderr);
    }
    return true;
  }

  // Begin a write of up to SDMMC_DMA_MAX_BLOCKS blocks. Caller data is copied
  // into the IDMA buffer synchronously inside this call; after return the caller
  // may modify `buf`. Use isTransferComplete() / endTransfer() to finalize.
  bool beginWriteBlocks(const uint32_t* buf, uint32_t block_addr, uint32_t count) {
    if (_xfer_state != TransferState::idle) return false;
    if (count == 0 || count > SDMMC_DMA_MAX_BLOCKS) return false;
    std::memcpy(dma_buf, buf, count * 512);
    uint32_t card_addr = block_addr;
    if (info.cardType == 1) card_addr *= 512;
    _xfer_read_dest = nullptr;
    return _beginDataTransfer(card_addr, count, false);
  }

  // Begin a read of up to SDMMC_DMA_MAX_BLOCKS blocks. `buf` must remain valid
  // until endTransfer() returns (memcpy from IDMA buffer happens on finalize).
  bool beginReadBlocks(uint32_t* buf, uint32_t block_addr, uint32_t count) {
    if (_xfer_state != TransferState::idle) return false;
    if (count == 0 || count > SDMMC_DMA_MAX_BLOCKS) return false;
    uint32_t card_addr = block_addr;
    if (info.cardType == 1) card_addr *= 512;
    _xfer_read_dest = buf;
    return _beginDataTransfer(card_addr, count, true);
  }

  // Non-blocking transfer completion check. Returns true when the data phase
  // finished (success or error). Callers should then invoke endTransfer() to
  // finalize and read the result.
  IGB_FAST_INLINE bool isTransferComplete() {
    if (_xfer_state == TransferState::busy_data) {
      // Soft timeout in case IRQ never arrived (e.g., IDMATE not raising
      // an enabled data-phase interrupt). Escalate to error.
      if (current_msec() - _xfer_start_msec > 5000) {
        sd.disableAllInterrupts();
        sd.disableIdma();
        _xfer_err_flags = 0xFFFFFFFF;
        _xfer_state = TransferState::error;
      }
    }
    return _xfer_state == TransferState::complete || _xfer_state == TransferState::error;
  }

  // Finalize a transfer that isTransferComplete() reported done.
  // Sends STOP_TRANS if multi-block, memcpy for reads, waitReady for writes,
  // then returns to idle. Returns overall success.
  bool endTransfer() {
    if (_xfer_state != TransferState::complete && _xfer_state != TransferState::error) {
      return false;
    }
    bool success = (_xfer_state == TransferState::complete);

    if (_xfer_is_multi) {
      sendCommand(SdCmd::STOP_TRANS, 0, igb::stm32::SdmmcWaitResp::shortCrc);
    }
    if (_xfer_is_read && success && _xfer_read_dest) {
      std::memcpy(_xfer_read_dest, dma_buf, _xfer_bytes);
    }
    _xfer_read_dest = nullptr;

    sd.clearAllFlags();

    // Writes must wait for the card to leave the RCV/DATA state before the
    // next command can be issued. Reads don't need this.
    bool ready_ok = true;
    if (!_xfer_is_read && success) {
      ready_ok = waitReady();
    }

    _xfer_state = TransferState::idle;
    return success && ready_ok;
  }

  // Called from SDMMC1_IRQHandler. Transitions busy_data → complete/error.
  IGB_FAST_INLINE void handleIrq() {
    if (_xfer_state != TransferState::busy_data) {
      // Spurious or state already advanced; clear flags and exit.
      sd.disableAllInterrupts();
      sd.clearAllFlags();
      return;
    }
    uint32_t sta = *reinterpret_cast<volatile uint32_t*>(sd.addr_STA);
    constexpr uint32_t err_mask =
        SDMMC_STA_DCRCFAIL_Msk | SDMMC_STA_DTIMEOUT_Msk
      | SDMMC_STA_TXUNDERR_Msk | SDMMC_STA_RXOVERR_Msk
      | SDMMC_STA_IDMATE_Msk;
    if (sta & err_mask) {
      _xfer_err_flags = sta & err_mask;
      sd.disableAllInterrupts();
      sd.disableIdma();
      sd.clearAllFlags();
      _xfer_state = TransferState::error;
    } else if (sta & SDMMC_STA_DATAEND_Msk) {
      sd.disableAllInterrupts();
      sd.disableIdma();
      _xfer_state = TransferState::complete;
      // Leave ICR flag clearing to endTransfer() to keep IRQ minimal.
    }
  }

  // ---- Synchronous wrappers (for FatFs diskio and legacy callers) ----
  bool readBlocks(uint32_t* buf, uint32_t block_addr, uint32_t count) {
    uint32_t blocks_done = 0;
    while (blocks_done < count) {
      uint32_t chunk = count - blocks_done;
      if (chunk > SDMMC_DMA_MAX_BLOCKS) chunk = SDMMC_DMA_MAX_BLOCKS;
      uint32_t* chunk_buf = reinterpret_cast<uint32_t*>(
          reinterpret_cast<uint8_t*>(buf) + blocks_done * 512);
      if (!beginReadBlocks(chunk_buf, block_addr + blocks_done, chunk)) return false;
      while (!isTransferComplete()) { __WFI(); }
      if (!endTransfer()) return false;
      blocks_done += chunk;
    }
    return true;
  }

  bool writeBlocks(const uint32_t* buf, uint32_t block_addr, uint32_t count) {
    uint32_t blocks_done = 0;
    while (blocks_done < count) {
      uint32_t chunk = count - blocks_done;
      if (chunk > SDMMC_DMA_MAX_BLOCKS) chunk = SDMMC_DMA_MAX_BLOCKS;
      const uint32_t* chunk_buf = reinterpret_cast<const uint32_t*>(
          reinterpret_cast<const uint8_t*>(buf) + blocks_done * 512);
      if (!beginWriteBlocks(chunk_buf, block_addr + blocks_done, chunk)) return false;
      while (!isTransferComplete()) { __WFI(); }
      if (!endTransfer()) return false;
      blocks_done += chunk;
    }
    return true;
  }
};

}  // namespace igb::sdk
