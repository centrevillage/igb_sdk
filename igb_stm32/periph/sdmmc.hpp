#pragma once

#if defined(STM32H7)

#ifdef STM32_PERIPHGRP_SDMMC_EXISTS

#include <stddef.h>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

// ============================================================
// SDMMC enums
// ============================================================

// POWER: PWRCTRL[1:0] — power supply control
enum class SdmmcPowerCtrl : uint32_t {
  off   = 0,
  cycle = 2,
  on    = 3,
};

// CLKCR: WIDBUS[1:0] — wide bus mode
enum class SdmmcBusWidth : uint32_t {
  _1bit = 0,
  _4bit = 1,
  _8bit = 2,
};

// CLKCR: SELCLKRX[1:0] — receive clock selection
enum class SdmmcRxClkSel : uint32_t {
  ioIn   = 0,
  ckinFb = 1,
  idmaFb = 2,
};

// CMD: WAITRESP[1:0] — wait for response
enum class SdmmcWaitResp : uint32_t {
  none       = 0,
  shortCrc   = 1,
  shortNoCrc = 2,
  longResp   = 3,
};

// DCTRL: DTMODE[1:0] — data transfer mode
enum class SdmmcDataMode : uint32_t {
  block     = 0,
  sdio      = 1,
  emmc      = 2,
  blockStop = 3,
};

// DCTRL: DBLOCKSIZE[3:0] — data block size (2^n bytes)
enum class SdmmcBlockSize : uint32_t {
  _1byte     = 0,
  _2byte     = 1,
  _4byte     = 2,
  _8byte     = 3,
  _16byte    = 4,
  _32byte    = 5,
  _64byte    = 6,
  _128byte   = 7,
  _256byte   = 8,
  _512byte   = 9,
  _1024byte  = 10,
  _2048byte  = 11,
  _4096byte  = 12,
  _8192byte  = 13,
  _16384byte = 14,
};

// ============================================================
// SDMMC conf structs
// ============================================================

struct SdmmcClockConf {
  uint16_t       clkDiv     = 0;
  bool           pwrSav     = false;
  SdmmcBusWidth  busWidth   = SdmmcBusWidth::_1bit;
  bool           negEdge    = false;
  bool           hwFlowCtrl = false;
  bool           ddr        = false;
  bool           busSpeed   = false;
  SdmmcRxClkSel  rxClkSel   = SdmmcRxClkSel::ioIn;
};

struct SdmmcCmdConf {
  uint8_t        cmdIndex   = 0;
  bool           cmdTrans   = false;
  bool           cmdStop    = false;
  SdmmcWaitResp  waitResp   = SdmmcWaitResp::none;
  bool           waitInt    = false;
  bool           waitPend   = false;
  bool           cpsmEn     = true;
  bool           dtHold     = false;
  bool           bootMode   = false;
  bool           bootEn     = false;
  bool           cmdSuspend = false;
};

struct SdmmcDataConf {
  uint32_t       dataTimeout = 0xFFFFFFFF;
  uint32_t       dataLength  = 0;
  bool           dataEn      = false;
  bool           dataDir     = false;
  SdmmcDataMode  dataMode    = SdmmcDataMode::block;
  SdmmcBlockSize blockSize   = SdmmcBlockSize::_512byte;
  bool           rwStart     = false;
  bool           rwStop      = false;
  bool           rwMod       = false;
  bool           sdioEn      = false;
  bool           bootAckEn   = false;
  bool           fifoRst     = false;
};

// ============================================================
// Sdmmc: SDMMC peripheral register wrapper
//
// Usage example:
//   Sdmmc<SdmmcType::sdmmc1> sd;
//   sd.enableBusClock();
//   sd.prepareGpio(GpioPinType::pc8);  // D0
//   sd.prepareGpio(GpioPinType::pc12); // CK
//   sd.prepareGpio(GpioPinType::pd2);  // CMD
//   sd.powerOn();
//   sd.initClock(clockConf);
// ============================================================

template<SdmmcType SDMMC_TYPE>
struct Sdmmc {
  constexpr static auto type = SDMMC_TYPE;
  constexpr static auto info = STM32_PERIPH_INFO.sdmmc[to_idx(type)];
  constexpr static auto addr = info.addr;

  // ----- Register addresses -----
  constexpr static uint32_t addr_POWER     = addr + offsetof(SDMMC_TypeDef, POWER);
  constexpr static uint32_t addr_CLKCR     = addr + offsetof(SDMMC_TypeDef, CLKCR);
  constexpr static uint32_t addr_ARG       = addr + offsetof(SDMMC_TypeDef, ARG);
  constexpr static uint32_t addr_CMD       = addr + offsetof(SDMMC_TypeDef, CMD);
  constexpr static uint32_t addr_RESPCMD   = addr + offsetof(SDMMC_TypeDef, RESPCMD);
  constexpr static uint32_t addr_RESP1     = addr + offsetof(SDMMC_TypeDef, RESP1);
  constexpr static uint32_t addr_RESP2     = addr + offsetof(SDMMC_TypeDef, RESP2);
  constexpr static uint32_t addr_RESP3     = addr + offsetof(SDMMC_TypeDef, RESP3);
  constexpr static uint32_t addr_RESP4     = addr + offsetof(SDMMC_TypeDef, RESP4);
  constexpr static uint32_t addr_DTIMER    = addr + offsetof(SDMMC_TypeDef, DTIMER);
  constexpr static uint32_t addr_DLEN      = addr + offsetof(SDMMC_TypeDef, DLEN);
  constexpr static uint32_t addr_DCTRL     = addr + offsetof(SDMMC_TypeDef, DCTRL);
  constexpr static uint32_t addr_DCOUNT    = addr + offsetof(SDMMC_TypeDef, DCOUNT);
  constexpr static uint32_t addr_STA       = addr + offsetof(SDMMC_TypeDef, STA);
  constexpr static uint32_t addr_ICR       = addr + offsetof(SDMMC_TypeDef, ICR);
  constexpr static uint32_t addr_MASK      = addr + offsetof(SDMMC_TypeDef, MASK);
  constexpr static uint32_t addr_ACKTIME   = addr + offsetof(SDMMC_TypeDef, ACKTIME);
  constexpr static uint32_t addr_IDMACTRL  = addr + offsetof(SDMMC_TypeDef, IDMACTRL);
  constexpr static uint32_t addr_IDMABSIZE = addr + offsetof(SDMMC_TypeDef, IDMABSIZE);
  constexpr static uint32_t addr_IDMABASE0 = addr + offsetof(SDMMC_TypeDef, IDMABASE0);
  constexpr static uint32_t addr_IDMABASE1 = addr + offsetof(SDMMC_TypeDef, IDMABASE1);
  constexpr static uint32_t addr_FIFO      = addr + offsetof(SDMMC_TypeDef, FIFO);

  // ===== POWER register fields =====
  RegEnum<addr_POWER, SDMMC_POWER_PWRCTRL_Msk,
          SdmmcPowerCtrl, SDMMC_POWER_PWRCTRL_Pos>     powerCtrl;
  RegFlag<addr_POWER, SDMMC_POWER_VSWITCH_Msk>         vswitch;
  RegFlag<addr_POWER, SDMMC_POWER_VSWITCHEN_Msk>       vswitchEn;
  RegFlag<addr_POWER, SDMMC_POWER_DIRPOL_Msk>          dirPol;

  // ===== CLKCR register fields =====
  RegValue<addr_CLKCR, SDMMC_CLKCR_CLKDIV_Msk,
           SDMMC_CLKCR_CLKDIV_Pos>                     clkDiv;
  RegFlag<addr_CLKCR, SDMMC_CLKCR_PWRSAV_Msk>          pwrSav;
  RegEnum<addr_CLKCR, SDMMC_CLKCR_WIDBUS_Msk,
          SdmmcBusWidth, SDMMC_CLKCR_WIDBUS_Pos>       busWidth;
  RegFlag<addr_CLKCR, SDMMC_CLKCR_NEGEDGE_Msk>         negEdge;
  RegFlag<addr_CLKCR, SDMMC_CLKCR_HWFC_EN_Msk>         hwFlowCtrl;
  RegFlag<addr_CLKCR, SDMMC_CLKCR_DDR_Msk>             ddr;
  RegFlag<addr_CLKCR, SDMMC_CLKCR_BUSSPEED_Msk>        busSpeed;
  RegEnum<addr_CLKCR, SDMMC_CLKCR_SELCLKRX_Msk,
          SdmmcRxClkSel, SDMMC_CLKCR_SELCLKRX_Pos>     rxClkSel;

  // ===== ARG register (32-bit command argument) =====
  Reg<addr_ARG>                                         cmdArg;

  // ===== CMD register fields =====
  RegValue<addr_CMD, SDMMC_CMD_CMDINDEX_Msk,
           SDMMC_CMD_CMDINDEX_Pos>                      cmdIndex;
  RegFlag<addr_CMD, SDMMC_CMD_CMDTRANS_Msk>             cmdTrans;
  RegFlag<addr_CMD, SDMMC_CMD_CMDSTOP_Msk>              cmdStop;
  RegEnum<addr_CMD, SDMMC_CMD_WAITRESP_Msk,
          SdmmcWaitResp, SDMMC_CMD_WAITRESP_Pos>        waitResp;
  RegFlag<addr_CMD, SDMMC_CMD_WAITINT_Msk>              waitInt;
  RegFlag<addr_CMD, SDMMC_CMD_WAITPEND_Msk>             waitPend;
  RegFlag<addr_CMD, SDMMC_CMD_CPSMEN_Msk>               cpsmEn;
  RegFlag<addr_CMD, SDMMC_CMD_DTHOLD_Msk>               dtHold;
  RegFlag<addr_CMD, SDMMC_CMD_BOOTMODE_Msk>             bootMode;
  RegFlag<addr_CMD, SDMMC_CMD_BOOTEN_Msk>               bootEn;
  RegFlag<addr_CMD, SDMMC_CMD_CMDSUSPEND_Msk>           cmdSuspend;

  // ===== RESPCMD register (read-only) =====
  RegValueRO<addr_RESPCMD, SDMMC_RESPCMD_RESPCMD_Msk,
             SDMMC_RESPCMD_RESPCMD_Pos>                 respCmd;

  // ===== RESP1..4 registers (read-only, 32-bit) =====
  RegRO<addr_RESP1>                                     resp1;
  RegRO<addr_RESP2>                                     resp2;
  RegRO<addr_RESP3>                                     resp3;
  RegRO<addr_RESP4>                                     resp4;

  // ===== DTIMER register (32-bit data timeout) =====
  Reg<addr_DTIMER>                                      dataTimer;

  // ===== DLEN register =====
  RegValue<addr_DLEN, SDMMC_DLEN_DATALENGTH_Msk,
           SDMMC_DLEN_DATALENGTH_Pos>                   dataLength;

  // ===== DCTRL register fields =====
  RegFlag<addr_DCTRL, SDMMC_DCTRL_DTEN_Msk>             dataEn;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_DTDIR_Msk>            dataDir;
  RegEnum<addr_DCTRL, SDMMC_DCTRL_DTMODE_Msk,
          SdmmcDataMode, SDMMC_DCTRL_DTMODE_Pos>        dataMode;
  RegEnum<addr_DCTRL, SDMMC_DCTRL_DBLOCKSIZE_Msk,
          SdmmcBlockSize, SDMMC_DCTRL_DBLOCKSIZE_Pos>   blockSize;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_RWSTART_Msk>          rwStart;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_RWSTOP_Msk>           rwStop;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_RWMOD_Msk>            rwMod;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_SDIOEN_Msk>           sdioEn;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_BOOTACKEN_Msk>        bootAckEn;
  RegFlag<addr_DCTRL, SDMMC_DCTRL_FIFORST_Msk>          fifoRst;

  // ===== DCOUNT register (read-only) =====
  RegValueRO<addr_DCOUNT, SDMMC_DCOUNT_DATACOUNT_Msk,
             SDMMC_DCOUNT_DATACOUNT_Pos>                dataCount;

  // ===== STA register fields (read-only status) =====
  RegFlagRO<addr_STA, SDMMC_STA_CCRCFAIL_Msk>           isCCrcFail;
  RegFlagRO<addr_STA, SDMMC_STA_DCRCFAIL_Msk>           isDCrcFail;
  RegFlagRO<addr_STA, SDMMC_STA_CTIMEOUT_Msk>           isCTimeout;
  RegFlagRO<addr_STA, SDMMC_STA_DTIMEOUT_Msk>           isDTimeout;
  RegFlagRO<addr_STA, SDMMC_STA_TXUNDERR_Msk>           isTxUnderr;
  RegFlagRO<addr_STA, SDMMC_STA_RXOVERR_Msk>            isRxOverr;
  RegFlagRO<addr_STA, SDMMC_STA_CMDREND_Msk>            isCmdRend;
  RegFlagRO<addr_STA, SDMMC_STA_CMDSENT_Msk>            isCmdSent;
  RegFlagRO<addr_STA, SDMMC_STA_DATAEND_Msk>            isDataEnd;
  RegFlagRO<addr_STA, SDMMC_STA_DHOLD_Msk>              isDHold;
  RegFlagRO<addr_STA, SDMMC_STA_DBCKEND_Msk>            isDbckEnd;
  RegFlagRO<addr_STA, SDMMC_STA_DABORT_Msk>             isDAbort;
  RegFlagRO<addr_STA, SDMMC_STA_DPSMACT_Msk>            isDpsmAct;
  RegFlagRO<addr_STA, SDMMC_STA_CPSMACT_Msk>            isCpsmAct;
  RegFlagRO<addr_STA, SDMMC_STA_TXFIFOHE_Msk>           isTxFifoHe;
  RegFlagRO<addr_STA, SDMMC_STA_RXFIFOHF_Msk>           isRxFifoHf;
  RegFlagRO<addr_STA, SDMMC_STA_TXFIFOF_Msk>            isTxFifoF;
  RegFlagRO<addr_STA, SDMMC_STA_RXFIFOF_Msk>            isRxFifoF;
  RegFlagRO<addr_STA, SDMMC_STA_TXFIFOE_Msk>            isTxFifoE;
  RegFlagRO<addr_STA, SDMMC_STA_RXFIFOE_Msk>            isRxFifoE;
  RegFlagRO<addr_STA, SDMMC_STA_BUSYD0_Msk>             isBusyD0;
  RegFlagRO<addr_STA, SDMMC_STA_BUSYD0END_Msk>          isBusyD0End;
  RegFlagRO<addr_STA, SDMMC_STA_SDIOIT_Msk>             isSdioIt;
  RegFlagRO<addr_STA, SDMMC_STA_ACKFAIL_Msk>            isAckFail;
  RegFlagRO<addr_STA, SDMMC_STA_ACKTIMEOUT_Msk>         isAckTimeout;
  RegFlagRO<addr_STA, SDMMC_STA_VSWEND_Msk>             isVswEnd;
  RegFlagRO<addr_STA, SDMMC_STA_CKSTOP_Msk>             isCkStop;
  RegFlagRO<addr_STA, SDMMC_STA_IDMATE_Msk>             isIdmaTE;
  RegFlagRO<addr_STA, SDMMC_STA_IDMABTC_Msk>            isIdmaBtc;

  // ===== ICR register fields (write-1-to-clear) =====
  RegFlagWO<addr_ICR, SDMMC_ICR_CCRCFAILC_Msk>          clearCCrcFail;
  RegFlagWO<addr_ICR, SDMMC_ICR_DCRCFAILC_Msk>          clearDCrcFail;
  RegFlagWO<addr_ICR, SDMMC_ICR_CTIMEOUTC_Msk>          clearCTimeout;
  RegFlagWO<addr_ICR, SDMMC_ICR_DTIMEOUTC_Msk>          clearDTimeout;
  RegFlagWO<addr_ICR, SDMMC_ICR_TXUNDERRC_Msk>          clearTxUnderr;
  RegFlagWO<addr_ICR, SDMMC_ICR_RXOVERRC_Msk>           clearRxOverr;
  RegFlagWO<addr_ICR, SDMMC_ICR_CMDRENDC_Msk>           clearCmdRend;
  RegFlagWO<addr_ICR, SDMMC_ICR_CMDSENTC_Msk>           clearCmdSent;
  RegFlagWO<addr_ICR, SDMMC_ICR_DATAENDC_Msk>           clearDataEnd;
  RegFlagWO<addr_ICR, SDMMC_ICR_DHOLDC_Msk>             clearDHold;
  RegFlagWO<addr_ICR, SDMMC_ICR_DBCKENDC_Msk>           clearDbckEnd;
  RegFlagWO<addr_ICR, SDMMC_ICR_DABORTC_Msk>            clearDAbort;
  RegFlagWO<addr_ICR, SDMMC_ICR_BUSYD0ENDC_Msk>         clearBusyD0End;
  RegFlagWO<addr_ICR, SDMMC_ICR_SDIOITC_Msk>            clearSdioIt;
  RegFlagWO<addr_ICR, SDMMC_ICR_ACKFAILC_Msk>           clearAckFail;
  RegFlagWO<addr_ICR, SDMMC_ICR_ACKTIMEOUTC_Msk>        clearAckTimeout;
  RegFlagWO<addr_ICR, SDMMC_ICR_VSWENDC_Msk>            clearVswEnd;
  RegFlagWO<addr_ICR, SDMMC_ICR_CKSTOPC_Msk>            clearCkStop;
  RegFlagWO<addr_ICR, SDMMC_ICR_IDMATEC_Msk>            clearIdmaTE;
  RegFlagWO<addr_ICR, SDMMC_ICR_IDMABTCC_Msk>           clearIdmaBtc;

  // ===== MASK register fields (interrupt enable) =====
  RegFlag<addr_MASK, SDMMC_MASK_CCRCFAILIE_Msk>          enableItCCrcFail;
  RegFlag<addr_MASK, SDMMC_MASK_DCRCFAILIE_Msk>          enableItDCrcFail;
  RegFlag<addr_MASK, SDMMC_MASK_CTIMEOUTIE_Msk>          enableItCTimeout;
  RegFlag<addr_MASK, SDMMC_MASK_DTIMEOUTIE_Msk>          enableItDTimeout;
  RegFlag<addr_MASK, SDMMC_MASK_TXUNDERRIE_Msk>          enableItTxUnderr;
  RegFlag<addr_MASK, SDMMC_MASK_RXOVERRIE_Msk>           enableItRxOverr;
  RegFlag<addr_MASK, SDMMC_MASK_CMDRENDIE_Msk>           enableItCmdRend;
  RegFlag<addr_MASK, SDMMC_MASK_CMDSENTIE_Msk>           enableItCmdSent;
  RegFlag<addr_MASK, SDMMC_MASK_DATAENDIE_Msk>           enableItDataEnd;
  RegFlag<addr_MASK, SDMMC_MASK_DHOLDIE_Msk>             enableItDHold;
  RegFlag<addr_MASK, SDMMC_MASK_DBCKENDIE_Msk>           enableItDbckEnd;
  RegFlag<addr_MASK, SDMMC_MASK_DABORTIE_Msk>            enableItDAbort;
  RegFlag<addr_MASK, SDMMC_MASK_TXFIFOHEIE_Msk>          enableItTxFifoHe;
  RegFlag<addr_MASK, SDMMC_MASK_RXFIFOHFIE_Msk>          enableItRxFifoHf;
  RegFlag<addr_MASK, SDMMC_MASK_RXFIFOFIE_Msk>           enableItRxFifoF;
  RegFlag<addr_MASK, SDMMC_MASK_TXFIFOEIE_Msk>           enableItTxFifoE;
  RegFlag<addr_MASK, SDMMC_MASK_BUSYD0ENDIE_Msk>         enableItBusyD0End;
  RegFlag<addr_MASK, SDMMC_MASK_SDIOITIE_Msk>            enableItSdioIt;
  RegFlag<addr_MASK, SDMMC_MASK_ACKFAILIE_Msk>           enableItAckFail;
  RegFlag<addr_MASK, SDMMC_MASK_ACKTIMEOUTIE_Msk>        enableItAckTimeout;
  RegFlag<addr_MASK, SDMMC_MASK_VSWENDIE_Msk>            enableItVswEnd;
  RegFlag<addr_MASK, SDMMC_MASK_CKSTOPIE_Msk>            enableItCkStop;
  RegFlag<addr_MASK, SDMMC_MASK_IDMABTCIE_Msk>           enableItIdmaBtc;

  // ===== ACKTIME register =====
  RegValue<addr_ACKTIME, SDMMC_ACKTIME_ACKTIME_Msk,
           SDMMC_ACKTIME_ACKTIME_Pos>                    ackTimeout;

  // ===== IDMACTRL register fields =====
  RegFlag<addr_IDMACTRL, SDMMC_IDMA_IDMAEN_Msk>         idmaEn;
  RegFlag<addr_IDMACTRL, SDMMC_IDMA_IDMABMODE_Msk>      idmaDoubleBuffer;
  RegFlag<addr_IDMACTRL, SDMMC_IDMA_IDMABACT_Msk>       idmaBufferSel;

  // ===== IDMABSIZE register =====
  RegValue<addr_IDMABSIZE, SDMMC_IDMABSIZE_IDMABNDT_Msk,
           SDMMC_IDMABSIZE_IDMABNDT_Pos>                idmaBufferSize;

  // ===== IDMABASE0/1 registers (32-bit buffer addresses) =====
  Reg<addr_IDMABASE0>                                    idmaBase0;
  Reg<addr_IDMABASE1>                                    idmaBase1;

  // ===== FIFO register (32-bit data access) =====
  Reg<addr_FIFO>                                         fifo;

  // ===== High-level init helpers =====

  IGB_FAST_INLINE void initClock(const SdmmcClockConf& conf) {
    (clkDiv.val(static_cast<uint32_t>(conf.clkDiv))
      | pwrSav.val(conf.pwrSav)
      | busWidth.val(conf.busWidth)
      | negEdge.val(conf.negEdge)
      | hwFlowCtrl.val(conf.hwFlowCtrl)
      | ddr.val(conf.ddr)
      | busSpeed.val(conf.busSpeed)
      | rxClkSel.val(conf.rxClkSel)
    ).update();
  }

  IGB_FAST_INLINE void sendCommand(const SdmmcCmdConf& conf) {
    (cmdIndex.val(static_cast<uint32_t>(conf.cmdIndex))
      | cmdTrans.val(conf.cmdTrans)
      | cmdStop.val(conf.cmdStop)
      | waitResp.val(conf.waitResp)
      | waitInt.val(conf.waitInt)
      | waitPend.val(conf.waitPend)
      | cpsmEn.val(conf.cpsmEn)
      | dtHold.val(conf.dtHold)
      | bootMode.val(conf.bootMode)
      | bootEn.val(conf.bootEn)
      | cmdSuspend.val(conf.cmdSuspend)
    ).update();
  }

  IGB_FAST_INLINE void initData(const SdmmcDataConf& conf) {
    dataTimer(conf.dataTimeout);
    dataLength(conf.dataLength);
    (dataEn.val(conf.dataEn)
      | dataDir.val(conf.dataDir)
      | dataMode.val(conf.dataMode)
      | blockSize.val(conf.blockSize)
      | rwStart.val(conf.rwStart)
      | rwStop.val(conf.rwStop)
      | rwMod.val(conf.rwMod)
      | sdioEn.val(conf.sdioEn)
      | bootAckEn.val(conf.bootAckEn)
      | fifoRst.val(conf.fifoRst)
    ).update();
  }

  IGB_FAST_INLINE void powerOn()  { powerCtrl(SdmmcPowerCtrl::on); }
  IGB_FAST_INLINE void powerOff() { powerCtrl(SdmmcPowerCtrl::off); }

  IGB_FAST_INLINE void     setArg(uint32_t arg) { cmdArg(arg); }
  IGB_FAST_INLINE uint32_t getResp1()            { return resp1(); }
  IGB_FAST_INLINE uint32_t getResp2()            { return resp2(); }
  IGB_FAST_INLINE uint32_t getResp3()            { return resp3(); }
  IGB_FAST_INLINE uint32_t getResp4()            { return resp4(); }

  IGB_FAST_INLINE void     writeFifo(uint32_t v) { fifo(v); }
  IGB_FAST_INLINE uint32_t readFifo()            { return fifo(); }

  IGB_FAST_INLINE void clearAllFlags() {
    Reg<addr_ICR> icr;
    icr(SDMMC_ICR_CCRCFAILC_Msk | SDMMC_ICR_DCRCFAILC_Msk | SDMMC_ICR_CTIMEOUTC_Msk
      | SDMMC_ICR_DTIMEOUTC_Msk | SDMMC_ICR_TXUNDERRC_Msk | SDMMC_ICR_RXOVERRC_Msk
      | SDMMC_ICR_CMDRENDC_Msk  | SDMMC_ICR_CMDSENTC_Msk  | SDMMC_ICR_DATAENDC_Msk
      | SDMMC_ICR_DHOLDC_Msk    | SDMMC_ICR_DBCKENDC_Msk  | SDMMC_ICR_DABORTC_Msk
      | SDMMC_ICR_BUSYD0ENDC_Msk | SDMMC_ICR_SDIOITC_Msk  | SDMMC_ICR_ACKFAILC_Msk
      | SDMMC_ICR_ACKTIMEOUTC_Msk | SDMMC_ICR_VSWENDC_Msk | SDMMC_ICR_CKSTOPC_Msk
      | SDMMC_ICR_IDMATEC_Msk   | SDMMC_ICR_IDMABTCC_Msk);
  }

  IGB_FAST_INLINE void disableAllInterrupts() {
    Reg<addr_MASK> mask;
    mask(0);
  }

  IGB_FAST_INLINE void setupIdma(uint32_t bufAddr0, uint32_t bufSize = 0, uint32_t bufAddr1 = 0) {
    idmaBase0(bufAddr0);
    if (bufAddr1 != 0) {
      idmaBase1(bufAddr1);
      idmaBufferSize(static_cast<uint32_t>(bufSize));
      idmaDoubleBuffer(true);
    }
    idmaEn(true);
  }

  IGB_FAST_INLINE void disableIdma() {
    idmaEn(false);
  }

  // ===== Bus clock management =====
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

  // ===== GPIO AF configuration =====
  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(
#if defined(STM32H7)
      GpioSpeedMode::veryHigh
#else
      GpioSpeedMode::high
#endif
    );
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(result.value());
  }
};

} // namespace stm32
} // namespace igb

#endif // STM32_PERIPHGRP_SDMMC_EXISTS

#endif // STM32H7
