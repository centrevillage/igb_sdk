#pragma once

#if defined(STM32H7)

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#define IGB_MPU_REG_ADDR(member) (MPU_BASE + offsetof(MPU_Type, member))

// Region size encoding: actual size = 2^(SIZE+1) bytes (minimum: SIZE=0x04 → 32 bytes)
// Note: Cortex-M7 MPU requires minimum 256-byte region alignment in practice
enum class MpuRegionSize : uint32_t {
  size32B   = 0x04U,
  size64B   = 0x05U,
  size128B  = 0x06U,
  size256B  = 0x07U,
  size512B  = 0x08U,
  size1KB   = 0x09U,
  size2KB   = 0x0AU,
  size4KB   = 0x0BU,
  size8KB   = 0x0CU,
  size16KB  = 0x0DU,
  size32KB  = 0x0EU,
  size64KB  = 0x0FU,
  size128KB = 0x10U,
  size256KB = 0x11U,
  size512KB = 0x12U,
  size1MB   = 0x13U,
  size2MB   = 0x14U,
  size4MB   = 0x15U,
  size8MB   = 0x16U,
  size16MB  = 0x17U,
  size32MB  = 0x18U,
  size64MB  = 0x19U,
  size128MB = 0x1AU,
  size256MB = 0x1BU,
  size512MB = 0x1CU,
  size1GB   = 0x1DU,
  size2GB   = 0x1EU,
  size4GB   = 0x1FU,
};

// Access Permission field (AP[2:0] in RASR[26:24])
// Ref: PM0253 Table 91
enum class MpuAccessPermission : uint32_t {
  noAccess         = 0x00U,  // Privileged: no access,  Unprivileged: no access
  privRw           = 0x01U,  // Privileged: R/W,         Unprivileged: no access
  privRwUnprivRo   = 0x02U,  // Privileged: R/W,         Unprivileged: read-only
  fullAccess       = 0x03U,  // Privileged: R/W,         Unprivileged: R/W
  privRo           = 0x05U,  // Privileged: read-only,   Unprivileged: no access
  roForAll         = 0x06U,  // Privileged: read-only,   Unprivileged: read-only
};

// Memory attribute combining TEX[2:0], C, B as a 5-bit value.
// Encoding: bits[4:2] = TEX[2:0], bit[1] = C, bit[0] = B
// Ref: PM0253 Table 92
//
// When TEX[2]=1 (cached memory), TEX[1:0] selects the outer cache policy
// and C|B selects the inner cache policy, both from the same 4 policies:
//   00 = Non-cacheable (NC)
//   01 = Write-back, write and read allocate (WBWA)
//   10 = Write-through, no write allocate (WT)
//   11 = Write-back, no write allocate (WBNoWA)
enum class MpuMemAttr : uint8_t {
  // TEX=000: Strongly-ordered / Device / Normal (no cache)
  stronglyOrdered      = 0b00000,  // TEX=000, C=0, B=0: Strongly-ordered, always shareable
  deviceShareable      = 0b00001,  // TEX=000, C=0, B=1: Device, shareable
  normalWT             = 0b00010,  // TEX=000, C=1, B=0: Normal, write-through no write alloc
  normalWBNoWA         = 0b00011,  // TEX=000, C=1, B=1: Normal, write-back no write alloc
  // TEX=001: Normal (non-cached or write-back+alloc)
  normalNC             = 0b00100,  // TEX=001, C=0, B=0: Normal, non-cacheable
  normalWBWA           = 0b00111,  // TEX=001, C=1, B=1: Normal, write-back write+read alloc
  // TEX=010: Device non-shareable
  deviceNonShareable   = 0b01000,  // TEX=010, C=0, B=0: Device, non-shareable
  // TEX=1xx: Cached memory; TEX[1:0]=outer policy, C|B=inner policy
  outerNCInnerNC         = 0b10000,
  outerNCInnerWBWA       = 0b10001,
  outerNCInnerWT         = 0b10010,
  outerNCInnerWBNoWA     = 0b10011,
  outerWBWAInnerNC       = 0b10100,
  outerWBWAInnerWBWA     = 0b10101,
  outerWBWAInnerWT       = 0b10110,
  outerWBWAInnerWBNoWA   = 0b10111,
  outerWTInnerNC         = 0b11000,
  outerWTInnerWBWA       = 0b11001,
  outerWTInnerWT         = 0b11010,
  outerWTInnerWBNoWA     = 0b11011,
  outerWBNoWAInnerNC     = 0b11100,
  outerWBNoWAInnerWBWA   = 0b11101,
  outerWBNoWAInnerWT     = 0b11110,
  outerWBNoWAInnerWBNoWA = 0b11111,
};


struct Mpu {
  constexpr static auto addr = MPU_BASE;

  // Register addresses
  constexpr static auto addr_TYPE    = IGB_MPU_REG_ADDR(TYPE);
  constexpr static auto addr_CTRL    = IGB_MPU_REG_ADDR(CTRL);
  constexpr static auto addr_RNR     = IGB_MPU_REG_ADDR(RNR);
  constexpr static auto addr_RBAR    = IGB_MPU_REG_ADDR(RBAR);
  constexpr static auto addr_RASR    = IGB_MPU_REG_ADDR(RASR);
  constexpr static auto addr_RBAR_A1 = IGB_MPU_REG_ADDR(RBAR_A1);
  constexpr static auto addr_RASR_A1 = IGB_MPU_REG_ADDR(RASR_A1);
  constexpr static auto addr_RBAR_A2 = IGB_MPU_REG_ADDR(RBAR_A2);
  constexpr static auto addr_RASR_A2 = IGB_MPU_REG_ADDR(RASR_A2);
  constexpr static auto addr_RBAR_A3 = IGB_MPU_REG_ADDR(RBAR_A3);
  constexpr static auto addr_RASR_A3 = IGB_MPU_REG_ADDR(RASR_A3);

  // TYPE register (read-only)
  // Number of supported MPU data regions (typically 8 on Cortex-M7)
  static RegValueRO<addr_TYPE, MPU_TYPE_DREGION_Msk, MPU_TYPE_DREGION_Pos> numDataRegions;
  // Number of MPU instruction regions (0 = unified MPU)
  static RegValueRO<addr_TYPE, MPU_TYPE_IREGION_Msk, MPU_TYPE_IREGION_Pos> numInstrRegions;
  // true = separate instruction/data MPU, false = unified MPU
  static RegFlagRO<addr_TYPE, MPU_TYPE_SEPARATE_Msk> isSeparate;

  // CTRL register
  // Enable/disable the MPU
  static RegFlag<addr_CTRL, MPU_CTRL_ENABLE_Msk>     ctrlEnable;
  // Enable MPU during HardFault, NMI, and FAULTMASK handlers
  static RegFlag<addr_CTRL, MPU_CTRL_HFNMIENA_Msk>   ctrlHfnmiEnable;
  // Use default memory map as background for privileged software accesses
  static RegFlag<addr_CTRL, MPU_CTRL_PRIVDEFENA_Msk>  ctrlPrivDefEnable;

  // RNR register — select which region RBAR/RASR refer to (0–7)
  static RegValue<addr_RNR, MPU_RNR_REGION_Msk, MPU_RNR_REGION_Pos> regionNumber;

  // RBAR register
  // Base address bits [31:5]; must be aligned to the region size
  // getter: returns raw address value (bits [31:5], lower 5 bits masked to 0)
  IGB_FAST_INLINE static uint32_t rbarAddr() {
    return (*((volatile uint32_t*)addr_RBAR)) & MPU_RBAR_ADDR_Msk;
  }
  // setter: writes only bits [31:5], preserving VALID[4] and REGION[3:0]
  IGB_FAST_INLINE static void rbarAddr(uint32_t v) {
    volatile uint32_t& reg = *((volatile uint32_t*)addr_RBAR);
    reg = (reg & ~MPU_RBAR_ADDR_Msk) | (v & MPU_RBAR_ADDR_Msk);
  }
  // When set, RBAR[3:0] is written to RNR simultaneously (allows atomic region select + address write)
  static RegFlag<addr_RBAR, MPU_RBAR_VALID_Msk>                                       rbarValid;
  // Region number written to RNR when VALID=1
  static RegValue<addr_RBAR, MPU_RBAR_REGION_Msk, MPU_RBAR_REGION_Pos>               rbarRegion;

  // RASR register
  // Enable this region
  static RegFlag<addr_RASR, MPU_RASR_ENABLE_Msk>                                      rasrEnable;
  // Region size (see MpuRegionSize enum)
  static RegEnum<addr_RASR, MPU_RASR_SIZE_Msk, MpuRegionSize, MPU_RASR_SIZE_Pos>      rasrSize;
  // Sub-region disable: each bit disables one 1/8th of the region (bit 0 = sub-region 0)
  static RegValue<addr_RASR, MPU_RASR_SRD_Msk, MPU_RASR_SRD_Pos>                     rasrSubRegionDisable;
  // Shareable (S bit): marks memory as shared between multiple bus masters
  static RegFlag<addr_RASR, MPU_RASR_S_Msk>                                           rasrShareable;
  // TEX[2:0] + C + B combined as MpuMemAttr (bits[4:2]=TEX, bit[1]=C, bit[0]=B)
  IGB_FAST_INLINE static MpuMemAttr rasrMemAttr() {
    uint32_t reg = *((volatile uint32_t*)addr_RASR);
    uint32_t tex = (reg & MPU_RASR_TEX_Msk) >> MPU_RASR_TEX_Pos;
    uint32_t c   = (reg & MPU_RASR_C_Msk)   >> MPU_RASR_C_Pos;
    uint32_t b   = (reg & MPU_RASR_B_Msk)   >> MPU_RASR_B_Pos;
    return static_cast<MpuMemAttr>((tex << 2) | (c << 1) | b);
  }
  IGB_FAST_INLINE static RegFragment<addr_RASR> rasrMemAttrVal(MpuMemAttr attr) {
    uint32_t v   = static_cast<uint32_t>(attr);
    uint32_t tex = (v >> 2) & 0x7U;
    uint32_t c   = (v >> 1) & 0x1U;
    uint32_t b   =  v       & 0x1U;
    return RegFragment<addr_RASR> {
      MPU_RASR_TEX_Msk | MPU_RASR_C_Msk | MPU_RASR_B_Msk,
      ((tex << MPU_RASR_TEX_Pos) & MPU_RASR_TEX_Msk)
      | ((c << MPU_RASR_C_Pos)   & MPU_RASR_C_Msk)
      | ((b << MPU_RASR_B_Pos)   & MPU_RASR_B_Msk)
    };
  }
  IGB_FAST_INLINE static void rasrMemAttr(MpuMemAttr attr) {
    rasrMemAttrVal(attr).update();
  }
  // Access permission
  static RegEnum<addr_RASR, MPU_RASR_AP_Msk, MpuAccessPermission, MPU_RASR_AP_Pos>   rasrAccessPermission;
  // Execute Never: when set, the region cannot be used for instruction fetch
  static RegFlag<addr_RASR, MPU_RASR_XN_Msk>                                         rasrExecuteNever;

  // Alias RBAR/RASR pairs for simultaneous multi-region programming (up to 4 regions at once)
  // Use with RNR = N, then alias_N covers region N+1, N+2, N+3
  static Reg<addr_RBAR_A1> rbarAlias1;
  static Reg<addr_RASR_A1> rasrAlias1;
  static Reg<addr_RBAR_A2> rbarAlias2;
  static Reg<addr_RASR_A2> rasrAlias2;
  static Reg<addr_RBAR_A3> rbarAlias3;
  static Reg<addr_RASR_A3> rasrAlias3;

  // High-level region configuration
  struct RegionConf {
    uint8_t              number;                                           // Region number (0–7)
    uint32_t             baseAddress;                                      // Must be aligned to region size
    MpuRegionSize        size;
    MpuAccessPermission  accessPermission = MpuAccessPermission::fullAccess;
    MpuMemAttr           memAttr          = MpuMemAttr::stronglyOrdered;
    uint8_t              subRegionDisable = 0x00U;                        // Bit N=1 disables sub-region N
    bool                 shareable        = false;
    bool                 executeNever     = false;
    bool                 enable           = true;
  };

  // Disable MPU with required memory barriers
  IGB_FAST_INLINE static void disable() {
    __DMB();
    ctrlEnable.disable();
    __DSB();
    __ISB();
  }

  // Enable MPU with required memory barriers.
  // privDefEnable: use default memory map for privileged accesses when no region matches
  // hfnmiEnable:   keep MPU active during HardFault/NMI handlers
  IGB_FAST_INLINE static void enable(bool privDefEnable = true, bool hfnmiEnable = false) {
    __DMB();
    ( ctrlEnable.val(true)
    | ctrlPrivDefEnable.val(privDefEnable)
    | ctrlHfnmiEnable.val(hfnmiEnable)
    ).update();
    __DSB();
    __ISB();
  }

  // Configure one or more MPU regions. Call between disable() and enable().
  static void configRegion(auto&&... confs) {
    static_assert(sizeof...(confs) > 0, "configRegion requires at least one RegionConf argument");
    ([&](const RegionConf& conf) {
      regionNumber(conf.number);
      rbarAddr(conf.baseAddress);
      ( rasrEnable.val(conf.enable)
      | rasrSize.val(conf.size)
      | rasrSubRegionDisable.val(conf.subRegionDisable)
      | rasrMemAttrVal(conf.memAttr)
      | rasrShareable.val(conf.shareable)
      | rasrAccessPermission.val(conf.accessPermission)
      | rasrExecuteNever.val(conf.executeNever)
      ).update();
    }(confs), ...);
  }

  // Disable and clear a single region
  IGB_FAST_INLINE static void clearRegion(uint8_t number) {
    regionNumber(number);
    *((volatile uint32_t*)addr_RBAR) = 0;
    *((volatile uint32_t*)addr_RASR) = 0;
  }
};

#undef IGB_MPU_REG_ADDR

}
}

#endif // defined(STM32H7)
