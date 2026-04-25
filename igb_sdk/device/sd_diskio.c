// FatFS diskio driver bridge for igb SDMMC SD card driver
//
// Registers as FatFS volume "0:/" via FATFS_LinkDriver().
// Calls into sd_card (C++ SdCard struct) through thin C wrappers.

#include "ff_gen_drv.h"
#include <string.h>

// C wrappers defined in sd_card_diskio.cpp
extern uint8_t sd_card_c_init(void);
extern uint8_t sd_card_c_status(void);
extern uint8_t sd_card_c_read(uint8_t* buf, uint32_t sector, uint32_t count);
extern uint8_t sd_card_c_write(const uint8_t* buf, uint32_t sector, uint32_t count);
extern uint32_t sd_card_c_block_count(void);
extern uint32_t sd_card_c_block_size(void);

// ---- FatFS diskio interface ----

static DSTATUS sd_initialize(BYTE lun) {
  (void)lun;
  return sd_card_c_init() ? 0 : STA_NOINIT;
}

static DSTATUS sd_status(BYTE lun) {
  (void)lun;
  return sd_card_c_status() ? 0 : STA_NOINIT;
}

static DRESULT sd_read(BYTE lun, BYTE* buff, DWORD sector, UINT count) {
  (void)lun;
  if (sd_card_c_read(buff, sector, count)) return RES_OK;
  return RES_ERROR;
}

static DRESULT sd_write(BYTE lun, const BYTE* buff, DWORD sector, UINT count) {
  (void)lun;
  if (sd_card_c_write(buff, sector, count)) return RES_OK;
  return RES_ERROR;
}

static DRESULT sd_ioctl(BYTE lun, BYTE cmd, void* buff) {
  (void)lun;
  switch (cmd) {
    case CTRL_SYNC:
      return RES_OK;
    case GET_SECTOR_COUNT:
      *(DWORD*)buff = sd_card_c_block_count();
      return RES_OK;
    case GET_SECTOR_SIZE:
      *(WORD*)buff = (WORD)sd_card_c_block_size();
      return RES_OK;
    case GET_BLOCK_SIZE:
      *(DWORD*)buff = 1; // erase block = 1 sector
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

// Driver structure
const Diskio_drvTypeDef SD_Driver = {
  sd_initialize,
  sd_status,
  sd_read,
  sd_write,
  sd_ioctl,
};

// Registration (called from app_sdmmc.init)
static char sd_path[4];
static FATFS sd_fs;

uint8_t sd_diskio_register(void) {
  return FATFS_LinkDriver(&SD_Driver, sd_path);
}

FATFS* sd_diskio_get_fs(void) {
  return &sd_fs;
}

const char* sd_diskio_get_path(void) {
  return sd_path;
}
