#pragma once

// Thin OOP wrapper around a FatFs FIL handle. Owns one file at a time —
// the typical pattern for SD card consumers that open / read / close
// sequentially. For directory iteration use opendir/readdir/closedir or
// the eachFile() helper.
//
// The class does NOT own the disk-IO driver or the FATFS mount; both must
// be set up by the caller before init(). On STM32H7 SDMMC this is done by
// `igb::sdk::SdCard` plus `sd_diskio.c`.

#include <cstdint>
#include <cstddef>
#include "ff.h"

namespace igb::sdk {

// C bindings exported by sd_diskio.c (FatFs is C, sd_diskio is the bridge).
extern "C" {
  uint8_t sd_diskio_register(void);
  FATFS*  sd_diskio_get_fs(void);
  const char* sd_diskio_get_path(void);
}

struct FatfsFile {
  FIL sd_file;

  bool init() {
    if (sd_diskio_register() != 0) return false;
    if (f_mount(sd_diskio_get_fs(), sd_diskio_get_path(), 1) != FR_OK) return false;
    return true;
  }

  bool open(const char* file_name, bool is_write = false) {
    auto flag = is_write ? ((FA_CREATE_ALWAYS) | (FA_WRITE) | (FA_READ)) : FA_READ;
    return f_open(&sd_file, file_name, flag) == FR_OK;
  }

  bool close() {
    return f_close(&sd_file) == FR_OK;
  }

  bool write(uint8_t* buf, size_t buf_size, size_t* written_size) {
    return f_write(&sd_file, buf, buf_size, written_size) == FR_OK;
  }

  bool read(uint8_t* buf, size_t buf_size, size_t* read_size) {
    return f_read(&sd_file, buf, buf_size, read_size) == FR_OK;
  }

  bool lseek(size_t pos) {
    return f_lseek(&sd_file, pos) == FR_OK;
  }

  bool opendir(DIR* dir, const char* dir_name) {
    return f_opendir(dir, dir_name) == FR_OK;
  }

  bool readdir(DIR* dir, FILINFO* file_info) {
    return f_readdir(dir, file_info) == FR_OK;
  }

  bool closedir(DIR* dir) {
    return f_closedir(dir) == FR_OK;
  }

  bool eof() {
    return f_eof(&sd_file);
  }

  bool sync() {
    return f_sync(&sd_file) == FR_OK;
  }

  bool existsDir(const char* dir_name) {
    DIR dir;
    if (!opendir(&dir, dir_name)) return false;
    closedir(&dir);
    return true;
  }

  void eachFile(const char* dir_name, auto&& func) {
    DIR dir;
    if (!opendir(&dir, dir_name)) return;
    FILINFO fno;
    while (readdir(&dir, &fno) && fno.fname[0] != 0) {
      if (fno.fattrib & (AM_HID | AM_DIR)) continue;
      func(fno);
    }
    closedir(&dir);
  }

  bool remove(const char* file_name) {
    return f_unlink(file_name) == FR_OK;
  }
};

}  // namespace igb::sdk
