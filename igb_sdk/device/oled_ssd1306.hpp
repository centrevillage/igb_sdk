#pragma once

#include <igb_stm32/periph/systick.hpp>
#include <igb_sdk/base.hpp>
#include <igb_sdk/font/bitmap/cvfont.hpp>
#include <igb_util/null_functor.hpp>
#include <cstring>

namespace igb {
namespace sdk {

// DMA_STREAM_TYPE: DmaStream<...> for async DMA update, NullFunctor (default) for sync.
//
// Async usage:
//   1. Declare DmaStream and OledSsd1306 with DMA_STREAM_TYPE
//   2. Register ISR: extern "C" void DMAx_StreamN_IRQHandler() { oled.dma_stream.handleIrq(); }
//   3. Call oled.init() — this configures DMA stream and registers the completion callback
//   4. Draw into screen_buffer, then call oled.process() to kick off the async update
//   5. Before modifying screen_buffer again, check !oled.isBusy()
template<typename SPI_TYPE, typename GPIO_PIN_TYPE, size_t screen_width = 128, size_t screen_height = 64, typename WAIT_FUNC = NullFunctor, typename DMA_STREAM_TYPE = NullFunctor>
struct OledSsd1306 {
  SPI_TYPE spi;
  GPIO_PIN_TYPE cs_pin;
  GPIO_PIN_TYPE dc_pin;
  GPIO_PIN_TYPE reset_pin;

  // 32-byte alignment required for SCB_CleanDCache_by_Addr
  alignas(32) uint8_t screen_buffer[screen_width * screen_height / 8];
  uint8_t prev_screen_buffer[screen_width * screen_height / 8] = {};

  WAIT_FUNC _wait_func;

  // DMA stream — only used when DMA_STREAM_TYPE != NullFunctor
  DMA_STREAM_TYPE dma_stream;

  volatile bool _dma_busy = false;
  volatile bool _page_done = false;
  uint8_t _current_page = 0;

  void init() {
    prepareGpio();
    sendInitCommands();
    if constexpr (!std::is_same_v<DMA_STREAM_TYPE, NullFunctor>) {
      // ISR only sets flag — all SPI operations happen in process() (main loop)
      dma_stream.on_complete = [this]() {
        _page_done = true;
      };
    }
  }

  bool isBusy() const {
    if constexpr (!std::is_same_v<DMA_STREAM_TYPE, NullFunctor>) {
      return _dma_busy;
    }
    return false;
  }

  void process() {
    if constexpr (!std::is_same_v<DMA_STREAM_TYPE, NullFunctor>) {
      if (_dma_busy) {
        if (_page_done) {
          _page_done = false;
          spi.sendBufU8dmaEnd();
          cs_pin.high();
          memcpy(&prev_screen_buffer[_current_page * screen_width],
                 &screen_buffer[_current_page * screen_width], screen_width);
          _startNextDmaPage(_current_page + 1);
        }
      } else {
        _startNextDmaPage(0);
      }
    } else {
      updateDiff();
    }
  }

  void reset() {
    cs_pin.high();
    reset_pin.high();
    delay_msec(1);
    reset_pin.low();
    delay_msec(10);
    reset_pin.high();
    delay_msec(10);
  }

  void prepareGpio() {
    cs_pin.enable();
    dc_pin.enable();
    reset_pin.enable();

    cs_pin.initOutputDefault();
    dc_pin.initOutputDefault();
    reset_pin.initOutputDefault();
  }

  void sendInitCommands() {
    reset();
    delay_msec(100);

    sendCommand(0xAE); // display off
    sendCommand(0x20); // Set Memory Addressing Mode
    sendCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                             // 10b,Page Addressing Mode (RESET); 11b,Invalid
    sendCommand(0xB0); // Set Page Start Address for Page Addressing Mode,0-7
    sendCommand(0xC8); // Set COM Output Scan Direction
    sendCommand(0x00); // Set low column address
    sendCommand(0x10); // Set high column address
    sendCommand(0x40); // Set start line address - CHECK
    sendCommand(0x81); // Set contrast control register - CHECK
    sendCommand(0xFF);
    sendCommand(0xA1); // Set segment re-map 0 to 127 - CHECK
    sendCommand(0xA6); // Set normal color
    //sendCommand(0xA7); // Set invert color
    sendCommand(0xA8); // Set multiplex ratio(1 to 64) - CHECK
    sendCommand(0x3F); //
    sendCommand(0xA4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    sendCommand(0xD3); // Set display offset - CHECK
    sendCommand(0x00); // Not offset
    sendCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    //sendCommand(0xF0); // Set divide ratio
    sendCommand(0x80); // Set divide ratio
    sendCommand(0xD9); // Set pre-charge period
    //sendCommand(0x22); //
    sendCommand(0xF1); //
    sendCommand(0xDA); // Set com pins hardware configuration - CHECK
    sendCommand(0x12);
    sendCommand(0xDB); // Set vcomh
    sendCommand(0x20); // 0x20,0.77xVcc
    sendCommand(0x8D); // Set DC-DC enable
    sendCommand(0x14); //
    sendCommand(0x2E); // stop scroll
    sendCommand(0xAF); // turn on SSD1306 panel

    // clear screen
    drawFillBG();
    updateScreen();
  }

  void sendCommand(uint8_t byte) {
    cs_pin.low(); // select OLED
    dc_pin.low(); // command
    spi.sendU8sync(byte, _wait_func);
    cs_pin.high(); // un-select OLED
  }

  void sendData(uint8_t* buf, size_t buff_size) {
    dc_pin.high(); // data
    cs_pin.low(); spi.sendU8sync(0, _wait_func); cs_pin.high();
    cs_pin.low(); spi.sendU8sync(0, _wait_func); cs_pin.high();
    cs_pin.low(); spi.sendBufU8sync(buf, buff_size, _wait_func); cs_pin.high();
  }

  void updateScreen() {
    for (uint8_t i = 0; i < screen_height/8; ++i) {
      sendCommand(0xB0 + i); // Set the current RAM page address.
      sendCommand(0x00);
      sendCommand(0x10);
      sendData(&screen_buffer[screen_width*i], screen_width);
    }
    memcpy(prev_screen_buffer, screen_buffer, sizeof(screen_buffer));
  }

  // 変更ページのみ送信する差分更新 (sync)
  void updateDiff() {
    for (uint8_t i = 0; i < screen_height/8; ++i) {
      const size_t page_offset = screen_width * i;
      if (memcmp(&screen_buffer[page_offset], &prev_screen_buffer[page_offset], screen_width) == 0) continue;
      sendCommand(0xB0 + i);
      sendCommand(0x00);
      sendCommand(0x10);
      sendData(&screen_buffer[page_offset], screen_width);
      memcpy(&prev_screen_buffer[page_offset], &screen_buffer[page_offset], screen_width);
    }
  }

  // 変更ページを探してDMA転送を開始する。なければ完了。mainループから呼ぶこと(ISR不可)。
  void _startNextDmaPage(uint8_t from) {
    for (uint8_t page = from; page < screen_height / 8; ++page) {
      const size_t page_offset = screen_width * page;
      if (memcmp(&screen_buffer[page_offset], &prev_screen_buffer[page_offset], screen_width) != 0) {
        _current_page = page;
        _dma_busy = true;
        sendCommand(0xB0 + page);
        sendCommand(0x00);
        sendCommand(0x10);
        // D-Cache flush: ensure DMA reads CPU-written data from physical memory
        SCB_CleanDCache_by_Addr(
          reinterpret_cast<uint32_t*>(&screen_buffer[page_offset]),
          static_cast<int32_t>(screen_width));
        // Start DMA: CS stays low until process() calls sendBufU8dmaEnd() after _page_done
        dc_pin.high();
        cs_pin.low(); spi.sendU8sync(0, _wait_func); cs_pin.high();
        cs_pin.low(); spi.sendU8sync(0, _wait_func); cs_pin.high();
        cs_pin.low();
        spi.sendBufU8dmaStart(&screen_buffer[page_offset], screen_width, dma_stream);
        return;
      }
    }
    _dma_busy = false;
  }

  void drawFillBG() {
    for (size_t i = 0; i < sizeof(screen_buffer); ++i) {
      screen_buffer[i] = 0;
    }
  }

  void drawFillFG() {
    for (size_t i = 0; i < sizeof(screen_buffer); ++i) {
      screen_buffer[i] = 0xFF;
    }
  }

  void drawPixel(uint8_t x, uint8_t y, bool fg) {
    if (fg) {
      screen_buffer[x + (y / 8) * screen_width] |= 1 << (y % 8);
    } else {
      screen_buffer[x + (y / 8) * screen_width] &= ~(1 << (y % 8));
    }
  }

  void drawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool fg) {
    for (uint16_t i = 0; i < width; ++i) {
      drawPixel(x+i, y, fg);
      drawPixel(x+i, y+height, fg);
    }
    for (uint16_t i = 1; i < height-1; ++i) {
      drawPixel(x, y+i, fg);
      drawPixel(x+width, y+i, fg);
    }
  }

  void drawFillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool fg, bool fill_fg) {
    drawRect(x, y, width, height, fg);
    uint8_t fill_bit = fill_fg ? 0xFF : 0;
    for (uint16_t i = 1; i < height-1;) {
      if (((y+i) % 8) == 0 && ((i+8) < height)) {
        // draw by page
        uint8_t page = y+i / 8;
        for (uint8_t j = 1; j < width-1; ++j) {
          setPageBit(page, x+j, fill_bit);
        }
        i += 8;
      } else {
        // draw by pixel
        for (uint8_t j = 1; j < width-1; ++j) {
          drawPixel(x+j, y+i, fill_fg);
        }
        ++i;
      }
    }
  }

  void drawPageBit(uint8_t page, uint8_t x, uint8_t bit) {
    screen_buffer[x + page * screen_width] |= bit;
  }

  void setPageBit(uint8_t page, uint8_t x, uint8_t bit) {
    screen_buffer[x + page * screen_width] = bit;
  }

  void drawTextMedium(const char* text, uint8_t length, uint16_t page, uint16_t offset) {
    uint8_t pos = 0;
    for (uint8_t i=0; i<length; ++i) {
      char c = text[i];
      if (c == '\n') {
        page += 2;
        pos = 0;
        continue;
      }
      if (c < 32 || c > 126) {
        return; // null or meta char
      }
      const uint16_t* image = igb::font::cvfont_8_16[c-32];
      for (uint8_t x=0; x<8; ++x) {
        uint16_t bits = image[x];
        screen_buffer[((page)*screen_width)+(pos*8)+offset+x] = (uint8_t)bits;
        screen_buffer[((page+1)*screen_width)+(pos*8)+offset+x] = (uint8_t)(bits>>8);
      }
      ++pos;
    }
  }

  void drawTextSmall(const char* text, uint8_t length, uint16_t page, uint16_t offset) {
    uint8_t pos = 0;
    for (uint8_t i=0; i<length; ++i) {
      char c = text[i];
      if (c == '\n') {
        page += 1;
        pos = 0;
        continue;
      }
      if (c < 32 || c > 126) {
        return; // null or meta char
      }
      const uint8_t* image = igb::font::cvfont_5_8[c-32];
      for (uint8_t x=0; x<5; ++x) {
        uint8_t bits = image[x];
        screen_buffer[((page)*screen_width)+(pos*5)+offset+x] = bits;
      }
      ++pos;
    }
  }

  void drawInvert(uint16_t page, uint16_t offset, uint16_t length) {
    for (uint16_t x = offset; x < offset+length; ++x) {
      screen_buffer[(page * screen_width) + x] ^= 0xFF;
    }
  }
};

}
}

