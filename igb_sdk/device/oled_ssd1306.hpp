#ifndef IGB_SDK_DEVICE_OLED_SSD1306_H
#define IGB_SDK_DEVICE_OLED_SSD1306_H

#include <igb_stm32/periph/systick.hpp>
#include <igb_sdk/base.hpp>
#include <igb_sdk/font/bitmap/cvfont_5x8.h>
#include <igb_sdk/font/bitmap/cvfont_8x16.h>

namespace igb {
namespace sdk {

template<typename SPI_TYPE, typename GPIO_PIN_TYPE, size_t screen_width = 128, size_t screen_height = 64>
struct OledSsd1306 {
  SPI_TYPE& spi;
  GPIO_PIN_TYPE cs_pin;
  GPIO_PIN_TYPE dc_pin;
  GPIO_PIN_TYPE reset_pin;

  uint8_t screen_buffer[screen_width * screen_height / 8];

  void init() {
    prepareGpio();
    sendInitCommands();
  }

  void process() {
    updateScreen();
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
  }

  void sendCommand(uint8_t byte) {
    cs_pin.low(); // select OLED
    dc_pin.low(); // command
    spi.sendU8sync(byte);
    cs_pin.high(); // un-select OLED
  }

  void sendData(uint8_t* buf, size_t buff_size) {
    dc_pin.high(); // data
    cs_pin.low(); spi.sendU8sync(0); cs_pin.high();
    cs_pin.low(); spi.sendU8sync(0); cs_pin.high();
    cs_pin.low(); spi.sendBufU8sync(buf, buff_size); cs_pin.high();
  }

  void updateScreen() {
    for (uint8_t i = 0; i < screen_height/8; ++i) {
      sendCommand(0xB0 + i); // Set the current RAM page address.
      sendCommand(0x00);
      sendCommand(0x10);
      sendData(&screen_buffer[screen_width*i], screen_width);
    }
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
      const uint16_t* image = cvfont_8_16[c-32];
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
      const uint8_t* image = cvfont_5_8[c-32];
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

#endif /* IGB_SDK_DEVICE_OLED_SSD1306_H */
