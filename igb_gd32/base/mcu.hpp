#ifndef IGB_GD32_BASE_MCU_H
#define IGB_GD32_BASE_MCU_H

#include <igb_gd32/base/define.hpp>
#include <igb_util/macro.hpp>

// GD32_MCU_NAME の定義値に応じて、それぞれのMCU用のヘッダを include する
// -- 
// '.' がトークン扱いされないのは GCC 限定かもしれない
#define GD32_BASE_MCU_HPP_FILE(mcusym) <igb_gd32/base/mcu/UNWRAP(mcusym).hpp>
// '.' がトークン扱いされる環境だとこうか？
// #define _WRAPPING(name) name
// #define GD32_BASE_MCU_HPP_FILE(mcusym) <igb_gd32/device/mcu/_WRAPPING(mcusym) ## .hpp>

#include GD32_BASE_MCU_HPP_FILE(GD32_MCU)

#endif /* IGB_GD32_BASE_MCU_H */
