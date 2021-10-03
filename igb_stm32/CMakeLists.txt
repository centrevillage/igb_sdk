add_library(igb_stm32 STATIC periph/systick.cpp)

target_include_directories(igb_stm32 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(igb_stm32 PRIVATE 
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
)
