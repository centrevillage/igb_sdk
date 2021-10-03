file(GLOB FONT_SRCS font/bitmap/*.cpp)
add_library(igb_sdk STATIC util/text.c ${FONT_SRCS})

target_include_directories(igb_sdk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
