add_library(igb_util STATIC tbl.cpp random.cpp dsp/cycle_speed.cpp dsp/dsp_tbl_func.cpp)

target_include_directories(igb_util PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
