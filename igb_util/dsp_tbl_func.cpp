#include <igb_util/dsp_tbl_func.hpp>

namespace igb {

#if defined(USE_SMALL_DSP_TABLE)

#include "_dsp_tbl_func_256.hpp"

#else

#include "_dsp_tbl_func_1024.hpp"

#endif

}
