#include "ostrich.h"

#include <libopencm3/stm32/rcc.h>

namespace Ostrich {
BoardConfig MakeBoardConfig() {
  BoardConfig bc;

  // For recording.
  //bc.clock_scale = rcc_3v3[RCC_CLOCK_3V3_216MHZ];
  //bc.systick_period_clocks = 216; // 1 us

  // For operation.
  bc.clock_scale = rcc_3v3[RCC_CLOCK_3V3_72MHZ];
  bc.systick_period_clocks = 200; // 2 us

  bc.hse_mhz = 12;
  bc.use_hse = true;

  bc.vdd_voltage_mV = 3300;

  return bc;
}
}
