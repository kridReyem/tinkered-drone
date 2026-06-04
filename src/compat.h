/**
 * STM32 / arm-none-eabi Compatibility Shim
 *
 * Problem: <cstdlib> (used by Adafruit Unified Sensor and others) does
 *   `using ::abort; using ::abs; ...` — but the ARM newlib-nano runtime
 *   does NOT forward these into the global `::` namespace by default when
 *   compiled as C++.  The result is "abort has not been declared in '::'"
 *   at cstdlib line 27+.
 *
 * Fix: include the plain C headers first (which DO populate `::`) so that
 *   the C++ wrappers can find them when they run `using ::abort` etc.
 *
 * This file is force-included via build_flags = -include src/compat.h
 * so it runs before any library header.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
}
#endif
