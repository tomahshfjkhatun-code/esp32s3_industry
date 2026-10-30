#pragma once

// 1: only the independent CAN/M31 bench task starts. 0: original 702 tasks start.
#ifndef PJ_BENCH_MODE
#define PJ_BENCH_MODE 0
#endif

// -1: unknown, refuse all DO writes. 1: Modbus coil true energizes DO.
// 0: Modbus coil false energizes DO. Confirm the actual M31 wiring before changing.
#ifndef PJ_DO_LEVEL
#define PJ_DO_LEVEL -1
#endif

#if PJ_BENCH_MODE != 0 && PJ_BENCH_MODE != 1
#error PJ_BENCH_MODE must be 0 or 1
#endif
#if PJ_DO_LEVEL < -1 || PJ_DO_LEVEL > 1
#error PJ_DO_LEVEL must be -1, 0 or 1
#endif
