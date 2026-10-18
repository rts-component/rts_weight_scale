/*
 * Copyright (c) 2020-2026, vaelin.zheng@gmail.com
 *
 * SPDX-License-Identifier: MIT
 *
 * Change Logs:
 * Date             Author                  Notes
 * 2026-10-18       Vaelin.Zheng            Init
*/

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#if defined(KCONFIG_HEADER_FILE)
    #if defined(__C51__) || defined(__ARMCC_VERSION)
        #define _RTS_MACRO_TO_STR_IMPL(s) #s
        #define _RTS_MACRO_TO_STR(s) _RTS_MACRO_TO_STR_IMPL(s)
        #include _RTS_MACRO_TO_STR(KCONFIG_HEADER_FILE)
        #undef _RTS_MACRO_TO_STR
        #undef _RTS_MACRO_TO_STR_IMPL
    #else
        #include KCONFIG_HEADER_FILE
    #endif
#endif

#if (CONFIG_RTS_WEIGHT_SCALE_ENABLE)

#include "stdint.h"
#include "stdbool.h"

#ifndef CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT
#define CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT   3
#endif

#ifndef CONFIG_RTS_WS_MAX_CALIBRATION_POINTS
#define CONFIG_RTS_WS_MAX_CALIBRATION_POINTS    4
#endif

#ifndef CONFIG_RTS_WS_STABLE_RANGE
#define CONFIG_RTS_WS_STABLE_RANGE              2
#endif

#ifndef CONFIG_RTS_WS_STABLE_SAMPLE_COUNT
#define CONFIG_RTS_WS_STABLE_SAMPLE_COUNT       5
#endif

_Static_assert(CONFIG_RTS_WS_MAX_CALIBRATION_POINTS >= 2, "CONFIG_RTS_WS_MAX_CALIBRATION_POINTS must be at least 2");
_Static_assert(CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT >= 3,"CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT must be at least 3");
_Static_assert((CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT % 2) == 1, "CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT must be odd");
_Static_assert(CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT <= 31, "CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT must not exceed 31");

typedef enum
{
    RTS_WS_OK = 0,
    RTS_WS_INVALID_ARGUMENT,
    RTS_WS_INVALID_CALIBRATION,
    RTS_WS_NOT_CALIBRATED,
    RTS_WS_NO_SAMPLE,
    RTS_WS_WEIGHT_OUT_OF_RANGE
} rts_ws_status_t;

typedef struct
{
    uint8_t point_count;
    uint32_t raw[CONFIG_RTS_WS_MAX_CALIBRATION_POINTS];
    int32_t weight[CONFIG_RTS_WS_MAX_CALIBRATION_POINTS];
} rts_ws_calibration_t;

typedef struct
{
    bool (*is_raw_ready)(void);
    uint32_t (*read_raw)(void);

    uint32_t raw_filter_sample[CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT];
    uint16_t raw_filter_sample_index;
    bool raw_filter_initialized;

    rts_ws_calibration_t calibration;
    int32_t k_factor_x10000[CONFIG_RTS_WS_MAX_CALIBRATION_POINTS - 1];

    int32_t stable_sample[CONFIG_RTS_WS_STABLE_SAMPLE_COUNT];
    uint16_t stable_sample_count;
    uint16_t stable_sample_index;
    bool is_stable;

    int32_t tare_weight;
    int32_t weight;
} rts_ws_t;

rts_ws_status_t rts_ws_init(rts_ws_t *self,
                            bool (*is_raw_ready)(void),
                            uint32_t (*read_raw)(void));

bool rts_ws_raw_is_ready(const rts_ws_t *self);
rts_ws_status_t rts_ws_raw_read(const rts_ws_t *self, uint32_t *raw);

rts_ws_status_t rts_ws_set_calibrate(rts_ws_t *self, const rts_ws_calibration_t *calibration);
const rts_ws_calibration_t *rts_ws_get_calibrate(const rts_ws_t *self);

rts_ws_status_t rts_ws_set_tare(rts_ws_t *self);
rts_ws_status_t rts_ws_get_weight(const rts_ws_t *self, int32_t *weight);

void rts_ws_task(rts_ws_t *self);

#endif // CONFIG_RTS_WEIGHT_SCALE_ENABLE
#ifdef __cplusplus
}
#endif
