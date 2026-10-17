#pragma once
#include "stdint.h"
#include "stdbool.h"

#define RTS_WS_MAX_CALIBRATION_POINTS     4
#define CONFIG_RTS_WS_STABLE_RANGE        10
#define CONFIG_RTS_WS_STABLE_SAMPLE_COUNT 5

_Static_assert(RTS_WS_MAX_CALIBRATION_POINTS >= 2, "RTS_WS_MAX_CALIBRATION_POINTS must be at least 2");

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
    uint32_t raw[RTS_WS_MAX_CALIBRATION_POINTS];
    int32_t weight[RTS_WS_MAX_CALIBRATION_POINTS];
} rts_ws_calibration_t;

typedef struct
{
    bool (*is_raw_ready)(void);
    uint32_t (*read_raw)(void);

    rts_ws_calibration_t calibration;
    int32_t k_factor_x10000[RTS_WS_MAX_CALIBRATION_POINTS - 1];
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