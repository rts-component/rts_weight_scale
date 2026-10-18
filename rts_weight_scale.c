/*
 * Copyright (c) 2020-2026, vaelin.zheng@gmail.com
 *
 * SPDX-License-Identifier: MIT
 *
 * Change Logs:
 * Date             Author                  Notes
 * 2026-10-18       Vaelin.Zheng            Init
*/

#include "rts_weight_scale.h"

#if (CONFIG_RTS_WEIGHT_SCALE_ENABLE)
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if(CONFIG_RTS_WS_DEBUG_ENABLE)
#include <assert.h>
#include <stdio.h>
#define RTS_WS_ASSERT       assert
#define RTS_WS_DEBUG_PRINT  printf
#else
#define RTS_WS_ASSERT(...)      do {} while(0)
#define RTS_WS_DEBUG_PRINT(...) do {} while(0)
#endif

_Static_assert(CONFIG_RTS_WS_STABLE_SAMPLE_COUNT > 0, "CONFIG_RTS_WS_STABLE_SAMPLE_COUNT must be greater than 0");

static void rts_ws_reset_stability(rts_ws_t *self)
{
    RTS_WS_ASSERT(self != NULL);

    self->stable_sample_count = 0;
    self->stable_sample_index = 0;
    self->is_stable           = false;
}

static rts_ws_status_t rts_ws_convert_raw(const rts_ws_t *self, uint32_t raw, int32_t *weight)
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(weight != NULL);

    const size_t point_count = self->calibration.point_count;
    if(point_count < 2)
    {
        return RTS_WS_NOT_CALIBRATED;
    }
    if(point_count > CONFIG_RTS_WS_MAX_CALIBRATION_POINTS)
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    // Find the segment of the calibration curve that contains the raw value
    // n0 -- n1 -- n2 -- n3
    size_t segment = 0;
    if(raw >= self->calibration.raw[point_count - 1])
    {
        segment = point_count - 2;
    }
    else
    {
        for(size_t i = 0; i + 1 < point_count; i++)
        {
            if(raw <= self->calibration.raw[i + 1])
            {
                segment = i;
                break;
            }
        }
    }

    const int64_t raw_delta           = (int64_t)raw - (int64_t)self->calibration.raw[segment];
    const int64_t scaled_weight_delta = raw_delta * self->k_factor_x10000[segment];
    const int64_t computed_weight     = (int64_t)self->calibration.weight[segment] + scaled_weight_delta / 10000;

    if(computed_weight < INT32_MIN || computed_weight > INT32_MAX)
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    *weight = (int32_t)computed_weight;
    return RTS_WS_OK;
}

rts_ws_status_t rts_ws_init(rts_ws_t *self, bool (*is_raw_ready)(void), uint32_t (*read_raw)(void))
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(is_raw_ready != NULL);
    RTS_WS_ASSERT(read_raw != NULL);

    memset(self, 0, sizeof(*self));
    self->is_raw_ready = is_raw_ready;
    self->read_raw     = read_raw;

    return RTS_WS_OK;
}

bool rts_ws_raw_is_ready(const rts_ws_t *self)
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(self->is_raw_ready != NULL);

    return self->is_raw_ready();
}

rts_ws_status_t rts_ws_raw_read(const rts_ws_t *self, uint32_t *raw)
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(self->is_raw_ready != NULL);
    RTS_WS_ASSERT(self->read_raw != NULL);
    RTS_WS_ASSERT(raw != NULL);

    if(!self->is_raw_ready())
    {
        return RTS_WS_NO_SAMPLE;
    }

    *raw = self->read_raw();
    return RTS_WS_OK;
}

rts_ws_status_t rts_ws_set_calibrate(rts_ws_t *self, const rts_ws_calibration_t *calibration)
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(calibration != NULL);

    if((calibration->point_count < 2) || (calibration->point_count > CONFIG_RTS_WS_MAX_CALIBRATION_POINTS))
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    const size_t point_count                           = calibration->point_count;
    int weight_direction                               = 0;
    int32_t factors[CONFIG_RTS_WS_MAX_CALIBRATION_POINTS - 1] = {0};

    for(size_t i = 1; i < point_count; ++i)
    {
        // Ensure that the raw values are strictly increasing
        if(calibration->raw[i] <= calibration->raw[i - 1])
        {
            return RTS_WS_INVALID_CALIBRATION;
        }

        const int64_t weight_delta = (int64_t)calibration->weight[i] - calibration->weight[i - 1];
        if(weight_delta == 0)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }

        // Ensure that the weight values are either strictly increasing or strictly decreasing
        const int current_direction = weight_delta > 0 ? 1 : -1;
        if(weight_direction != 0 && current_direction != weight_direction)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }
        weight_direction = current_direction;

        const double factor         = (double)weight_delta * 10000.0 / ((double)calibration->raw[i] - calibration->raw[i - 1]);
        const double rounded_factor = factor >= 0.0 ? factor + 0.5 : factor - 0.5;
        if(!isfinite(rounded_factor) ||
           rounded_factor < (double)INT32_MIN ||
           rounded_factor > (double)INT32_MAX)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }
        factors[i - 1] = (int32_t)rounded_factor;
    }

    self->calibration = *calibration;
    memcpy(self->k_factor_x10000, factors, sizeof(factors));
    self->tare_weight = 0;
    rts_ws_reset_stability(self);
    return RTS_WS_OK;
}

const rts_ws_calibration_t *rts_ws_get_calibrate(const rts_ws_t *self)
{
    RTS_WS_ASSERT(self != NULL);

    return &self->calibration;
}

rts_ws_status_t rts_ws_set_tare(rts_ws_t *self)
{
    RTS_WS_ASSERT(self != NULL);

    if(!self->is_stable)
    {
        return RTS_WS_NO_SAMPLE;
    }

    self->tare_weight = self->weight;

    return RTS_WS_OK;
}

rts_ws_status_t rts_ws_get_weight(const rts_ws_t *self, int32_t *weight)
{

    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(weight != NULL);

    *weight = self->weight - self->tare_weight;

    return RTS_WS_OK;
}

static uint32_t rts_ws_raw_filter(uint32_t v)
{
    // todo: implement a better filter
    return v;
}

void rts_ws_task(rts_ws_t *self)
{
    RTS_WS_ASSERT(self != NULL);
    RTS_WS_ASSERT(self->is_raw_ready != NULL);
    RTS_WS_ASSERT(self->read_raw != NULL);

    if(self->is_raw_ready())
    {
        uint32_t raw            = 0;
        uint32_t raw_filtered   = 0;
        int32_t weight          = 0;
        int32_t weight_filtered = 0;
        // read raw value
        if(RTS_WS_OK != rts_ws_raw_read(self, &raw))
        {
            rts_ws_reset_stability(self);
            return;
        }

        // filter raw value
        raw_filtered = rts_ws_raw_filter(raw);

        // convert raw to weight
        if(RTS_WS_OK != rts_ws_convert_raw(self, raw_filtered, &weight))
        {
            rts_ws_reset_stability(self);
            return;
        }

        // store weight in stable sample buffer and stability check
        self->stable_sample[self->stable_sample_index] = weight;
        self->stable_sample_index++;
        if(self->stable_sample_index >= CONFIG_RTS_WS_STABLE_SAMPLE_COUNT) self->stable_sample_index = 0;
        if(self->stable_sample_count < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT)
        {
            self->stable_sample_count++;
        }

        int32_t min_weight = self->stable_sample[0];
        int32_t max_weight = self->stable_sample[0];
        int64_t sum_weight = self->stable_sample[0];

        for(size_t i = 1; i < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT; ++i)
        {
            sum_weight += self->stable_sample[i];
            if(self->stable_sample[i] < min_weight)
            {
                min_weight = self->stable_sample[i];
            }
            if(self->stable_sample[i] > max_weight)
            {
                max_weight = self->stable_sample[i];
            }
        }
        weight_filtered = sum_weight / CONFIG_RTS_WS_STABLE_SAMPLE_COUNT;

        if((self->stable_sample_count >= CONFIG_RTS_WS_STABLE_SAMPLE_COUNT) && self->is_stable)
        {
            if((weight_filtered < (self->weight - CONFIG_RTS_WS_STABLE_RANGE)) ||
               (weight_filtered > (self->weight + CONFIG_RTS_WS_STABLE_RANGE)))
            {
                self->is_stable = false;
                rts_ws_reset_stability(self);
            }
        }

        if(!self->is_stable)
        {
            if((self->stable_sample_count >= CONFIG_RTS_WS_STABLE_SAMPLE_COUNT) &&
               ((int64_t)max_weight - min_weight <= CONFIG_RTS_WS_STABLE_RANGE))
            {
                self->is_stable = true;
            }
            self->weight = weight_filtered;
        }

        #if(CONFIG_RTS_WS_DEBUG_ENABLE)
        // raw, raw_filtered, weight, weight_filtered, min_weight, max_weight, stable, weight
        static bool is_title_printed = false;
        if(!is_title_printed)
        {
            RTS_WS_DEBUG_PRINT("raw, raw_filtered, weight, weight_filtered, min_weight, max_weight, is_stable, weight\n");
            is_title_printed = true;
        }
        RTS_WS_DEBUG_PRINT("%u, %u, %d, %d, %d, %d, %d, %d\n",
                           raw,
                           raw_filtered,
                           weight,
                           weight_filtered,
                           min_weight,
                           max_weight,
                           self->is_stable,
                           self->weight);
        #endif
    }
}

#endif // CONFIG_RTS_WEIGHT_SCALE_ENABLE