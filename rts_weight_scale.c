#include "rts_weight_scale.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(CONFIG_RTS_WS_STABLE_SAMPLE_COUNT > 0,
               "CONFIG_RTS_WS_STABLE_SAMPLE_COUNT must be greater than 0");

static void rts_ws_reset_stability(rts_ws_t *self)
{
    self->stable_sample_count = 0;
    self->stable_sample_index = 0;
}

static rts_ws_status_t rts_ws_convert_raw(
    const rts_ws_t *self,
    uint32_t raw,
    int32_t *weight)
{
    const size_t point_count = self->calibration.point_count;
    if(point_count < 2)
    {
        return RTS_WS_NOT_CALIBRATED;
    }
    if(point_count > RTS_WS_MAX_CALIBRATION_POINTS)
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    size_t segment = 0;
    if(raw >= self->calibration.raw[point_count - 1])
    {
        segment = point_count - 2;
    }
    else
    {
        for(size_t i = 0; i + 1 < point_count; ++i)
        {
            if(raw <= self->calibration.raw[i + 1])
            {
                segment = i;
                break;
            }
        }
    }

    const double raw_start    = self->calibration.raw[segment];
    const double raw_end      = self->calibration.raw[segment + 1];
    const double weight_start = self->calibration.weight[segment];
    const double weight_end   = self->calibration.weight[segment + 1];
    const double ratio        = ((double)raw - raw_start) / (raw_end - raw_start);
    const double result       = weight_start + ratio * (weight_end - weight_start);
    const double rounded      = result >= 0.0 ? result + 0.5 : result - 0.5;

    if(!isfinite(rounded) || rounded < (double)INT32_MIN ||
       rounded > (double)INT32_MAX)
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    *weight = (int32_t)rounded;
    return RTS_WS_OK;
}

rts_ws_status_t rts_ws_init(
    rts_ws_t *self,
    bool (*is_raw_ready)(void),
    uint32_t (*read_raw)(void))
{
    if(self == NULL || is_raw_ready == NULL || read_raw == NULL)
    {
        return RTS_WS_INVALID_ARGUMENT;
    }

    memset(self, 0, sizeof(*self));
    self->is_raw_ready = is_raw_ready;
    self->read_raw     = read_raw;
    return RTS_WS_OK;
}

bool rts_ws_raw_is_ready(const rts_ws_t *self)
{
    return (self != NULL) && (self->is_raw_ready != NULL) &&
           (self->read_raw != NULL) && self->is_raw_ready();
}

rts_ws_status_t rts_ws_raw_read(const rts_ws_t *self, uint32_t *raw)
{
    if(self == NULL || raw == NULL || self->is_raw_ready == NULL ||
       self->read_raw == NULL)
    {
        return RTS_WS_INVALID_ARGUMENT;
    }

    if(!self->is_raw_ready())
    {
        return RTS_WS_NO_SAMPLE;
    }

    *raw = self->read_raw();
    return RTS_WS_OK;
}

rts_ws_status_t rts_ws_set_calibrate(rts_ws_t *self, const rts_ws_calibration_t *calibration)
{
    if(self == NULL || calibration == NULL)
    {
        return RTS_WS_INVALID_ARGUMENT;
    }

    if((calibration->point_count < 2) || (calibration->point_count > RTS_WS_MAX_CALIBRATION_POINTS))
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    const size_t point_count                           = calibration->point_count;
    int weight_direction                               = 0;
    int32_t factors[RTS_WS_MAX_CALIBRATION_POINTS - 1] = {0};

    for(size_t i = 1; i < point_count; ++i)
    {
        if(calibration->raw[i] <= calibration->raw[i - 1])
        {
            return RTS_WS_INVALID_CALIBRATION;
        }

        const int64_t weight_delta = (int64_t)calibration->weight[i] - calibration->weight[i - 1];
        if(weight_delta == 0)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }

        const int current_direction = weight_delta > 0 ? 1 : -1;
        if(weight_direction != 0 && current_direction != weight_direction)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }
        weight_direction = current_direction;

        const double factor =
            (double)weight_delta * 1000.0 /
            ((double)calibration->raw[i] - calibration->raw[i - 1]);
        const double rounded_factor =
            factor >= 0.0 ? factor + 0.5 : factor - 0.5;
        if(!isfinite(rounded_factor) ||
           rounded_factor < (double)INT32_MIN ||
           rounded_factor > (double)INT32_MAX)
        {
            return RTS_WS_INVALID_CALIBRATION;
        }
        factors[i - 1] = (int32_t)rounded_factor;
    }

    self->calibration = *calibration;
    memcpy(self->k_factor_x1000, factors, sizeof(factors));
    rts_ws_reset_stability(self);
    return RTS_WS_OK;
}

const rts_ws_calibration_t *rts_ws_get_calibrate(const rts_ws_t *self)
{
    if(self == NULL || self->calibration.point_count < 2 ||
       self->calibration.point_count > RTS_WS_MAX_CALIBRATION_POINTS)
    {
        return NULL;
    }

    return &self->calibration;
}

bool rts_ws_is_stable(const rts_ws_t *self)
{
    if((self == NULL) || (self->stable_sample_count < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT))
    {
        return false;
    }

    int32_t min_weight = self->stable_sample[0];
    int32_t max_weight = self->stable_sample[0];
    for(size_t i = 1; i < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT; ++i)
    {
        if(self->stable_sample[i] < min_weight)
        {
            min_weight = self->stable_sample[i];
        }
        if(self->stable_sample[i] > max_weight)
        {
            max_weight = self->stable_sample[i];
        }
    }

    return (int64_t)max_weight - min_weight <= CONFIG_RTS_WS_STABLE_RANGE;
}

rts_ws_status_t rts_ws_get_weight(const rts_ws_t *self, int32_t *weight)
{
    if((self == NULL) || (weight == NULL))
    {
        return RTS_WS_INVALID_ARGUMENT;
    }
    if(self->calibration.point_count < 2)
    {
        return RTS_WS_NOT_CALIBRATED;
    }
    if(self->calibration.point_count > RTS_WS_MAX_CALIBRATION_POINTS)
    {
        return RTS_WS_INVALID_CALIBRATION;
    }

    uint32_t raw                      = 0;
    const rts_ws_status_t read_status = rts_ws_raw_read(self, &raw);
    if(read_status != RTS_WS_OK)
    {
        return read_status;
    }

    return rts_ws_convert_raw(self, raw, weight);
}

void rts_ws_task(rts_ws_t *self)
{
    if(self == NULL)
    {
        return;
    }

    uint32_t raw = 0;
    if(rts_ws_raw_read(self, &raw) != RTS_WS_OK)
    {
        rts_ws_reset_stability(self);
        return;
    }

    int32_t weight = 0;
    if(rts_ws_convert_raw(self, raw, &weight) != RTS_WS_OK)
    {
        rts_ws_reset_stability(self);
        return;
    }

    self->stable_sample[self->stable_sample_index] = weight;
    self->stable_sample_index =
        (uint16_t)((self->stable_sample_index + 1) %
                   CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    if(self->stable_sample_count < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT)
    {
        ++self->stable_sample_count;
    }
}
