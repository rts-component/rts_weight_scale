# RTS Weight Scale

A small, ADC-independent C library for converting raw measurement values to
weight using a multi-point calibration curve. The library does not acquire
samples, configure an ADC, or depend on a particular sensor driver.

## Calibration

`rts_weight_scale_set_calibration` accepts 2 to
`RTS_WEIGHT_SCALE_MAX_SCALES` calibration points. Each point pairs a raw value
with its known weight. Raw values must be strictly increasing, and weights must
be strictly monotonic (either increasing or decreasing). Include the unloaded
reading with a weight of `0.0f` if the calibration should produce zero at that
reading.

Weights between calibration points are linearly interpolated. Readings outside
the calibrated range are linearly extrapolated using the nearest segment. A
failed calibration update leaves the previous calibration unchanged.

## Example

```c
#include "rts_weight_scale.h"

rts_weight_scale_t scale = {0};
const rts_weight_scale_point_t points[] = {
    {120000, 0.0f},
    {135500, 1.0f},
    {151300, 2.0f},
};

if (rts_weight_scale_set_calibration(&scale, points, 3) ==
    RTS_WEIGHT_SCALE_OK)
{
    float weight;
    if (rts_weight_scale_get_weight(&scale, sensor_raw_value, &weight) ==
        RTS_WEIGHT_SCALE_OK)
    {
        /* Use weight. */
    }
}
```

The acquisition layer is responsible for obtaining `sensor_raw_value` and
passing it to the conversion function.