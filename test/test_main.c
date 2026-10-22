#include "../rts_weight_scale.h"

#include "unity.h"

#include <limits.h>

static rts_ws_t ws;
static bool raw_ready;
static uint32_t raw_value;
static unsigned int read_raw_call_count;

static bool mock_is_raw_ready(void)
{
    return raw_ready;
}

static uint32_t mock_read_raw(void)
{
    read_raw_call_count++;
    raw_ready = false;
    return raw_value;
}

static void reset_fixture(void)
{
    raw_ready           = false;
    raw_value           = 0;
    read_raw_call_count = 0;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_init(&ws, mock_is_raw_ready, mock_read_raw));
}

static void build_calibration(rts_ws_calibration_t *calibration, uint8_t point_count,
                              const uint32_t *raw, const int32_t *weight)
{
    calibration->point_count = point_count;
    for(uint8_t i = 0; i < point_count; ++i)
    {
        calibration->raw[i]    = raw[i];
        calibration->weight[i] = weight[i];
    }
}

static void set_two_point_calibration(uint32_t raw0, int32_t weight0,
                                      uint32_t raw1, int32_t weight1)
{
    const uint32_t raw[2]    = {raw0, raw1};
    const int32_t weight[2]  = {weight0, weight1};
    rts_ws_calibration_t calibration;

    build_calibration(&calibration, 2, raw, weight);
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &calibration));
}

static void feed_raw(uint32_t raw)
{
    raw_ready = true;
    raw_value = raw;
    rts_ws_task(&ws);
}

static void feed_constant_raw(uint32_t raw, unsigned int count)
{
    for(unsigned int i = 0; i < count; ++i)
    {
        feed_raw(raw);
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_if_init_clears_state_and_stores_callbacks(void)
{
    ws.weight      = 123;
    ws.tare_weight = 45;
    ws.is_stable   = true;

    reset_fixture();

    TEST_ASSERT_EQUAL_PTR(mock_is_raw_ready, ws.is_raw_ready);
    TEST_ASSERT_EQUAL_PTR(mock_read_raw, ws.read_raw);
    TEST_ASSERT_EQUAL_INT(0, ws.weight);
    TEST_ASSERT_EQUAL_INT(0, ws.tare_weight);
    TEST_ASSERT_FALSE(ws.is_stable);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_count);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_index);
    TEST_ASSERT_EQUAL_UINT16(0, ws.raw_filter_sample_index);
    TEST_ASSERT_FALSE(ws.raw_filter_initialized);
    TEST_ASSERT_EQUAL_UINT8(0, ws.calibration.point_count);
}

void test_if_raw_read_returns_no_sample_when_data_is_not_ready(void)
{
    uint32_t raw = 0x12345678u;
    reset_fixture();

    TEST_ASSERT_FALSE(rts_ws_raw_is_ready(&ws));
    TEST_ASSERT_EQUAL(RTS_WS_NO_SAMPLE, rts_ws_raw_read(&ws, &raw));
    TEST_ASSERT_EQUAL_HEX32(0x12345678u, raw);
    TEST_ASSERT_EQUAL_UINT(0, read_raw_call_count);
}

void test_if_raw_read_returns_ready_sample(void)
{
    uint32_t raw = 0;
    reset_fixture();
    raw_ready = true;
    raw_value = 0xabcdef01u;

    TEST_ASSERT_TRUE(rts_ws_raw_is_ready(&ws));
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_raw_read(&ws, &raw));
    TEST_ASSERT_EQUAL_HEX32(0xabcdef01u, raw);
    TEST_ASSERT_EQUAL_UINT(1, read_raw_call_count);
    TEST_ASSERT_FALSE(rts_ws_raw_is_ready(&ws));
}

void test_if_task_leaves_state_unchanged_without_a_ready_sample(void)
{
    reset_fixture();
    ws.weight              = 42;
    ws.is_stable           = true;
    ws.stable_sample_count = 3;
    ws.stable_sample_index = 2;

    rts_ws_task(&ws);

    TEST_ASSERT_EQUAL_INT(42, ws.weight);
    TEST_ASSERT_TRUE(ws.is_stable);
    TEST_ASSERT_EQUAL_UINT16(3, ws.stable_sample_count);
    TEST_ASSERT_EQUAL_UINT16(2, ws.stable_sample_index);
    TEST_ASSERT_EQUAL_UINT(0, read_raw_call_count);
}

void test_if_calibration_accepts_two_and_maximum_point_counts(void)
{
    reset_fixture();
    const rts_ws_calibration_t two_points = {
        .point_count = 2,
        .raw         = {10, 20},
        .weight      = {0, 100},
    };
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &two_points));
    TEST_ASSERT_EQUAL_UINT8(2, rts_ws_get_calibrate(&ws)->point_count);

    rts_ws_calibration_t maximum_points = {
        .point_count = CONFIG_RTS_WS_MAX_CALIBRATION_POINTS,
    };
    const uint8_t last_point = CONFIG_RTS_WS_MAX_CALIBRATION_POINTS - 1;

    for(uint8_t i = 0; i <= last_point; ++i)
    {
        maximum_points.raw[i]    = 10u * (i + 1u);
        maximum_points.weight[i] = 100 * (int32_t)i;
    }

    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &maximum_points));
    TEST_ASSERT_EQUAL_UINT8(CONFIG_RTS_WS_MAX_CALIBRATION_POINTS,
                            rts_ws_get_calibrate(&ws)->point_count);
    TEST_ASSERT_EQUAL_UINT32(10u * (last_point + 1u),
                             rts_ws_get_calibrate(&ws)->raw[last_point]);
    TEST_ASSERT_EQUAL_INT32(100 * (int32_t)last_point,
                            rts_ws_get_calibrate(&ws)->weight[last_point]);
}

void test_if_calibration_rejects_invalid_point_counts(void)
{
    reset_fixture();
    rts_ws_calibration_t calibration = {
        .point_count = 1,
        .raw         = {10, 20},
        .weight      = {0, 100},
    };
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));

    calibration.point_count = CONFIG_RTS_WS_MAX_CALIBRATION_POINTS + 1;
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));
}

void test_if_calibration_rejects_non_increasing_raw_values(void)
{
    const uint32_t raw_points[3]     = {10, 10, 30};
    const int32_t weight_points[3]   = {0, 100, 200};
    const uint8_t point_count        = CONFIG_RTS_WS_MAX_CALIBRATION_POINTS >= 3 ? 3 : 2;
    rts_ws_calibration_t calibration;

    reset_fixture();
    build_calibration(&calibration, point_count, raw_points, weight_points);
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));

    calibration.raw[1] = 5;
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));
}

void test_if_calibration_rejects_non_monotonic_weight_values(void)
{
    const uint32_t raw_points[3]           = {10, 20, 30};
    const int32_t equal_weights[3]         = {0, 0, 100};
    const int32_t direction_change[3]      = {0, 100, 50};
    const int32_t decreasing_weights[3]    = {100, 50, 0};
    const uint8_t point_count              = CONFIG_RTS_WS_MAX_CALIBRATION_POINTS >= 3 ? 3 : 2;
    rts_ws_calibration_t calibration;

    reset_fixture();
    build_calibration(&calibration, point_count, raw_points, equal_weights);
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));

    if(point_count >= 3)
    {
        build_calibration(&calibration, point_count, raw_points, direction_change);
        TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                          rts_ws_set_calibrate(&ws, &calibration));
    }

    build_calibration(&calibration, point_count, raw_points, decreasing_weights);
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &calibration));
}

void test_if_calibration_rejects_out_of_range_conversion_factor(void)
{
    reset_fixture();
    const rts_ws_calibration_t calibration = {
        .point_count = 2,
        .raw         = {1, 2},
        .weight      = {INT32_MIN, INT32_MAX},
    };

    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &calibration));
}

void test_if_invalid_calibration_does_not_replace_previous_calibration(void)
{
    reset_fixture();
    set_two_point_calibration(100, 0, 200, 100);
    ws.tare_weight = 25;

    const rts_ws_calibration_t invalid_calibration = {
        .point_count = 2,
        .raw         = {200, 100},
        .weight      = {0, 100},
    };
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&ws, &invalid_calibration));

    TEST_ASSERT_EQUAL_UINT32(100, rts_ws_get_calibrate(&ws)->raw[0]);
    TEST_ASSERT_EQUAL_UINT32(200, rts_ws_get_calibrate(&ws)->raw[1]);
    TEST_ASSERT_EQUAL_INT32(25, ws.tare_weight);
}

void test_if_task_resets_stability_when_not_calibrated(void)
{
    reset_fixture();
    ws.stable_sample_count = CONFIG_RTS_WS_STABLE_SAMPLE_COUNT;
    ws.stable_sample_index = 3;
    ws.is_stable           = true;
    raw_ready              = true;

    rts_ws_task(&ws);

    TEST_ASSERT_FALSE(ws.is_stable);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_count);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_index);
}

void test_if_calibrated_raw_values_convert_at_points_and_between_points(void)
{
    const uint32_t raw_points[4]   = {0, 100, 200, 300};
    const int32_t weight_points[4] = {0, 100, 300, 600};
    const uint8_t point_count =
        CONFIG_RTS_WS_MAX_CALIBRATION_POINTS >= 4 ? 4 : CONFIG_RTS_WS_MAX_CALIBRATION_POINTS;
    rts_ws_calibration_t calibration;

    build_calibration(&calibration, point_count, raw_points, weight_points);

    for(uint8_t i = 0; i < point_count; ++i)
    {
        reset_fixture();
        TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &calibration));
        feed_constant_raw(raw_points[i], CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
        TEST_ASSERT_EQUAL_INT32(weight_points[i], ws.weight);
    }

    for(uint8_t i = 0; i + 1 < point_count; ++i)
    {
        const uint32_t mid_raw   = (raw_points[i] + raw_points[i + 1]) / 2;
        const int32_t mid_weight = (weight_points[i] + weight_points[i + 1]) / 2;

        reset_fixture();
        TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_calibrate(&ws, &calibration));
        feed_constant_raw(mid_raw, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
        TEST_ASSERT_EQUAL_INT32(mid_weight, ws.weight);
    }
}

void test_if_conversion_extrapolates_outside_calibration_range(void)
{
    reset_fixture();
    set_two_point_calibration(100, 0, 200, 100);

    feed_constant_raw(50, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_EQUAL_INT32(-50, ws.weight);

    reset_fixture();
    set_two_point_calibration(100, 0, 200, 100);
    feed_constant_raw(250, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_EQUAL_INT32(150, ws.weight);
}

void test_if_conversion_supports_decreasing_weight_calibration(void)
{
    reset_fixture();
    set_two_point_calibration(100, 1000, 200, 0);

    feed_constant_raw(150, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);

    TEST_ASSERT_EQUAL_INT32(500, ws.weight);
}

void test_if_raw_filter_rejects_short_spikes(void)
{
    const unsigned int spike_count = CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT / 2;
    unsigned int sample_index      = 0;

    reset_fixture();
    set_two_point_calibration(0, 0, 2000, 2000);
    feed_constant_raw(100, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);

    // Spikes shorter than half of the filter window must not reach the output
    for(unsigned int i = 0; i < spike_count; ++i)
    {
        feed_raw(2000);
        TEST_ASSERT_EQUAL_INT32(100, ws.stable_sample[sample_index]);
        sample_index = (sample_index + 1) % CONFIG_RTS_WS_STABLE_SAMPLE_COUNT;
    }

    // One more spike makes them the majority and the median follows
    feed_raw(2000);
    TEST_ASSERT_EQUAL_INT32(2000, ws.stable_sample[sample_index]);
}

void test_if_stability_requires_a_full_window_and_uses_configured_range(void)
{
    const int32_t base              = 40;
    const int32_t delta             = CONFIG_RTS_WS_STABLE_RANGE;
    const unsigned int settle_count = CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT / 2 + 1;

    reset_fixture();
    set_two_point_calibration(0, 0, 100, 100);

    feed_constant_raw((uint32_t)base, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT - 1);
    TEST_ASSERT_FALSE(ws.is_stable);
    TEST_ASSERT_EQUAL_UINT16(CONFIG_RTS_WS_STABLE_SAMPLE_COUNT - 1,
                             ws.stable_sample_count);

    feed_raw((uint32_t)base);
    TEST_ASSERT_TRUE(ws.is_stable);
    TEST_ASSERT_EQUAL_INT32(base, ws.weight);

    // A change within the configured range keeps the weight stable
    feed_constant_raw((uint32_t)(base + delta),
                      settle_count + CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_TRUE(ws.is_stable);

    // A change beyond the configured range breaks stability
    unsigned int fed = 0;
    while(ws.is_stable && (fed < (settle_count + 2 * CONFIG_RTS_WS_STABLE_SAMPLE_COUNT)))
    {
        feed_raw((uint32_t)(base + 2 * delta + 1));
        ++fed;
    }
    TEST_ASSERT_FALSE(ws.is_stable);
}

void test_if_stability_ignores_single_raw_spikes(void)
{
    reset_fixture();
    set_two_point_calibration(0, 0, 100, 100);

    feed_constant_raw(0, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_TRUE(ws.is_stable);

    feed_raw(CONFIG_RTS_WS_STABLE_RANGE + 1);

    TEST_ASSERT_TRUE(ws.is_stable);
    TEST_ASSERT_EQUAL_INT32(0, ws.weight);
}

void test_if_tare_requires_stability_and_zeroes_net_weight(void)
{
    int32_t weight = -1;
    reset_fixture();
    set_two_point_calibration(0, 0, 200, 200);

    TEST_ASSERT_EQUAL(RTS_WS_NO_SAMPLE, rts_ws_set_tare(&ws));
    feed_constant_raw(100, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_TRUE(ws.is_stable);
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_tare(&ws));
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&ws, &weight));
    TEST_ASSERT_EQUAL_INT32(0, weight);
}

void test_if_net_weight_tracks_load_change_after_tare(void)
{
    int32_t weight = 0;
    const int32_t load_step =
        (int32_t)CONFIG_RTS_WS_STABLE_SAMPLE_COUNT * (CONFIG_RTS_WS_STABLE_RANGE + 1);

    reset_fixture();
    set_two_point_calibration(0, 0, 2000, 2000);
    feed_constant_raw(100, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_set_tare(&ws));

    // Two samples of the new load move the average outside the stable range
    feed_constant_raw(100 + load_step,
                      CONFIG_RTS_WS_RAW_FILTER_SAMPLE_COUNT / 2 + 2);
    TEST_ASSERT_FALSE(ws.is_stable);

    feed_constant_raw(100 + load_step, CONFIG_RTS_WS_STABLE_SAMPLE_COUNT);

    TEST_ASSERT_TRUE(ws.is_stable);
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&ws, &weight));
    TEST_ASSERT_EQUAL_INT32(load_step, weight);
}

void test_if_setting_calibration_resets_tare_and_stability(void)
{
    reset_fixture();
    set_two_point_calibration(0, 0, 100, 100);
    ws.tare_weight         = 40;
    ws.is_stable           = true;
    ws.stable_sample_count = CONFIG_RTS_WS_STABLE_SAMPLE_COUNT;

    set_two_point_calibration(0, 0, 200, 200);

    TEST_ASSERT_EQUAL_INT32(0, ws.tare_weight);
    TEST_ASSERT_FALSE(ws.is_stable);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_count);
    TEST_ASSERT_EQUAL_UINT16(0, ws.stable_sample_index);
}

void test_if_setting_calibration_resets_raw_filter_history(void)
{
    reset_fixture();
    set_two_point_calibration(0, 0, 2000, 2000);
    feed_raw(100);

    set_two_point_calibration(0, 0, 2000, 2000);
    feed_raw(900);

    TEST_ASSERT_EQUAL_INT32(900, ws.stable_sample[0]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_if_init_clears_state_and_stores_callbacks);
    RUN_TEST(test_if_raw_read_returns_no_sample_when_data_is_not_ready);
    RUN_TEST(test_if_raw_read_returns_ready_sample);
    RUN_TEST(test_if_task_leaves_state_unchanged_without_a_ready_sample);
    RUN_TEST(test_if_calibration_accepts_two_and_maximum_point_counts);
    RUN_TEST(test_if_calibration_rejects_invalid_point_counts);
    RUN_TEST(test_if_calibration_rejects_non_increasing_raw_values);
    RUN_TEST(test_if_calibration_rejects_non_monotonic_weight_values);
    RUN_TEST(test_if_calibration_rejects_out_of_range_conversion_factor);
    RUN_TEST(test_if_invalid_calibration_does_not_replace_previous_calibration);
    RUN_TEST(test_if_task_resets_stability_when_not_calibrated);
    RUN_TEST(test_if_calibrated_raw_values_convert_at_points_and_between_points);
    RUN_TEST(test_if_conversion_extrapolates_outside_calibration_range);
    RUN_TEST(test_if_conversion_supports_decreasing_weight_calibration);
    RUN_TEST(test_if_raw_filter_rejects_short_spikes);
    RUN_TEST(test_if_stability_requires_a_full_window_and_uses_configured_range);
    RUN_TEST(test_if_stability_ignores_single_raw_spikes);
    RUN_TEST(test_if_tare_requires_stability_and_zeroes_net_weight);
    RUN_TEST(test_if_net_weight_tracks_load_change_after_tare);
    RUN_TEST(test_if_setting_calibration_resets_tare_and_stability);
    RUN_TEST(test_if_setting_calibration_resets_raw_filter_history);
    return UNITY_END();
}
