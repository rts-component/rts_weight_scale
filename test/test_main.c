#include "../rts_weight_scale.h"
#include "unity.h"

static bool raw_ready = true;
static uint32_t raw_value;

static bool test_is_raw_ready(void)
{
    return raw_ready;
}

static uint32_t test_read_raw(void)
{
    return raw_ready ? raw_value : 0;
}

static rts_ws_t scale;

void setUp(void)
{
    raw_ready = true;
    raw_value = 0;
    TEST_ASSERT_EQUAL(RTS_WS_OK,
                      rts_ws_init(&scale, test_is_raw_ready, test_read_raw));
}

void tearDown(void)
{
}

static void set_default_calibration(void)
{
    const rts_ws_calibration_t calibration = {
        .point_count = 3,
        .raw = {100, 200, 400},
        .weight = {0, 100, 300},
    };

    TEST_ASSERT_EQUAL(RTS_WS_OK,
                      rts_ws_set_calibrate(&scale, &calibration));
}

static void test_init_and_raw_read(void)
{
    uint32_t raw = 0;

    TEST_ASSERT_EQUAL(RTS_WS_INVALID_ARGUMENT,
                      rts_ws_init(NULL, test_is_raw_ready, test_read_raw));
    TEST_ASSERT_TRUE(rts_ws_raw_is_ready(&scale));

    raw_value = 1234;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_raw_read(&scale, &raw));
    TEST_ASSERT_EQUAL_UINT32(raw_value, raw);

    raw_ready = false;
    TEST_ASSERT_FALSE(rts_ws_raw_is_ready(&scale));
    TEST_ASSERT_EQUAL(RTS_WS_NO_SAMPLE, rts_ws_raw_read(&scale, &raw));
}

static void test_calibration_and_weight_conversion(void)
{
    int32_t weight = 0;

    set_default_calibration();
    raw_value = 150;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL_INT32(50, weight);

    raw_value = 300;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL_INT32(200, weight);

    raw_value = 50;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL_INT32(-50, weight);

    raw_value = 500;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL_INT32(400, weight);
}

static void test_calibration_validation_and_preservation(void)
{
    const rts_ws_calibration_t invalid = {
        .point_count = 3,
        .raw = {100, 100, 400},
        .weight = {0, 100, 300},
    };
    const rts_ws_calibration_t non_monotonic_weight = {
        .point_count = 3,
        .raw = {100, 200, 400},
        .weight = {0, 100, 50},
    };
    int32_t weight = 0;

    set_default_calibration();
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&scale, &invalid));
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_CALIBRATION,
                      rts_ws_set_calibrate(&scale, &non_monotonic_weight));
    TEST_ASSERT_NOT_NULL(rts_ws_get_calibrate(&scale));

    raw_value = 150;
    TEST_ASSERT_EQUAL(RTS_WS_OK, rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL_INT32(50, weight);
}

static void test_stability_window(void)
{
    const uint32_t stable_values[] = {120, 124, 121, 125, 123};

    set_default_calibration();
    TEST_ASSERT_FALSE(rts_ws_is_stable(&scale));
    for (size_t i = 0; i < CONFIG_RTS_WS_STABLE_SAMPLE_COUNT; ++i)
    {
        raw_value = stable_values[i];
        rts_ws_task(&scale);
    }
    TEST_ASSERT_TRUE(rts_ws_is_stable(&scale));

    raw_value = 300;
    rts_ws_task(&scale);
    TEST_ASSERT_FALSE(rts_ws_is_stable(&scale));

    raw_ready = false;
    rts_ws_task(&scale);
    TEST_ASSERT_FALSE(rts_ws_is_stable(&scale));
}

static void test_unconfigured_errors(void)
{
    int32_t weight = 0;

    TEST_ASSERT_EQUAL(RTS_WS_OK,
                      rts_ws_init(&scale, test_is_raw_ready, test_read_raw));
    raw_value = 150;
    TEST_ASSERT_EQUAL(RTS_WS_NOT_CALIBRATED,
                      rts_ws_get_weight(&scale, &weight));
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_ARGUMENT,
                      rts_ws_get_weight(&scale, NULL));
    TEST_ASSERT_EQUAL(RTS_WS_INVALID_ARGUMENT,
                      rts_ws_set_calibrate(&scale, NULL));
    TEST_ASSERT_NULL(rts_ws_get_calibrate(NULL));
    TEST_ASSERT_FALSE(rts_ws_is_stable(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_raw_read);
    RUN_TEST(test_calibration_and_weight_conversion);
    RUN_TEST(test_calibration_validation_and_preservation);
    RUN_TEST(test_stability_window);
    RUN_TEST(test_unconfigured_errors);
    return UNITY_END();
}