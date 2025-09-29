/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_joystick

/* Macros */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif // MIN
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif // MAX
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif // BIT
#ifndef IS_BIT_SET
#define IS_BIT_SET(value, bit) ((((value) >> (bit)) & (0x1)) != 0)
#endif // IS_BIT_SET
#ifndef WRITE_BIT
#define WRITE_BIT(var, bit, set) ((var) = (set) ? ((var) | BIT(bit)) : ((var) & ~BIT(bit)))
#endif // WRITE_BIT

/* M_PI constant */
#ifndef M_PIf
#define M_PIf 3.1415927f
#endif

/* Calibration constants */
#define KSCAN_JC_NSAMPLES 32
#define KSCAN_JC_SHRINK(n) (((n) * 3) / 4)
#define KSCAN_JC_SCALED 127
#define KSCAN_JC_DEFAULT 1000

struct kscan_joystick_calibration {
    bool is_active;
    uint8_t counter;
    uint16_t samples[KSCAN_JC_NSAMPLES][2];
    int16_t center[2];
    int16_t min[2];
    int16_t max[2];
};

struct kscan_joystick_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    
    // ADC related members
    int16_t adc_buffer[2];
    struct adc_sequence adc_sequence;
    struct kscan_joystick_calibration calibration;

    // KSCAN related members
    uint16_t key_state[8];
    uint8_t threshold_state;
    uint8_t idle_timeout;
};

struct kscan_joystick_config {
    struct adc_dt_spec adc_0;
    struct adc_dt_spec adc_1;
    int32_t idle_period_ms;
    int32_t poll_period_ms;
    float angle_offset;
    int16_t angle_overlap;
    uint8_t n_directions;
    uint8_t hysteris;
    uint8_t thresholds_len;
    uint8_t thresholds[8];
};

static bool kscan_joystick_calibration_handler(struct kscan_joystick_calibration *calibration, int16_t x, int16_t y) {
    // Update min and max
    if (x > calibration->center[0]) {
        // x is greater than the center, update displacement in the positive direction
        calibration->max[0] = MAX(calibration->max[0], x - calibration->center[0]);
    } else {
        // x is less than or equal to the center, update displacement in the negative direction
        calibration->min[0] = MAX(calibration->min[0], calibration->center[0] - x);
    }
    if (y > calibration->center[1]) {
        // y is greater than the center, update displacement in the positive direction
        calibration->max[1] = MAX(calibration->max[1], y - calibration->center[1]);
    } else {
        // y is less than or equal to the center, displacement in the update negative direction
        calibration->min[1] = MAX(calibration->min[1], calibration->center[1] - y);
    }
    // Update center
    if (calibration->is_active) {
        // Write the analog values to the buffer
        calibration->samples[calibration->counter][0] = x;
        calibration->samples[calibration->counter][1] = y;
        // Check whether enough samples have been collected
        if (++calibration->counter >= KSCAN_JC_NSAMPLES) {
            // Sum the samples
            int32_t sum_x = 0, sum_y = 0;
            for (uint8_t i = 0; i < KSCAN_JC_NSAMPLES; i++){
                sum_x += calibration->samples[i][0];
                sum_y += calibration->samples[i][1];
            }
            // Write the new calibration
            calibration->center[0] = (int16_t)(sum_x / KSCAN_JC_NSAMPLES);
            calibration->center[1] = (int16_t)(sum_y / KSCAN_JC_NSAMPLES);
            // Shrink the min & max values
            calibration->min[0] = KSCAN_JC_SHRINK(calibration->min[0]);
            calibration->max[0] = KSCAN_JC_SHRINK(calibration->max[0]);
            calibration->min[1] = KSCAN_JC_SHRINK(calibration->min[1]);
            calibration->max[1] = KSCAN_JC_SHRINK(calibration->max[1]);
            // End the calibration
            calibration->counter = 0;
            calibration->is_active = false;
            return true;
        }
        // Do not process joystick
        return false;
    }

    return true;
}

static void kscan_joystick_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_joystick_data *data = CONTAINER_OF(dwork, struct kscan_joystick_data, work);
    const struct device *dev = data->dev;
    const struct kscan_joystick_config *config = dev->config;
    struct kscan_joystick_calibration *calibration = &data->calibration;
    
    int ret;

    // ADC device pointer from config (both channels belong to this device)
    const struct device *adc_dev = config->adc_0.dev;
    
    // Read ADC values from both channels
    ret = adc_read(adc_dev, &data->adc_sequence);
    if (ret) {
        LOG_ERR("ADC read failed with error %d", ret);
        goto schedule_next;
    }

    int16_t x_raw = data->adc_buffer[0];
    int16_t y_raw = data->adc_buffer[1];

    // LOG_DBG("RAW ADC CH0: %d, CH1: %d", x_raw, y_raw);

    // Update the calibration
    if (kscan_joystick_calibration_handler(calibration, x_raw, y_raw)) {

        // Offset and scale the ADC values
        int16_t x = x_raw - calibration->center[0];
        int16_t y = y_raw - calibration->center[1];
        x = (x > calibration->center[0]) ? ((x * KSCAN_JC_SCALED) / calibration->max[0]) : ((x * KSCAN_JC_SCALED) / calibration->min[0]);
        y = (y > calibration->center[1]) ? ((y * KSCAN_JC_SCALED) / calibration->max[1]) : ((y * KSCAN_JC_SCALED) / calibration->min[1]);

        // LOG_DBG("CAL ADC CH0: %d, CH1: %d", x, y);

        // Calculate angle
        float angle_rad = atan2f((float)y, (float)x);
        float angle_deg = angle_rad * (180.0f / M_PIf);
        // angle_deg = (angle_deg < 0.0f) ? (angle_deg + 360.0f) : angle_deg;
        angle_deg = 360.0f + angle_deg + config->angle_offset;
        angle_deg = fmodf(angle_deg, 360.0f);

        // Calculate magnitude
        float magnitude = sqrtf((float)(x) * (float)(x) + (float)(y) * (float)(y));

        // LOG_DBG("ANGLE: %d, MAGNITUDE: %d", (int16_t)angle_deg, (int16_t)magnitude);

        // Iterate backwards through the thresholds, decrementing for each threshold not crossed
        for (uint8_t i = data->threshold_state; i > 0; i--) {
            if (
                magnitude < (config->thresholds[i - 1] - config->hysteris)
            ) {
                data->threshold_state--;
            }
        }
        // Check whether the next threshold has been crossed
        if (
            (data->threshold_state < config->thresholds_len) &&
            (magnitude > config->thresholds[data->threshold_state])
        ) {
            data->threshold_state++;
        }
        
        // Determine which directions it is facing
        bool col_0_active = false;
        float segment = (float)(360 / config->n_directions);
        for (uint8_t i = 0; i <= config->n_directions; i++) {
            uint8_t col = i % config->n_directions;
            if (
                (angle_deg >  ((float)(i - 0.5) * segment) - (float)(config->angle_overlap / 2)) &&
                (angle_deg <= ((float)(i + 0.5) * segment) + (float)(config->angle_overlap / 2))
            ) {
                // Account for col 0 being read twice
                if (col == 0) {
                    col_0_active = true;
                }
                for (uint8_t row = 0; row < config->thresholds_len; row++) {
                    if (row < data->threshold_state) {
                        if (!IS_BIT_SET(data->key_state[row], col)) {
                            // LOG_DBG("STATE: %d, A: %d, M: %d, ROW: %d, COL: %d", data->threshold_state, (int16_t)angle_deg, (int16_t)magnitude, row, col);
                            data->callback(dev, row, col, true);
                            WRITE_BIT(data->key_state[row], col, 1);
                        }
                    } else {
                        if (IS_BIT_SET(data->key_state[row], col)) {
                            data->callback(dev, row, col, false);
                            WRITE_BIT(data->key_state[row], col, 0);
                        }
                    }
                }
            }
            else {
                for (uint8_t row = 0; row < config->thresholds_len; row++) {
                    if (
                        (IS_BIT_SET(data->key_state[row], col)) &&
                        (!(col == 0 && col_0_active))
                    ) {
                        data->callback(dev, row, col, false);
                        WRITE_BIT(data->key_state[row], col, 0);
                    }
                }
            }
        }
    }
    
schedule_next:
    // Schedule next scan
    k_work_reschedule(&data->work, K_MSEC(config->poll_period_ms));
}

static int kscan_joystick_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_joystick_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    return 0;
}

static int kscan_joystick_enable(const struct device *dev) {
    struct kscan_joystick_data *data = dev->data;
    const struct kscan_joystick_config *config = dev->config;
    struct kscan_joystick_calibration *calibration = &data->calibration;

    LOG_DBG("Joystick kscan enabled - starting ADC polling");

    // Trigger a calibration
    calibration->counter = 0;
    calibration->is_active = true;
    
    // Start periodic ADC scanning
    k_work_reschedule(&data->work, K_MSEC(config->poll_period_ms));
    
    return 0;
}

static int kscan_joystick_disable(const struct device *dev) {
    struct kscan_joystick_data *data = dev->data;

    // TODO: Implement your disable logic here
    // This is called when scanning should stop (e.g., for power saving)
    
    // Cancel any scheduled work
    k_work_cancel_delayable(&data->work);
    
    LOG_DBG("Joystick kscan disabled - implement your logic here");
    return 0;
}

static int kscan_joystick_init(const struct device *dev) {
    struct kscan_joystick_data *data = dev->data;
    const struct kscan_joystick_config *config = dev->config;
    struct kscan_joystick_calibration *calibration = &data->calibration;
    int err = 0;

    data->dev = dev;

    // Set key state to zero
    memset(data->key_state, 0, sizeof(data->key_state));
    data->threshold_state = 0;
    data->idle_timeout = 0;

    // Trigger a calibration
    calibration->center[0] = 2047;
    calibration->center[1] = 2047;
    calibration->min[0] = KSCAN_JC_DEFAULT;
    calibration->max[0] = KSCAN_JC_DEFAULT;
    calibration->min[1] = KSCAN_JC_DEFAULT;
    calibration->max[1] = KSCAN_JC_DEFAULT;
    memset(calibration->samples, 0, sizeof(calibration->samples));
    calibration->counter = 0;
    calibration->is_active = true;

    // Check both channels belong to same device for sequence scan
    __ASSERT(config->adc_0.dev == config->adc_1.dev,
        "Both ADC channels must belong to the same device.");

    data->adc_sequence.buffer = data->adc_buffer;
    data->adc_sequence.buffer_size = sizeof(data->adc_buffer);

    adc_sequence_init_dt(&config->adc_0, &data->adc_sequence);
    data->adc_sequence.channels |= BIT(config->adc_1.channel_id);

    if (!adc_is_ready_dt(&config->adc_0)) {
        LOG_ERR("ADC device is not ready");
        return -EINVAL;
    }

    err = adc_channel_setup_dt(&config->adc_0);
    if (err) {
        LOG_ERR("failed to configure ADC channel 0 (err %d)",
            err);
        return err;
    }
    err = adc_channel_setup_dt(&config->adc_1);
    if (err) {
        LOG_ERR("failed to configure ADC channel 1 (err %d)",
            err);
        return err;
    }

    LOG_INF("ADC-based kscan initialized successfully");

    // Initialize the work queue
    k_work_init_delayable(&data->work, kscan_joystick_work_handler);

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);
#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif
#endif

    return err;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int kscan_joystick_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_DBG("Joystick kscan suspended");
        return kscan_joystick_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        LOG_DBG("Joystick kscan resumed");
        return kscan_joystick_enable(dev);
    default:
        return -ENOTSUP;
    }
}
#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static const struct kscan_driver_api kscan_joystick_api = {
    .config = kscan_joystick_configure,
    .enable_callback = kscan_joystick_enable,
    .disable_callback = kscan_joystick_disable,
};

#define KSCAN_JOYSTICK_INIT(n)                                                                      \
                                                                                                    \
    BUILD_ASSERT(DT_INST_PROP_OR(n, poll_period_ms, 10) > 0,                                        \
                 "poll-period-ms is less than or equal to zero");                                   \
    BUILD_ASSERT(DT_INST_PROP_OR(n, idle_period_ms, 100) > 0,                                       \
                 "idle-period-ms is less than or equal to zero");                                   \
    BUILD_ASSERT(DT_INST_PROP_OR(n, angle_offset, 0) >= 0,                                          \
                 "angle-offset is negative");                                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, n_directions, 4) > 0,                                           \
                 "n-directions is less than or equal to zero");                                     \
    BUILD_ASSERT(DT_INST_PROP_OR(n, n_directions, 4) <= 16,                                         \
                 "n-directions is greater than 16");                                                \
    BUILD_ASSERT(DT_INST_PROP_LEN_OR(n, thresholds, 2) <= 8,                                        \
                 "thresholds can have a maximum of 8 members");                                     \
                                                                                                    \
    static struct kscan_joystick_data kscan_joystick_data_##n = {};                                 \
                                                                                                    \
    static const struct kscan_joystick_config kscan_joystick_config_##n = {                         \
        .poll_period_ms = DT_INST_PROP_OR(n, poll_period_ms, 10),                                   \
        .idle_period_ms = DT_INST_PROP_OR(n, idle_period_ms, 100),                                  \
        .adc_0 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 0),                                                 \
        .adc_1 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 1),                                                 \
        .angle_offset   = (float)DT_INST_PROP_OR(n, angle_offset, 0),                               \
        .angle_overlap  = (float)DT_INST_PROP_OR(n, angle_overlap, 0),                              \
        .n_directions   = DT_INST_PROP_OR(n, n_directions, 4),                                      \
        .hysteris       = DT_INST_PROP_OR(n, hysteris, 10),                                         \
        .thresholds_len = DT_INST_PROP_LEN_OR(n, thresholds, 2),                                    \
        .thresholds     = DT_INST_PROP(n, thresholds),                                              \
    };                                                                                              \
                                                                                                    \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_joystick_pm_action);                                          \
                                                                                                    \
    DEVICE_DT_INST_DEFINE(n, &kscan_joystick_init, PM_DEVICE_DT_INST_GET(n),                        \
                          &kscan_joystick_data_##n, &kscan_joystick_config_##n, POST_KERNEL,        \
                          CONFIG_KSCAN_INIT_PRIORITY, &kscan_joystick_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_JOYSTICK_INIT);
