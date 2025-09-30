/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
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
#define KSCAN_JC_SCALED 127
#define KSCAN_JC_DEFAULT 1000
#define KSCAN_JC_SHRINK(n) (((n) * 3) / 4)
/* Simple moving average constants */
#define KSCAN_JS_NSAMPLES 8
/* Angle filter constants */
#define KSCAN_JA_HYSTERIS 1.0f
/* Idle timeout threshold */
#define KSCAN_JI_THRESHOLD 3
#define KSCAN_JI_NSCANS 1000

struct kscan_joystick_calibration {
    bool is_active;
    uint8_t counter;
    uint16_t samples[KSCAN_JC_NSAMPLES][2];
    int16_t center[2];
    int16_t neg_disp[2];
    int16_t pos_disp[2];
};

struct kscan_joystick_sma {
    uint8_t counter;
    int16_t buffer[KSCAN_JS_NSAMPLES][2];
};

struct kscan_joystick_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    
    // structs
    struct adc_sequence adc_sequence;
    struct kscan_joystick_calibration calibration;
    struct kscan_joystick_sma sma;
    // buffers
    int16_t adc_buffer[2];
    uint16_t key_state[8];
    // smaller variables
    float prev_angle;
    uint16_t idle_timeout;
    uint8_t threshold_state;
};

struct kscan_joystick_config {
    struct adc_dt_spec adc_0;
    struct adc_dt_spec adc_1;
    int32_t idle_period_ms;
    int32_t poll_period_ms;
    int16_t angle_offset;
    int16_t angle_overlap;
    uint8_t n_directions;
    uint8_t hysteris;
    uint8_t thresholds_len;
    uint8_t thresholds[8];
};

static bool kscan_joystick_calibration_handler(struct kscan_joystick_calibration *calibration, int16_t x, int16_t y) {
    // Update maximum displacement from the center
    if (x > calibration->center[0]) {
        // x is greater than the center, update displacement in the positive direction
        calibration->pos_disp[0] = MAX(calibration->pos_disp[0], x - calibration->center[0]);
    } else {
        // x is less than or equal to the center, update displacement in the negative direction
        calibration->neg_disp[0] = MAX(calibration->neg_disp[0], calibration->center[0] - x);
    }
    if (y > calibration->center[1]) {
        // y is greater than the center, update displacement in the positive direction
        calibration->pos_disp[1] = MAX(calibration->pos_disp[1], y - calibration->center[1]);
    } else {
        // y is less than or equal to the center, displacement in the update negative direction
        calibration->neg_disp[1] = MAX(calibration->neg_disp[1], calibration->center[1] - y);
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
            // Shrink the neg_disp & pos_disp values
            calibration->neg_disp[0] = KSCAN_JC_SHRINK(calibration->neg_disp[0]);
            calibration->pos_disp[0] = KSCAN_JC_SHRINK(calibration->pos_disp[0]);
            calibration->neg_disp[1] = KSCAN_JC_SHRINK(calibration->neg_disp[1]);
            calibration->pos_disp[1] = KSCAN_JC_SHRINK(calibration->pos_disp[1]);
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

static void kscan_joystick_apply_sma(struct kscan_joystick_sma *sma, int16_t *x, int16_t *y) {
    // Add the values to the buffer
    sma->buffer[sma->counter][0] = *x;
    sma->buffer[sma->counter][1] = *y;
    // Increment the counter
    sma->counter = (sma->counter + 1) % KSCAN_JS_NSAMPLES;
    // Average the values in the buffer
    int32_t sum_x = 0, sum_y = 0;
    for (uint8_t i = 0; i < KSCAN_JS_NSAMPLES; i++) {
        sum_x += sma->buffer[i][0];
        sum_y += sma->buffer[i][1];
    }
    // Write the averaged value
    *x = (int16_t)(sum_x / KSCAN_JS_NSAMPLES);
    *y = (int16_t)(sum_y / KSCAN_JS_NSAMPLES);
}

static void kscan_joystick_apply_angle_hysteris(float *angle, float *prev) {
    float diff = *angle - *prev;

    // Normalise diff, assuming angle is [-180, 180]
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }

    // Update prev angle to the center of the hysteris
    if (diff > KSCAN_JA_HYSTERIS) {
        *prev = *angle - KSCAN_JA_HYSTERIS;
    } else if (diff < -KSCAN_JA_HYSTERIS) {
        *prev = *angle + KSCAN_JA_HYSTERIS;
    }

    // Since prev was updated, we can set angle to prev
    *angle = *prev;
    return;
}

static void kscan_joystick_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_joystick_data *data = CONTAINER_OF(dwork, struct kscan_joystick_data, work);
    const struct device *dev = data->dev;
    const struct kscan_joystick_config *config = dev->config;
    struct kscan_joystick_calibration *calibration = &data->calibration;
    struct kscan_joystick_sma *sma = &data->sma;
    
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

    // Update the calibration
    if (kscan_joystick_calibration_handler(calibration, x_raw, y_raw)) {

        // Apply a simple moving average
        kscan_joystick_apply_sma(sma, &x_raw, &y_raw);

        // LOG_DBG("RAW ADC CH0: %d, CH1: %d", x_raw, y_raw);

        // Offset the ADC values
        int16_t x = x_raw - calibration->center[0];
        int16_t y = y_raw - calibration->center[1];
        // Scale the ADC values between -127 and 127
        x = (x > 0)?
            ((x * KSCAN_JC_SCALED) / calibration->pos_disp[0]):
            ((x * KSCAN_JC_SCALED) / calibration->neg_disp[0]);
        y = (y > 0)?
            ((y * KSCAN_JC_SCALED) / calibration->pos_disp[1]):
            ((y * KSCAN_JC_SCALED) / calibration->neg_disp[1]);

        // LOG_DBG("CAL ADC CH0: %d, CH1: %d", x, y);

        // Calculate angle
        float angle_rad = atan2f((float)y, (float)x);
        float angle_deg = angle_rad * (180.0f / M_PIf);
        // Apply angle hysteris
        kscan_joystick_apply_angle_hysteris(&angle_deg, &data->prev_angle);
        // Offset the angle
        angle_deg += (float)(config->angle_offset);
        // Wrap to [0, 360) degrees
        while (angle_deg < 0.0f) angle_deg += 360.0f;
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
                        // LOG_DBG("STATE: %d, ROW: %d, COL: %d", data->threshold_state, row, col);
                        if (!IS_BIT_SET(data->key_state[row], col)) {
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

        // Check whether it is in the center
        if (magnitude < KSCAN_JI_THRESHOLD) {
            data->idle_timeout++;
        } else {
            data->idle_timeout = 0;
        }
    }

    // Schedule next scan at a longer interval
    if (data->idle_timeout > KSCAN_JI_NSCANS) {
        LOG_DBG("Joystick kscan - switching to low polling rate");
        k_work_reschedule(&data->work, K_MSEC(config->idle_period_ms));
        return;
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

    // Cancel any scheduled work
    k_work_cancel_delayable(&data->work);
    
    LOG_DBG("Joystick kscan disabled - implement your logic here");
    return 0;
}

static int kscan_joystick_init(const struct device *dev) {
    struct kscan_joystick_data *data = dev->data;
    const struct kscan_joystick_config *config = dev->config;
    struct kscan_joystick_calibration *calibration = &data->calibration;
    struct kscan_joystick_sma *sma = &data->sma;
    int err = 0;

    data->dev = dev;

    // Initialise key & threshold states
    data->threshold_state = 0;
    memset(data->key_state, 0, sizeof(data->key_state));
    // Initialise idle timeout
    data->idle_timeout = 0;
    // Initialise smoothing filters
    data->prev_angle = 0;
    sma->counter = 0;
    memset(sma->buffer, 0, sizeof(sma->buffer));
    // Initialise calibration variables
    calibration->center[0] = 2047;
    calibration->center[1] = 2047;
    calibration->neg_disp[0] = KSCAN_JC_DEFAULT;
    calibration->pos_disp[0] = KSCAN_JC_DEFAULT;
    calibration->neg_disp[1] = KSCAN_JC_DEFAULT;
    calibration->pos_disp[1] = KSCAN_JC_DEFAULT;
    // Trigger a calibration
    calibration->counter = 0;
    calibration->is_active = true;
    memset(calibration->samples, 0, sizeof(calibration->samples));

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
    BUILD_ASSERT(DT_INST_PROP_LEN(n, thresholds) > 0,                                               \
                 "thresholds must have between 1 - 8 members");                                     \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, thresholds) <= 8,                                              \
                 "thresholds must have between 1 - 8 members");                                     \
    BUILD_ASSERT(DT_INST_PROP_OR(n, hysteris, 5) < DT_INST_PROP_BY_IDX(n, thresholds, 0),           \
                 "hysteris must be less than the first threshold value");                           \
                                                                                                    \
    static struct kscan_joystick_data kscan_joystick_data_##n = {};                                 \
                                                                                                    \
    static const struct kscan_joystick_config kscan_joystick_config_##n = {                         \
        .poll_period_ms = DT_INST_PROP_OR(n, poll_period_ms, 10),                                   \
        .idle_period_ms = DT_INST_PROP_OR(n, idle_period_ms, 100),                                  \
        .adc_0 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 0),                                                 \
        .adc_1 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 1),                                                 \
        .angle_offset   = DT_INST_PROP_OR(n, angle_offset, 0),                                      \
        .angle_overlap  = DT_INST_PROP_OR(n, angle_overlap, 0),                                     \
        .n_directions   = DT_INST_PROP_OR(n, n_directions, 4),                                      \
        .hysteris       = DT_INST_PROP_OR(n, hysteris, 5),                                          \
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
