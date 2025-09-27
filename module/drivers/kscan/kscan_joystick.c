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

/* M_PI constant */
#ifndef MPIf
#define M_PIf 3.1415927f
#endif

/* ADC Configuration */
#define ADC_RESOLUTION  12
#define ADC_GAIN        ADC_GAIN_1_6
#define ADC_REFERENCE   ADC_REF_INTERNAL
#define ADC_ACQUISITION ADC_ACQ_TIME_DEFAULT
#define BUFFER_SIZE 2

struct kscan_joystick_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    
    // ADC related members
    int16_t adc_buffer[BUFFER_SIZE];
    struct adc_sequence adc_sequence;
};

struct kscan_joystick_config {
    struct adc_dt_spec adc_0;
    struct adc_dt_spec adc_1;
    int32_t idle_period_ms;
    int32_t poll_period_ms;
    int16_t angle_offset;
};

static void kscan_joystick_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_joystick_data *data = CONTAINER_OF(dwork, struct kscan_joystick_data, work);
    const struct device *dev = data->dev;
    const struct kscan_joystick_config *config = dev->config;
    
    int ret;

    // ADC device pointer from config (both channels belong to this device)
    const struct device *adc_dev = config->adc_0.dev;
    
    // Read ADC values from both channels
    ret = adc_read(adc_dev, &data->adc_sequence);
    if (ret) {
        LOG_ERR("ADC read failed with error %d", ret);
        goto schedule_next;
    }
    
    // Get the ADC values
    float x = (float)data->adc_buffer[0];
    float y = (float)data->adc_buffer[1];

    LOG_DBG("ADC CH0: %d, ADC CH1: %d", data->adc_buffer[0], data->adc_buffer[1]);
    
    // Calculate angle and offset
    float angle_rad = atan2f(y, x);
    float angle_deg = angle_rad * (180.0f / M_PIf);
    angle_deg = (angle_deg < 0.0f) ? (angle_deg + 360.0f) : angle_deg;
    angle_deg = fmodf(angle_deg + config->angle_offset, 360.0f);

    // Calculate magnitude
    float magnitude = sqrtf(x * x + y * y);

    LOG_DBG("ANGLE: %.2f, MAGNITUDE: %.2f", angle_deg, magnitude);

    // TODO: Process ADC values to determine key states
    // Example logic:
    // - Compare ADC values against thresholds
    // - Determine if keys are pressed/released
    // - Call data->callback(dev, row, col, pressed) for changes
    
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

    LOG_DBG("Joystick kscan enabled - starting ADC polling");
    
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
    int err = 0;

    data->dev = dev;

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
        // TODO: Implement suspend logic (save power)
        LOG_DBG("Joystick kscan suspended");
        return kscan_joystick_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        // TODO: Implement resume logic (restore from power save)
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
    static struct kscan_joystick_data kscan_joystick_data_##n = {};                                 \
                                                                                                    \
    static const struct kscan_joystick_config kscan_joystick_config_##n = {                         \
        .idle_period_ms = DT_INST_PROP_OR(n, idle_period_ms, 100),                                  \
        .poll_period_ms = DT_INST_PROP_OR(n, poll_period_ms, 10),                                   \
        .adc_0 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 0),                                                 \
        .adc_1 = ADC_DT_SPEC_INST_GET_BY_IDX(n, 1),                                                 \
        .angle_offset  = DT_INST_PROP_OR(n, angle_offset, 0),                                       \
    };                                                                                              \
                                                                                                    \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_joystick_pm_action);                                          \
                                                                                                    \
    DEVICE_DT_INST_DEFINE(n, &kscan_joystick_init, PM_DEVICE_DT_INST_GET(n),                        \
                          &kscan_joystick_data_##n, &kscan_joystick_config_##n, POST_KERNEL,        \
                          CONFIG_KSCAN_INIT_PRIORITY, &kscan_joystick_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_JOYSTICK_INIT);

/*
 * IMPLEMENTATION GUIDE:
 * 
 * ADC-based Key Scanning:
 * This template now includes ADC initialization and continuous polling of two ADC channels.
 * 
 * Current Implementation:
 * - Initializes ADC device and configures two channels
 * - Continuously polls both ADC channels at configurable intervals
 * - Logs ADC values for debugging
 * 
 * Next Steps for Key Detection:
 * 1. In kscan_joystick_work_handler(), add logic to:
 *    - Compare ADC values against thresholds to determine key states
 *    - Track previous states to detect changes
 *    - Call data->callback(dev, row, col, pressed) for state changes
 * 
 * 2. Device Tree Example:
 *    kscan0: kscan {
 *        compatible = "zmk,kscan-joystick";
 *        poll-period-ms = <20>;
 *        io-channels = <&adc1 0>, <&adc1 1>;
 *    };
 * 
 * 3. Make sure to enable ADC in your Kconfig:
 *    CONFIG_ADC=y
 * 
 * 4. Example Key Detection Logic (add to work_handler):
 *    // Define thresholds
 *    #define KEY_PRESS_THRESHOLD 512
 *    
 *    static bool prev_key0_state = false;
 *    static bool prev_key1_state = false;
 *    
 *    bool key0_pressed = (x > KEY_PRESS_THRESHOLD);
 *    bool key1_pressed = (y > KEY_PRESS_THRESHOLD);
 *    
 *    if (key0_pressed != prev_key0_state) {
 *        data->callback(dev, 0, 0, key0_pressed);
 *        prev_key0_state = key0_pressed;
 *    }
 *    
 *    if (key1_pressed != prev_key1_state) {
 *        data->callback(dev, 0, 1, key1_pressed);
 *        prev_key1_state = key1_pressed;
 *    }
 */