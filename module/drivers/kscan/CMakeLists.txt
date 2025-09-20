# Copyright (c) 2020-2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DRIVER kscan_gpio.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DIRECT_TWO kscan_gpio_direct_two.c)
