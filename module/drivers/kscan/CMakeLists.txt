# Copyright (c) 2020-2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_JOYSTICK kscan_joystick.c)
