/**
 ******************************************************************************
 * @file    app_config.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG

#include "postprocess_conf.h"

#define USE_DCACHE

/* Define DVP sensor info (PX9210K) */
#define SENSOR_DVP_WIDTH 1920
#define SENSOR_DVP_HEIGHT 1080
#define SENSOR_DVP_FLIP CMW_MIRRORFLIP_NONE

/* Keep VENC at 720p during bring-up to fit PSRAM and reduce DCMIPP load. */
#define VENC_DVP_WIDTH 1280
#define VENC_DVP_HEIGHT 720

/* Delay display by CAPTURE_DELAY frame number */
#define CAPTURE_DELAY 1

/* Bring-up mode for new DVP sensors: run only PIPE1 first, then re-enable NN/PIPE2 later. */
#define APP_DVP_BRINGUP_PIPE1_ONLY 0

/* DVP timing test selector (1..8), then rebuild and retest.
 * bit2: VSYNC polarity (0=active low, 1=active high)
 * bit1: HSYNC polarity (0=active low, 1=active high)
 * bit0: PCLK edge      (0=falling,    1=rising)
 * Example: case=2 -> 001b => VSYNC low, HSYNC low, PCLK rising.
 */
#define APP_DVP_TEST_CASE 3

#define APP_DVP_VSYNC_ACTIVE_HIGH   (((APP_DVP_TEST_CASE) - 1) >> 2 & 0x1)
#define APP_DVP_HSYNC_ACTIVE_HIGH   (((APP_DVP_TEST_CASE) - 1) >> 1 & 0x1)
#define APP_DVP_PIXCLK_RISING_EDGE  (((APP_DVP_TEST_CASE) - 1) >> 0 & 0x1)

/* PX9210K I2C debug dump. */
#define APP_PX9210_I2C_DUMP_ENABLE 0
/* 8-bit I2C address: 0x34 write / 0x35 read (7-bit address = 0x1A). Set 0 to auto-scan. */
#define APP_PX9210_I2C_ADDR_8BIT 0x34U
/* Kept for compatibility with old code path; not used in essential 32-bit read mode. */
#define APP_PX9210_REG_DUMP_START 0x0000U
#define APP_PX9210_REG_DUMP_END 0xFFFFU

/* Trial read for Group A (datasheet lists A:offset, but BASE_A must be validated). */
#define APP_PX9210_READ_GROUP_A_TRY 0
#define APP_PX9210_GROUP_A_BASE 0x00000000U

#endif
