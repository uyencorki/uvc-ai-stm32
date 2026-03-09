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
#define SENSOR_DVP_WIDTH 1280
#define SENSOR_DVP_HEIGHT 720
#define SENSOR_DVP_FLIP CMW_MIRRORFLIP_NONE

/* Define VENC info for DVP sensor */
#define VENC_DVP_WIDTH 1280
#define VENC_DVP_HEIGHT 720

/* Delay display by CAPTURE_DELAY frame number */
#define CAPTURE_DELAY 1

#endif
