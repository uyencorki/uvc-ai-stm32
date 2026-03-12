 /**
 ******************************************************************************
 * @file    app.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef APP_H
#define APP_H

#include "app_config.h"
#include "stai_network.h"

#define VENC_WIDTH CAM_GetVencWidth()
#define VENC_HEIGHT CAM_GetVencHeight()
#if APP_DVP_BRINGUP_PIPE1_ONLY
/* In bring-up mode, keep PIPE1 in YUV422 so we can inspect raw-like camera bytes. */
#define CAPTURE_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1
#define CAPTURE_BPP 2
#else
#define CAPTURE_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_ARGB8888
#define CAPTURE_BPP 4
#endif

/* Model Related Info */
#define NN_WIDTH STAI_NETWORK_IN_1_WIDTH
#define NN_HEIGHT STAI_NETWORK_IN_1_HEIGHT
#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP 3

void app_run(void);
void BOARD_Pins_Init_DCMIPP(void);

#endif