 /**
 ******************************************************************************
 * @file    enc.h
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
#ifndef ENC
#define ENC

#include <stddef.h>
#include <stdint.h>

#ifndef APP_ENC_USE_H264
#define APP_ENC_USE_H264 0
#endif

typedef enum {
  ENC_INPUT_RGB888 = 0,
  ENC_INPUT_YUV422_YUYV,
  ENC_INPUT_YUV422_UYVY,
} ENC_InputType_t;

typedef struct {
  int width;
  int height;
  int fps;
  ENC_InputType_t input_type;
} ENC_Conf_t;

void ENC_Init(ENC_Conf_t *p_conf);
void ENC_DeInit(void);
int ENC_EncodeFrame(uint8_t *p_in, uint8_t *p_out, size_t out_len, int is_intra_force);

#endif
