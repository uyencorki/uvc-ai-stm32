 /**
 ******************************************************************************
 * @file    app_cam.c
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
#include <assert.h>
#include <stdio.h>
#include "app.h"
#include "cmw_camera.h"
#include "app_cam.h"
#include "app_config.h"
#include "utils.h"
#include "stm32n6570_discovery_bus.h"

static int sensor_width;
static int sensor_height;
static int venc_width;
static int venc_height;
static int sensor_mirror_flip = CMW_MIRRORFLIP_NONE;
void APP_CAM_DebugOnPipeError(uint32_t pipe);

typedef struct
{
  uint32_t index;
  const char *name;
  const char *meaning;
} PX9210_Reg32Desc_t;

typedef struct
{
  uint8_t offset;
  const char *name;
  const char *meaning;
} PX9210_GroupA_OffsetDesc_t;

/* Essential registers documented for PX9210K I2C slave 32-bit index/data access. */
static const PX9210_Reg32Desc_t g_px9210_essential_reg_desc[] = {
  {0x0100FF00U, "I2CM_CMD", "bit7=Start, bit6=Busy, bit0=1:write 0:read"},
  {0x0100FF04U, "I2CM_ADDR3", "Sensor register address byte[31:24]"},
  {0x0100FF08U, "I2CM_ADDR2", "Sensor register address byte[23:16]"},
  {0x0100FF0CU, "I2CM_ADDR1", "Sensor register address byte[15:8]"},
  {0x0100FF10U, "I2CM_ADDR0", "Sensor register address byte[7:0]"},
  {0x0100FF14U, "I2CM_DATA3", "Sensor data byte[31:24]"},
  {0x0100FF18U, "I2CM_DATA2", "Sensor data byte[23:16]"},
  {0x0100FF1CU, "I2CM_DATA1", "Sensor data byte[15:8]"},
  {0x0100FF20U, "I2CM_DATA0", "Sensor data byte[7:0]"},
};

/* Group A offsets extracted from PX9210K CDS (essential subset for bring-up). */
static const PX9210_GroupA_OffsetDesc_t g_px9210_group_a_list[] = {
  {0x05U, "mirror", "mirror/flip control"},
  {0x06U, "framewidth_h", "frame width high"},
  {0x07U, "framewidth_l", "frame width low"},
  {0x08U, "frameheight_h", "frame height high"},
  {0x09U, "frameheight_l", "frame height low"},
  {0x0CU, "windowx1_h", "window x1 high"},
  {0x0DU, "windowx1_l", "window x1 low"},
  {0x0EU, "windowy1_h", "window y1 high"},
  {0x0FU, "windowy1_l", "window y1 low"},
  {0x10U, "windowx2_h", "window x2 high"},
  {0x11U, "windowx2_l", "window x2 low"},
  {0x12U, "windowy2_h", "window y2 high"},
  {0x13U, "windowy2_l", "window y2 low"},
  {0x15U, "i2c_control_1", "I2C control register"},
  {0x20U, "genlock_pad_en", "genlock pad enable"},
  {0x4FU, "pll_control2", "plltg_pd/reg_pll_bypass/pllmp_pd"},
  {0x55U, "plltg_rdiv", "timing PLL r divider"},
  {0x56U, "plltg_dsm_divh_h", "timing PLL divh high"},
  {0x57U, "plltg_dsm_divh_l", "timing PLL divh low"},
  {0x58U, "plltg_dsm_divl_h", "timing PLL divl high"},
  {0x59U, "plltg_dsm_divl_l", "timing PLL divl low"},
  {0x5AU, "plltg_dsm_xstep", "timing PLL xstep"},
  {0x5BU, "plltg_dsm_ystep", "timing PLL ystep"},
  {0x5DU, "pllmp_rdiv", "MIPI PLL r divider"},
  {0x5EU, "pllmp_dsm_divh_h", "MIPI PLL divh high"},
  {0x5FU, "pllmp_dsm_divh_l", "MIPI PLL divh low"},
  {0x60U, "pllmp_dsm_divl_h", "MIPI PLL divl high"},
  {0x61U, "pllmp_dsm_divl_l", "MIPI PLL divl low"},
  {0x65U, "ispclk_div", "ISP clock divider"},
  {0x66U, "mipiclk_div_ddclk_div", "MIPI clock divider / DD clock divider"},
};

static int32_t APP_CAM_I2C1_ReadReg32x32(uint16_t addr8, uint32_t reg_index, uint32_t *value)
{
  uint8_t idx_buf[4];
  uint8_t data_buf[4];
  HAL_StatusTypeDef hret;

  idx_buf[0] = (uint8_t)(reg_index >> 24);
  idx_buf[1] = (uint8_t)(reg_index >> 16);
  idx_buf[2] = (uint8_t)(reg_index >> 8);
  idx_buf[3] = (uint8_t)(reg_index);

  hret = HAL_I2C_Master_Transmit(&hbus_i2c1, addr8, idx_buf, sizeof(idx_buf), 100U);
  if (hret != HAL_OK)
  {
    return BSP_ERROR_BUS_FAILURE;
  }

  hret = HAL_I2C_Master_Receive(&hbus_i2c1, addr8, data_buf, sizeof(data_buf), 100U);
  if (hret != HAL_OK)
  {
    return BSP_ERROR_BUS_FAILURE;
  }

  *value = ((uint32_t)data_buf[0] << 24) |
           ((uint32_t)data_buf[1] << 16) |
           ((uint32_t)data_buf[2] << 8) |
           ((uint32_t)data_buf[3]);

  return BSP_ERROR_NONE;
}

static void APP_CAM_ReadPx9210GroupATry(uint16_t addr8)
{
#if APP_PX9210_READ_GROUP_A_TRY
  uint32_t i;
  uint32_t ok_cnt = 0U;
  uint32_t fail_cnt = 0U;

  printf("[CAM][I2C][GA] trial read Group A with BASE_A=0x%08lX\r\n",
         (unsigned long)APP_PX9210_GROUP_A_BASE);

  for (i = 0; i < (uint32_t)(sizeof(g_px9210_group_a_list) / sizeof(g_px9210_group_a_list[0])); i++)
  {
    const PX9210_GroupA_OffsetDesc_t *d = &g_px9210_group_a_list[i];
    uint32_t reg_index = APP_PX9210_GROUP_A_BASE + (uint32_t)d->offset;
    uint32_t value = 0U;
    int32_t ret;

    ret = APP_CAM_I2C1_ReadReg32x32(addr8, reg_index, &value);
    if (ret == BSP_ERROR_NONE)
    {
      printf("[CAM][GA] A:0x%02X IDX=0x%08lX DATA=0x%08lX | %s | %s\r\n",
             (unsigned int)d->offset,
             (unsigned long)reg_index,
             (unsigned long)value,
             d->name,
             d->meaning);
      ok_cnt++;
    }
    else
    {
      printf("[CAM][GA] A:0x%02X IDX=0x%08lX READ_FAIL=%ld | %s\r\n",
             (unsigned int)d->offset,
             (unsigned long)reg_index,
             (long)ret,
             d->name);
      fail_cnt++;
    }
  }

  printf("[CAM][I2C][GA] trial done ok=%lu fail=%lu\r\n",
         (unsigned long)ok_cnt,
         (unsigned long)fail_cnt);
#else
  (void)addr8;
#endif
}

static uint16_t APP_CAM_DetectSensorAddr8_I2C1(void)
{
  uint16_t addr7;

  printf("[CAM][I2C] scan I2C1 (PH9/PC1) for PX9210K...\r\n");

  for (addr7 = 0x08U; addr7 <= 0x77U; addr7++)
  {
    uint16_t addr8 = (uint16_t)(addr7 << 1);
    if (BSP_I2C1_IsReady(addr8, 2U) == BSP_ERROR_NONE)
    {
      printf("[CAM][I2C] found device addr7=0x%02X addr8=0x%02X\r\n",
             (unsigned int)addr7,
             (unsigned int)addr8);
      return addr8;
    }
  }

  return 0U;
}

static void APP_CAM_ReadPx9210EssentialRegisters(void)
{
#if APP_PX9210_I2C_DUMP_ENABLE
  uint16_t addr8 = APP_PX9210_I2C_ADDR_8BIT;
  uint32_t i;
  uint32_t ok_cnt = 0U;
  uint32_t fail_cnt = 0U;
  int32_t ret;

  printf("[CAM][I2C] init I2C1 for PX9210K (SCL=PH9 SDA=PC1)\r\n");
  ret = BSP_I2C1_Init();
  if (ret != BSP_ERROR_NONE)
  {
    printf("[CAM][I2C] BSP_I2C1_Init failed=%ld\r\n", (long)ret);
    return;
  }

  if (addr8 == 0U)
  {
    addr8 = APP_CAM_DetectSensorAddr8_I2C1();
  }

  if (addr8 == 0U)
  {
    printf("[CAM][I2C] no I2C device found on I2C1\r\n");
    return;
  }

  printf("[CAM][I2C] PX9210K slave addr7=0x%02X addr8(w)=0x%02X addr8(r)=0x%02X\r\n",
         (unsigned int)(addr8 >> 1),
         (unsigned int)addr8,
         (unsigned int)(addr8 | 0x01U));
  printf("[CAM][I2C] read mode: 32-bit index + 32-bit data (essential regs only)\r\n");

  for (i = 0; i < (uint32_t)(sizeof(g_px9210_essential_reg_desc) / sizeof(g_px9210_essential_reg_desc[0])); i++)
  {
    uint32_t value = 0U;
    const PX9210_Reg32Desc_t *desc = &g_px9210_essential_reg_desc[i];

    ret = APP_CAM_I2C1_ReadReg32x32(addr8, desc->index, &value);
    if (ret == BSP_ERROR_NONE)
    {
      printf("[CAM][REG32] IDX=0x%08lX DATA=0x%08lX | %s | %s\r\n",
             (unsigned long)desc->index,
             (unsigned long)value,
             desc->name,
             desc->meaning);
      ok_cnt++;
    }
    else
    {
      printf("[CAM][REG32] IDX=0x%08lX READ_FAIL=%ld | %s\r\n",
             (unsigned long)desc->index,
             (long)ret,
             desc->name);
      fail_cnt++;
    }
  }

  printf("[CAM][I2C] essential read done ok=%lu fail=%lu\r\n",
         (unsigned long)ok_cnt,
         (unsigned long)fail_cnt);

  APP_CAM_ReadPx9210GroupATry(addr8);
#endif
}

static const char *sensor_names[] = {
  "CMW_UNKNOWN",
  "CMW_DVP",
};

static void CAM_setSensorInfo(CMW_Sensor_Name_t sensor)
{
  int sensor_name_idx = 0;

  switch (sensor) {
  case CMW_DVP_Sensor:
    sensor_width = SENSOR_DVP_WIDTH;
    sensor_height = SENSOR_DVP_HEIGHT;
    sensor_mirror_flip = SENSOR_DVP_FLIP;
    venc_width = VENC_DVP_WIDTH;
    venc_height = VENC_DVP_HEIGHT;
    sensor_name_idx = 1;
    break;
  default:
    assert(0);
  }
  printf("[CAM] detected=%s sensor=%dx%d venc=%dx%d flip=%d\r\n",
         sensor_names[sensor_name_idx],
         sensor_width,
         sensor_height,
         venc_width,
         venc_height,
         sensor_mirror_flip);
}

/* Keep display output aspect ratio using crop area */
static void CAM_InitCropConfig(CMW_Manual_roi_area_t *roi, int sensor_width, int sensor_height)
{
  const float ratiox = (float)sensor_width / VENC_WIDTH;
  const float ratioy = (float)sensor_height / VENC_HEIGHT;
  const float ratio = MIN(ratiox, ratioy);

  assert(ratio >= 1);
  assert(ratio < 64);

  roi->width = (uint32_t) MIN(VENC_WIDTH * ratio, sensor_width);
  roi->height = (uint32_t) MIN(VENC_HEIGHT * ratio, sensor_height);
  roi->offset_x = (sensor_width - roi->width + 1) / 2;
  roi->offset_y = (sensor_height - roi->height + 1) / 2;
}

static void DCMIPP_PipeInitDisplay(int sensor_width, int sensor_height)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  assert(VENC_WIDTH >= VENC_HEIGHT);

  dcmipp_conf.output_width = VENC_WIDTH;
  dcmipp_conf.output_height = VENC_HEIGHT;
  dcmipp_conf.output_format = CAPTURE_FORMAT;
  dcmipp_conf.output_bpp = CAPTURE_BPP;
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = 0;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);

    printf("[CAM] pipe1 cfg out=%lux%lu fmt=%lu bpp=%lu pitch=%lu roi=(%lu,%lu %lux%lu)\r\n",
         (unsigned long)dcmipp_conf.output_width,
         (unsigned long)dcmipp_conf.output_height,
      (unsigned long)dcmipp_conf.output_format,
         (unsigned long)dcmipp_conf.output_bpp,
         (unsigned long)hw_pitch,
         (unsigned long)dcmipp_conf.manual_conf.offset_x,
         (unsigned long)dcmipp_conf.manual_conf.offset_y,
         (unsigned long)dcmipp_conf.manual_conf.width,
         (unsigned long)dcmipp_conf.manual_conf.height);
}

static void DCMIPP_PipeInitNn(int sensor_width, int sensor_height)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  dcmipp_conf.output_width = NN_WIDTH;
  dcmipp_conf.output_height = NN_HEIGHT;
  dcmipp_conf.output_format = NN_FORMAT;
  dcmipp_conf.output_bpp = NN_BPP;
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = 1;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);

  printf("[CAM] pipe2 cfg out=%lux%lu bpp=%lu pitch=%lu swap=%d roi=(%lu,%lu %lux%lu)\r\n",
         (unsigned long)dcmipp_conf.output_width,
         (unsigned long)dcmipp_conf.output_height,
         (unsigned long)dcmipp_conf.output_bpp,
         (unsigned long)hw_pitch,
         dcmipp_conf.enable_swap,
         (unsigned long)dcmipp_conf.manual_conf.offset_x,
         (unsigned long)dcmipp_conf.manual_conf.offset_y,
         (unsigned long)dcmipp_conf.manual_conf.width,
         (unsigned long)dcmipp_conf.manual_conf.height);
}

static void DCMIPP_IpPlugInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  DCMIPP_IPPlugConfTypeDef ipplug_conf = { 0 };
  int ret;

  ipplug_conf.MemoryPageSize = DCMIPP_MEMORY_PAGE_SIZE_256BYTES;

#if APP_DVP_BRINGUP_PIPE1_ONLY
  /* Bring-up mode: dedicate all DCMIPP bandwidth to main display pipe. */
  ipplug_conf.Client = DCMIPP_CLIENT5; /* main rgb pipe */
  ipplug_conf.Traffic = DCMIPP_TRAFFIC_BURST_SIZE_128BYTES;
  ipplug_conf.MaxOutstandingTransactions = DCMIPP_OUTSTANDING_TRANSACTION_3;
  ipplug_conf.DPREGStart = 0;
  ipplug_conf.DPREGEnd = 639;
  ipplug_conf.WLRURatio = 0;
  ret = HAL_DCMIPP_SetIPPlugConfig(hdcmipp, &ipplug_conf);
  assert(ret == HAL_OK);
#else
  ipplug_conf.Client = DCMIPP_CLIENT2; /* aux pipe */
  ipplug_conf.Traffic = DCMIPP_TRAFFIC_BURST_SIZE_128BYTES;
  ipplug_conf.MaxOutstandingTransactions = DCMIPP_OUTSTANDING_TRANSACTION_NONE;
  ipplug_conf.DPREGStart = 0;
  ipplug_conf.DPREGEnd = 559; /* (4480 bytes / one line) */
  ipplug_conf.WLRURatio = 15; /* 16 parts of BW */
  ret = HAL_DCMIPP_SetIPPlugConfig(hdcmipp, &ipplug_conf);
  assert(ret == HAL_OK);

  ipplug_conf.Client = DCMIPP_CLIENT5; /* main rgb pipe */
  ipplug_conf.Traffic = DCMIPP_TRAFFIC_BURST_SIZE_128BYTES;
  ipplug_conf.MaxOutstandingTransactions = DCMIPP_OUTSTANDING_TRANSACTION_3;
  ipplug_conf.DPREGStart = 560;
  ipplug_conf.DPREGEnd = 639;
  ipplug_conf.WLRURatio = 0; /* 1 parts of BW */
  ret = HAL_DCMIPP_SetIPPlugConfig(hdcmipp, &ipplug_conf);
  assert(ret == HAL_OK);
#endif
}

static void DCMIPP_ReduceSpurious(DCMIPP_HandleTypeDef *hdcmipp)
{
  int ret;

  ret = HAL_DCMIPP_PIPE_EnableLineEvent(hdcmipp, DCMIPP_PIPE1, DCMIPP_MULTILINE_128_LINES);
  assert(ret == HAL_OK);
  ret = HAL_DCMIPP_PIPE_DisableLineEvent(hdcmipp, DCMIPP_PIPE1);
  assert(ret == HAL_OK);
}

void CAM_Init(void)
{
  CMW_CameraInit_t cam_conf;
  CMW_Sensor_Name_t sensor;
  int ret;

  printf("[CAM] init start\r\n");
    printf("[CAM] expect PX9210K out: 1920x1080@30 YUV422_8bit\r\n");
    printf("[CAM] build cfg sensor=%dx%d venc=%dx%d pipe1_only=%d\r\n",
      SENSOR_DVP_WIDTH,
      SENSOR_DVP_HEIGHT,
      VENC_DVP_WIDTH,
      VENC_DVP_HEIGHT,
      APP_DVP_BRINGUP_PIPE1_ONLY);

  ret = CMW_CAMERA_GetSensorName(&sensor);
  assert(ret == CMW_ERROR_NONE);
  CAM_setSensorInfo(sensor);

  /* Let sensor driver choose which width/height to use */
  cam_conf.width = sensor_width;
  cam_conf.height = sensor_height;
  cam_conf.fps = CAMERA_FPS;
  cam_conf.mirror_flip = sensor_mirror_flip;
  ret = CMW_CAMERA_Init(&cam_conf, NULL);
  assert(ret == CMW_ERROR_NONE);
  sensor_width = cam_conf.width;
  sensor_height = cam_conf.height;

  printf("[CAM] CMW_CAMERA_Init OK final=%dx%d fps=%d\r\n", sensor_width, sensor_height, cam_conf.fps);

  DCMIPP_IpPlugInit(CMW_CAMERA_GetDCMIPPHandle());
  printf("[CAM] ipplug configured\r\n");
  DCMIPP_PipeInitDisplay(cam_conf.width, cam_conf.height);
#if APP_DVP_BRINGUP_PIPE1_ONLY
  printf("[CAM] bring-up mode: pipe2 config skipped\r\n");
#else
  DCMIPP_PipeInitNn(cam_conf.width, cam_conf.height);
#endif
  DCMIPP_ReduceSpurious(CMW_CAMERA_GetDCMIPPHandle());
  printf("[CAM] init done\r\n");

  APP_CAM_ReadPx9210EssentialRegisters();
}

void CAM_DisplayPipe_Start(uint8_t *display_pipe_dst, uint32_t cam_mode)
{
  int ret;

  printf("[CAM] start pipe1 mode=%lu dst=0x%08lX\r\n",
         (unsigned long)cam_mode,
         (unsigned long)display_pipe_dst);
  ret = CMW_CAMERA_Start(DCMIPP_PIPE1, display_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
  printf("[CAM] pipe1 started\r\n");
}

void CAM_NNPipe_Start(uint8_t *nn_pipe_dst, uint32_t cam_mode)
{
  int ret;

  printf("[CAM] start pipe2 mode=%lu dst=0x%08lX\r\n",
         (unsigned long)cam_mode,
         (unsigned long)nn_pipe_dst);
  ret = CMW_CAMERA_Start(DCMIPP_PIPE2, nn_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
  printf("[CAM] pipe2 started\r\n");
}

void CAM_IspUpdate(void)
{
  int ret;

  ret = CMW_CAMERA_Run();
  assert(ret == CMW_ERROR_NONE);
}

int CAM_GetVencWidth()
{
  assert(venc_width);

  return venc_width;
}

int CAM_GetVencHeight()
{
  assert(venc_height);

  return venc_height;
}

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  printf("[CAM][ERR] DCMIPP pipe error pipe=%lu state=%lu\r\n",
         (unsigned long)pipe,
         (unsigned long)HAL_DCMIPP_PIPE_GetState(CMW_CAMERA_GetDCMIPPHandle(), pipe));
  APP_CAM_DebugOnPipeError(pipe);
  /* FIXME : Need to tune sensor/ipplug so we can remove this implementation */
}
