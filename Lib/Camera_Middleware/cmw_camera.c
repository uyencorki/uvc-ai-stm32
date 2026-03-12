 /**
 ******************************************************************************
 * @file    cmw_camera.c
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


/* Includes ------------------------------------------------------------------*/
#include "cmw_camera.h"

#include "isp_api.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "stm32n6xx_hal.h"
#include "cmw_utils.h"
#include "cmw_io.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "assert.h"

/* Switch capture interface to DVP without removing existing CSI flow. */
#ifndef CMW_CAMERA_USE_DVP
#define CMW_CAMERA_USE_DVP 1
#endif

/* Default DVP input settings; adjust once sensor timing/polarity is known. */
#ifndef CMW_CAMERA_DVP_FORMAT
#define CMW_CAMERA_DVP_FORMAT DCMIPP_FORMAT_YUV422
#endif
#ifndef CMW_CAMERA_DVP_VSPOLARITY
#define CMW_CAMERA_DVP_VSPOLARITY DCMIPP_VSPOLARITY_LOW
#endif
#ifndef CMW_CAMERA_DVP_HSPOLARITY
#define CMW_CAMERA_DVP_HSPOLARITY DCMIPP_HSPOLARITY_LOW
#endif
#ifndef CMW_CAMERA_DVP_PCKPOLARITY
#define CMW_CAMERA_DVP_PCKPOLARITY DCMIPP_PCKPOLARITY_RISING
#endif
#ifndef CMW_CAMERA_DVP_INTERFACE
#define CMW_CAMERA_DVP_INTERFACE DCMIPP_INTERFACE_8BITS
#endif

typedef struct
{
  uint32_t Resolution;
  uint32_t pixel_format;
  uint32_t LightMode;
  uint32_t ColorEffect;
  int32_t  Brightness;
  int32_t  Saturation;
  int32_t  Contrast;
  int32_t  HueDegree;
  int32_t  Gain;
  int32_t  Exposure;
  int32_t  ExposureMode;
  uint32_t MirrorFlip;
  uint32_t Zoom;
  uint32_t NightMode;
  uint32_t IsMspCallbacksValid;
  uint32_t TestPattern;
} CAMERA_Ctx_t;

CMW_CameraInit_t  camera_conf;
CMW_Sensor_Name_t connected_sensor;
CAMERA_Ctx_t  Camera_Ctx;

DCMIPP_HandleTypeDef hcamera_dcmipp;
static CMW_Sensor_if_t Camera_Drv;
static uint8_t camera_bsp;

int is_camera_init = 0;
int is_camera_started = 0;
int is_pipe1_2_shared = 0;

static int32_t CMW_CAMERA_DVP_Init(CMW_Sensor_Init_t *initSensors_params);
static void CMW_CAMERA_EnableGPIOs(void);
static void CMW_CAMERA_PwrDown(void);
static int32_t CMW_CAMERA_SetPipe(DCMIPP_HandleTypeDef *hdcmipp, uint32_t pipe, CMW_DCMIPP_Conf_t *p_conf, uint32_t *pitch);
static int CMW_CAMERA_Probe_Sensor(CMW_Sensor_Init_t *initValues, CMW_Sensor_Name_t *sensorName);

DCMIPP_HandleTypeDef* CMW_CAMERA_GetDCMIPPHandle(void)
{
    return &hcamera_dcmipp;
}

int32_t CMW_CAMERA_SetPipeConfig(uint32_t pipe, CMW_DCMIPP_Conf_t *p_conf, uint32_t *pitch)
{
  return CMW_CAMERA_SetPipe(&hcamera_dcmipp, pipe, p_conf, pitch);
}

/**
  * @brief  Get Sensor name.
  * @param  sensorName  Camera sensor name
  * @retval CMW status
  */
int32_t CMW_CAMERA_GetSensorName(CMW_Sensor_Name_t *sensorName)
{
  int32_t ret = CMW_ERROR_NONE;
  CMW_Sensor_Init_t initValues = {0};

  if (is_camera_init != 0)
  {
    *sensorName = connected_sensor;
    return CMW_ERROR_NONE;
  }

  initValues.width = 0;
  initValues.height = 0;
  initValues.fps = 30;
  initValues.mirrorFlip = CMW_MIRRORFLIP_NONE;
  initValues.sensor_config = NULL;

  /* Set DCMIPP instance */
  hcamera_dcmipp.Instance = DCMIPP;

  /* Configure DCMIPP clock */
  ret = MX_DCMIPP_ClockConfig(&hcamera_dcmipp);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }
  /* Enable DCMIPP clock */
  ret = HAL_DCMIPP_Init(&hcamera_dcmipp);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  CMW_CAMERA_EnableGPIOs();

  ret = CMW_CAMERA_Probe_Sensor(&initValues, &connected_sensor);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_UNKNOWN_COMPONENT;
  }
  *sensorName = connected_sensor;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Set White Balance mode.
  * @param  Automatic  If not null, set automatic white balance mode
  * @param  RefColorTemp  If automatic is null, set white balance mode
  * @retval CMW status
  */
int32_t CMW_CAMERA_SetWBRefMode(uint8_t Automatic, uint32_t RefColorTemp)
{
  int ret;

  if (Camera_Drv.SetWBRefMode == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetWBRefMode(&camera_bsp, Automatic, RefColorTemp);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  ret = CMW_ERROR_NONE;
  /* Return CMW status */
  return ret;
}

/**
  * @brief  Get White Balance reference modes list.
  * @param  RefColorTemp  White Balance reference modes
  * @retval CMW status
  */
int32_t CMW_CAMERA_ListWBRefModes(uint32_t RefColorTemp[])
{
  int ret;

  if (Camera_Drv.ListWBRefModes == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.ListWBRefModes(&camera_bsp, RefColorTemp);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  ret = CMW_ERROR_NONE;
  /* Return CMW status */
  return ret;
}

/**
  * @brief  Probe camera sensor.
  * @param  initValues  Initialization values for the sensor
  * @param  sensorName  Camera sensor name
  * @retval CMW status
  */
static int CMW_CAMERA_Probe_Sensor(CMW_Sensor_Init_t *initValues, CMW_Sensor_Name_t *sensorName)
{
  int ret;

  ret = CMW_CAMERA_DVP_Init(initValues);
  if (ret != CMW_ERROR_NONE)
  {
    return ret;
  }

  *sensorName = CMW_DVP_Sensor;
  return CMW_ERROR_NONE;
}



/**
  * @brief  Initializes the camera.
  * @param  initConf  Mandatory: General camera config
  * @param  advanced_config  Optional: Sensor specific configuration
  * @retval CMW status
  */
int32_t CMW_CAMERA_Init(CMW_CameraInit_t *initConf, CMW_Advanced_Config_t *advanced_config)
{
  int32_t ret = CMW_ERROR_NONE;
  CMW_Sensor_Init_t initValues = {0};

  initValues.width = initConf->width;
  initValues.height = initConf->height;
  initValues.fps = initConf->fps;
  initValues.mirrorFlip = initConf->mirror_flip;

  if ((advanced_config != NULL) && (advanced_config->selected_sensor != CMW_UNKNOWN_Sensor))
  {
    /* Assume The sensor is the one selected by the application. Check during probe */
    connected_sensor = advanced_config->selected_sensor;
    initValues.sensor_config = (void *) &advanced_config->config_sensor;
  }
  else
  {
    connected_sensor = CMW_UNKNOWN_Sensor;
    initValues.sensor_config = NULL;
  }

  /* Set DCMIPP instance */
  hcamera_dcmipp.Instance = DCMIPP;

  /* Configure DCMIPP clock */
  ret = MX_DCMIPP_ClockConfig(&hcamera_dcmipp);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }
  /* Enable DCMIPP clock */
  ret = HAL_DCMIPP_Init(&hcamera_dcmipp);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  CMW_CAMERA_EnableGPIOs();

  ret = CMW_CAMERA_Probe_Sensor(&initValues, &connected_sensor);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_UNKNOWN_COMPONENT;
  }

  /* Write back the initValue width and height that might be changed */
  initConf->width = initValues.width;
  initConf->height = initValues.height ;
  camera_conf = *initConf;

  is_camera_init++;
  /* CMW status */
  ret = CMW_ERROR_NONE;
  return ret;
}

/**
  * @brief  Set the camera Mirror/Flip.
  * @param  MirrorFlip CMW_MIRRORFLIP_NONE CMW_MIRRORFLIP_FLIP CMW_MIRRORFLIP_MIRROR CMW_MIRRORFLIP_FLIP_MIRROR
  * @retval CMW status
*/
int32_t CMW_CAMERA_SetMirrorFlip(int32_t MirrorFlip)
{
  int ret;

  if (Camera_Drv.SetMirrorFlip == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetMirrorFlip(&camera_bsp, MirrorFlip);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  camera_conf.mirror_flip = MirrorFlip;
  ret = CMW_ERROR_NONE;
  /* Return CMW status */
  return ret;
}

/**
  * @brief  Get the camera Mirror/Flip.
  * @param  MirrorFlip CMW_MIRRORFLIP_NONE CMW_MIRRORFLIP_FLIP CMW_MIRRORFLIP_MIRROR CMW_MIRRORFLIP_FLIP_MIRROR
  * @retval CMW status
*/
int32_t CMW_CAMERA_GetMirrorFlip(int32_t *MirrorFlip)
{
  *MirrorFlip = camera_conf.mirror_flip;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Starts the camera capture in the selected mode.
  * @param  pipe  DCMIPP Pipe
  * @param  pbuff pointer to the camera output buffer
  * @param  mode  CMW_MODE_CONTINUOUS or CMW_MODE_SNAPSHOT
  * @retval CMW status
  */
int32_t CMW_CAMERA_Start(uint32_t pipe, uint8_t *pbuff, uint32_t mode)
{
  int32_t ret = CMW_ERROR_NONE;

  printf("[CMW] start pipe=%lu mode=%lu buf=0x%08lX\r\n",
         (unsigned long)pipe,
         (unsigned long)mode,
         (unsigned long)pbuff);

  if (pipe >= DCMIPP_NUM_OF_PIPES)
  {
    return CMW_ERROR_WRONG_PARAM;
  }

#if CMW_CAMERA_USE_DVP
  /* DVP path: start capture from parallel input (no CSI VC). */
  ret = HAL_DCMIPP_PIPE_Start(&hcamera_dcmipp, pipe, (uint32_t)pbuff, mode);
#else
  /* CSI path: keep legacy behavior for fallback/testing. */
  ret = HAL_DCMIPP_CSI_PIPE_Start(&hcamera_dcmipp, pipe, DCMIPP_VIRTUAL_CHANNEL0, (uint32_t)pbuff, mode);
#endif
  if (ret != HAL_OK)
  {
    printf("[CMW] start pipe=%lu HAL error=%ld\r\n", (unsigned long)pipe, (long)ret);
    return CMW_ERROR_PERIPH_FAILURE;
  }

  if (!is_camera_started)
  {
    if (Camera_Drv.Start != NULL)
    {
      ret = Camera_Drv.Start(&camera_bsp);
      if (ret != CMW_ERROR_NONE)
      {
        printf("[CMW] sensor start failed=%ld\r\n", (long)ret);
        return CMW_ERROR_COMPONENT_FAILURE;
      }
    }
    is_camera_started++;
  }

  /* Return CMW status */
  printf("[CMW] start pipe=%lu done\r\n", (unsigned long)pipe);
  return ret;
}

#if defined (STM32N657xx)
/**
  * @brief  Starts the camera capture in the selected mode.
  * @param  pipe  DCMIPP Pipe
  * @param  pbuff1 pointer to the first camera output buffer
  * @param  pbuff2 pointer to the second camera output buffer
  * @param  mode  CMW_MODE_CONTINUOUS or CMW_MODE_SNAPSHOT
  * @retval CMW status
  */
int32_t CMW_CAMERA_DoubleBufferStart(uint32_t pipe, uint8_t *pbuff1, uint8_t *pbuff2, uint32_t Mode)
{
  int32_t ret = CMW_ERROR_NONE;

  if (pipe >= DCMIPP_NUM_OF_PIPES)
  {
    return CMW_ERROR_WRONG_PARAM;
  }

#if CMW_CAMERA_USE_DVP
  /* DVP path: double buffer start from parallel input. */
  if (HAL_DCMIPP_PIPE_DoubleBufferStart(&hcamera_dcmipp, pipe, (uint32_t)pbuff1, (uint32_t)pbuff2, Mode) != HAL_OK)
#else
  if (HAL_DCMIPP_CSI_PIPE_DoubleBufferStart(&hcamera_dcmipp, pipe, DCMIPP_VIRTUAL_CHANNEL0, (uint32_t)pbuff1,
                                            (uint32_t)pbuff2, Mode) != HAL_OK)
#endif
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  if (!is_camera_started)
  {
    if (Camera_Drv.Start != NULL)
    {
      ret = Camera_Drv.Start(&camera_bsp);
      if (ret != CMW_ERROR_NONE)
      {
        return CMW_ERROR_COMPONENT_FAILURE;
      }
    }
    is_camera_started++;
  }

  /* Return CMW status */
  return ret;
}
#endif




/**
  * @brief  DCMIPP Clock Config for DCMIPP.
  * @param  hdcmipp  DCMIPP Handle
  *         Being __weak it can be overwritten by the application
  * @retval HAL_status
  */
__weak HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
  UNUSED(hdcmipp);

  return HAL_OK;
}

/**
  * @brief  DeInitializes the camera.
  * @retval CMW status
  */
int32_t CMW_CAMERA_DeInit(void)
{
  int32_t ret = CMW_ERROR_NONE;

  if (HAL_DCMIPP_PIPE_GetState(&hcamera_dcmipp, DCMIPP_PIPE1) != HAL_DCMIPP_PIPE_STATE_RESET)
  {
#if CMW_CAMERA_USE_DVP
    /* DVP path: stop standard pipe capture. */
    ret = HAL_DCMIPP_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE1);
#else
    ret = HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
#endif
    if (ret != HAL_OK)
    {
      return CMW_ERROR_PERIPH_FAILURE;
    }
  }

  if (HAL_DCMIPP_PIPE_GetState(&hcamera_dcmipp, DCMIPP_PIPE2) != HAL_DCMIPP_PIPE_STATE_RESET)
  {
#if CMW_CAMERA_USE_DVP
    ret = HAL_DCMIPP_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE2);
#else
    ret = HAL_DCMIPP_CSI_PIPE_Stop(&hcamera_dcmipp, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0);
#endif
    if (ret != HAL_OK)
    {
      return CMW_ERROR_PERIPH_FAILURE;
    }
  }

  ret = HAL_DCMIPP_DeInit(&hcamera_dcmipp);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  if (is_camera_init <= 0)
  {
    return CMW_ERROR_NONE;
  }

  /* De-initialize the camera module */
  if (Camera_Drv.DeInit != NULL)
  {
    ret = Camera_Drv.DeInit(&camera_bsp);
    if (ret != CMW_ERROR_NONE)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }
  /* Set Camera in Power Down */
  CMW_CAMERA_PwrDown();

  /* Update DCMIPPInit counter */
  is_camera_init--;
  is_camera_started--;
  is_pipe1_2_shared--;

  /* Return CMW status */
  ret = CMW_ERROR_NONE;
  return ret;
}

/**
  * @brief  Suspend the CAMERA capture on selected pipe
  * @param  pipe Dcmipp pipe.
  * @retval CMW status
  */
int32_t CMW_CAMERA_Suspend(uint32_t pipe)
{
  HAL_DCMIPP_PipeStateTypeDef state = hcamera_dcmipp.PipeState[pipe];

  if (state == HAL_DCMIPP_PIPE_STATE_SUSPEND)
  {
    return CMW_ERROR_NONE;
  }
  else if (state > HAL_DCMIPP_PIPE_STATE_READY)
  {
    if (HAL_DCMIPP_PIPE_Suspend(&hcamera_dcmipp, pipe) != HAL_OK)
    {
      return CMW_ERROR_PERIPH_FAILURE;
    }
  }

  /* Return CMW status */
  return CMW_ERROR_NONE;
}

/**
  * @brief  Resume the CAMERA capture on selected pipe
  * @param  pipe Dcmipp pipe.
  * @retval CMW status
  */
int32_t CMW_CAMERA_Resume(uint32_t pipe)
{
  HAL_DCMIPP_PipeStateTypeDef state = hcamera_dcmipp.PipeState[pipe];

  if (state == HAL_DCMIPP_PIPE_STATE_BUSY)
  {
    return CMW_ERROR_NONE;
  }
  else if (state > HAL_DCMIPP_PIPE_STATE_BUSY)
  {
    if (HAL_DCMIPP_PIPE_Resume(&hcamera_dcmipp, pipe) != HAL_OK)
    {
      return CMW_ERROR_PERIPH_FAILURE;
    }
  }

  /* Return CMW status */
  return CMW_ERROR_NONE;
}

/**
  * @brief  Enable the Restart State. When enabled, at system restart, the ISP middleware configuration
  *         is restored from the last update before the restart.
  * @param  ISP_RestartState pointer to ISP Restart State. To use this mode in a Low Power use case, where
  *         the ISP state is applied at system wake up, this pointer must be in some retention memory.
  * @retval CMW status
  */
int32_t CMW_CAMERA_EnableRestartState(ISP_RestartStateTypeDef *ISP_RestartState)
{
  if (ISP_EnableRestartState(NULL, ISP_RestartState) != ISP_OK)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  /* Return CMW status */
  return CMW_ERROR_NONE;
}

/**
  * @brief  Disable the Restart State
  * @retval CMW status
  */
int32_t CMW_CAMERA_DisableRestartState()
{
  if (ISP_DisableRestartState(NULL) != ISP_OK)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  /* Return CMW status */
  return CMW_ERROR_NONE;
}

/**
  * @brief  Set the camera gain.
  * @param  Gain     Gain in mdB
  * @retval CMW status
  */
int CMW_CAMERA_SetGain(int32_t Gain)
{
  int ret;
  if(Camera_Drv.SetGain == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetGain(&camera_bsp, Gain);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  Camera_Ctx.Gain = Gain;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Get the camera gain.
  * @param  Gain     Gain in mdB
  * @retval CMW status
  */
int CMW_CAMERA_GetGain(int32_t *Gain)
{
  *Gain = Camera_Ctx.Gain;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Set the camera exposure.
  * @param  exposure exposure in microseconds
  * @retval CMW status
  */
int CMW_CAMERA_SetExposure(int32_t exposure)
{
  int ret;

  if(Camera_Drv.SetExposure == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetExposure(&camera_bsp, exposure);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  Camera_Ctx.Exposure = exposure;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Get the camera exposure.
  * @param  exposure exposure in microseconds
  * @retval CMW status
  */
int CMW_CAMERA_GetExposure(int32_t *exposure)
{
  *exposure = Camera_Ctx.Exposure;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Set the camera exposure mode.
  * @param  exposureMode Exposure mode CMW_EXPOSUREMODE_AUTO, CMW_EXPOSUREMODE_AUTOFREEZE, CMW_EXPOSUREMODE_MANUAL
  * @retval CMW status
  */
int32_t CMW_CAMERA_SetExposureMode(int32_t exposureMode)
{
  int ret;

  if(Camera_Drv.SetExposureMode == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetExposureMode(&camera_bsp, exposureMode);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  Camera_Ctx.ExposureMode = exposureMode;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Get the camera exposure mode.
  * @param  exposureMode Exposure mode CAMERA_EXPOSURE_AUTO, CAMERA_EXPOSURE_AUTOFREEZE, CAMERA_EXPOSURE_MANUAL
  * @retval CMW status
  */
int32_t CMW_CAMERA_GetExposureMode(int32_t *exposureMode)
{
  *exposureMode = Camera_Ctx.ExposureMode;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Set (Enable/Disable and Configure) the camera test pattern
  * @param  mode Pattern mode (sensor specific value) to be configured. '-1' means disable.
  * @retval CMW status
  */
int32_t CMW_CAMERA_SetTestPattern(int32_t mode)
{
  int32_t ret;

  if(Camera_Drv.SetTestPattern == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.SetTestPattern(&camera_bsp, mode);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  Camera_Ctx.TestPattern = mode;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Get the camera test pattern
  * @param  mode Pattern mode (sensor specific value) to be returned. '-1' means disable.
  * @retval CMW status
  */
int32_t CMW_CAMERA_GetTestPattern(int32_t *mode)
{
  *mode = Camera_Ctx.TestPattern;
  return CMW_ERROR_NONE;
}

/**
  * @brief  Get the Camera Sensor info.
  * @param  info  pointer to sensor info
  * @note   This function should be called after the init. This to get Capabilities
  *         from the camera sensor
  * @retval Component status
  */
int32_t CMW_CAMERA_GetSensorInfo(ISP_SensorInfoTypeDef *info)
{

  int32_t ret;

  if(Camera_Drv.GetSensorInfo == NULL)
  {
    return CMW_ERROR_FEATURE_NOT_SUPPORTED;
  }

  ret = Camera_Drv.GetSensorInfo(&camera_bsp, info);
  if (ret != CMW_ERROR_NONE)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  return CMW_ERROR_NONE;
}



int32_t CMW_CAMERA_Run()
{
  if(Camera_Drv.Run != NULL)
  {
      return Camera_Drv.Run(&camera_bsp);
  }
  return CMW_ERROR_NONE;
}

/**
 * @brief  Vsync Event callback on pipe
 * @param  Pipe  Pipe receiving the callback
 * @retval None
 */
__weak int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  UNUSED(pipe);

  return CMW_ERROR_NONE;
}

/**
 * @brief  Frame Event callback on pipe
 * @param  Pipe  Pipe receiving the callback
 * @retval None
 */
__weak int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  UNUSED(pipe);

  return CMW_ERROR_NONE;
}

/**
 * @brief  Error callback on pipe
 * @param  Pipe  Pipe receiving the callback
 * @retval None
 */
__weak void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  assert(0);
}

/**
 * @brief  Vsync Event callback on pipe
 * @param  hdcmipp DCMIPP device handle
 *         Pipe    Pipe receiving the callback
 * @retval None
 */
void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  UNUSED(hdcmipp);
  if(Camera_Drv.VsyncEventCallback != NULL)
  {
      Camera_Drv.VsyncEventCallback(&camera_bsp, Pipe);
  }
  CMW_CAMERA_PIPE_VsyncEventCallback(Pipe);
}

/**
 * @brief  Frame Event callback on pipe
 * @param  hdcmipp DCMIPP device handle
 *         Pipe    Pipe receiving the callback
 * @retval None
 */
void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  UNUSED(hdcmipp);

  if(Camera_Drv.FrameEventCallback != NULL)
  {
      Camera_Drv.FrameEventCallback(&camera_bsp, Pipe);
  }
  CMW_CAMERA_PIPE_FrameEventCallback(Pipe);
}

/**
  * @brief  Initializes the DCMIPP MSP.
  * @param  hdcmipp  DCMIPP handle
  * @retval None
  */
void HAL_DCMIPP_MspInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  UNUSED(hdcmipp);

  /*** Enable peripheral clock ***/
  /* Enable DCMIPP clock */
  __HAL_RCC_DCMIPP_CLK_ENABLE();
  __HAL_RCC_DCMIPP_CLK_SLEEP_ENABLE();
  __HAL_RCC_DCMIPP_FORCE_RESET();
  __HAL_RCC_DCMIPP_RELEASE_RESET();

  /*** Configure the NVIC for DCMIPP ***/
  /* NVIC configuration for DCMIPP transfer complete interrupt */
  HAL_NVIC_SetPriority(DCMIPP_IRQn, 0x07, 0);
  HAL_NVIC_EnableIRQ(DCMIPP_IRQn);

  /*** Enable peripheral clock ***/
  /* Enable CSI clock */
  __HAL_RCC_CSI_CLK_ENABLE();
  __HAL_RCC_CSI_CLK_SLEEP_ENABLE();
  __HAL_RCC_CSI_FORCE_RESET();
  __HAL_RCC_CSI_RELEASE_RESET();

  /*** Configure the NVIC for CSI ***/
  /* NVIC configuration for CSI transfer complete interrupt */
  HAL_NVIC_SetPriority(CSI_IRQn, 0x07, 0);
  HAL_NVIC_EnableIRQ(CSI_IRQn);

}

/**
  * @brief  DeInitializes the DCMIPP MSP.
  * @param  hdcmipp  DCMIPP handle
  * @retval None
  */
void HAL_DCMIPP_MspDeInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  UNUSED(hdcmipp);

  __HAL_RCC_DCMIPP_FORCE_RESET();
  __HAL_RCC_DCMIPP_RELEASE_RESET();

  /* Disable NVIC  for DCMIPP transfer complete interrupt */
  HAL_NVIC_DisableIRQ(DCMIPP_IRQn);

  /* Disable DCMIPP clock */
  __HAL_RCC_DCMIPP_CLK_DISABLE();

  __HAL_RCC_CSI_FORCE_RESET();
  __HAL_RCC_CSI_RELEASE_RESET();

  /* Disable NVIC  for DCMIPP transfer complete interrupt */
  HAL_NVIC_DisableIRQ(CSI_IRQn);

  /* Disable DCMIPP clock */
  __HAL_RCC_CSI_CLK_DISABLE();
}

/**
  * @brief  CAMERA hardware reset
  * @retval CMW status
  */
static void CMW_CAMERA_EnableGPIOs(void)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  /* Enable GPIO clocks */
  EN_CAM_GPIO_ENABLE_VDDIO();
  EN_CAM_GPIO_CLK_ENABLE();
  NRST_CAM_GPIO_ENABLE_VDDIO();
  NRST_CAM_GPIO_CLK_ENABLE();

  gpio_init_structure.Pin       = EN_CAM_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(EN_CAM_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = NRST_CAM_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NRST_CAM_PORT, &gpio_init_structure);
}

/**
  * @brief  CAMERA power down
  * @retval CMW status
  */
static void CMW_CAMERA_PwrDown(void)
{
  GPIO_InitTypeDef gpio_init_structure = {0};

  gpio_init_structure.Pin       = EN_CAM_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(EN_CAM_PORT, &gpio_init_structure);

  gpio_init_structure.Pin       = NRST_CAM_PIN;
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(NRST_CAM_PORT, &gpio_init_structure);

  /* Camera power down sequence */
  /* Assert the camera Enable pin (active high) */
  HAL_GPIO_WritePin(EN_CAM_PORT, EN_CAM_PIN, GPIO_PIN_RESET);

  /* De-assert the camera NRST pin (active low) */
  HAL_GPIO_WritePin(NRST_CAM_PORT, NRST_CAM_PIN, GPIO_PIN_RESET);

}

static void CMW_CAMERA_ShutdownPin(int value)
{
  HAL_GPIO_WritePin(NRST_CAM_PORT, NRST_CAM_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void CMW_CAMERA_EnablePin(int value)
{
  HAL_GPIO_WritePin(EN_CAM_PORT, EN_CAM_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static ISP_StatusTypeDef CB_ISP_SetSensorGain(uint32_t camera_instance, int32_t gain)
{
  if (CMW_CAMERA_SetGain(gain) != CMW_ERROR_NONE)
    return ISP_ERR_SENSORGAIN;

  return ISP_OK;
}

static ISP_StatusTypeDef CB_ISP_GetSensorGain(uint32_t camera_instance, int32_t *gain)
{
  if (CMW_CAMERA_GetGain(gain) != CMW_ERROR_NONE)
    return ISP_ERR_SENSORGAIN;

  return ISP_OK;
}

static ISP_StatusTypeDef CB_ISP_SetSensorExposure(uint32_t camera_instance, int32_t exposure)
{
  if (CMW_CAMERA_SetExposure(exposure) != CMW_ERROR_NONE)
    return ISP_ERR_SENSOREXPOSURE;

  return ISP_OK;
}

static ISP_StatusTypeDef CB_ISP_GetSensorExposure(uint32_t camera_instance, int32_t *exposure)
{
  if (CMW_CAMERA_GetExposure(exposure) != CMW_ERROR_NONE)
    return ISP_ERR_SENSOREXPOSURE;

  return ISP_OK;
}

static ISP_StatusTypeDef CB_ISP_GetSensorInfo(uint32_t camera_instance, ISP_SensorInfoTypeDef *Info)
{
  if(Camera_Drv.GetSensorInfo != NULL)
  {
    if (Camera_Drv.GetSensorInfo(&camera_bsp, Info) != CMW_ERROR_NONE)
      return ISP_ERR_SENSOREXPOSURE;
  }
  return ISP_OK;
}


static int32_t CMW_CAMERA_DVP_Init(CMW_Sensor_Init_t *initSensors_params)
{
  printf("[CMW] DVP init requested %lux%lu fps=%ld\r\n",
         (unsigned long)initSensors_params->width,
         (unsigned long)initSensors_params->height,
         (long)initSensors_params->fps);
  int32_t ret = CMW_ERROR_NONE;
#if !CMW_CAMERA_USE_DVP
  DCMIPP_CSI_ConfTypeDef csi_conf = { 0 };
  DCMIPP_CSI_PIPE_ConfTypeDef csi_pipe_conf = { 0 };
  uint32_t dt_format = 0;
  uint32_t dt = 0;
#endif

  CMW_DVP_config_t default_sensor_config = {0};
  CMW_DVP_config_t *sensor_config;
  memset(&Camera_Drv, 0, sizeof(Camera_Drv));

  default_sensor_config.pixel_format = CMW_PIXEL_FORMAT_YUV422_8;
  initSensors_params->sensor_config = initSensors_params->sensor_config ? initSensors_params->sensor_config : &default_sensor_config;
  sensor_config = (CMW_DVP_config_t *)(initSensors_params->sensor_config);
  printf("[CMW] sensor pixel_format=%lu\r\n", (unsigned long)sensor_config->pixel_format);

  if ((initSensors_params->width == 0U) || (initSensors_params->height == 0U))
  {
    initSensors_params->width = 1920;
    initSensors_params->height = 1080;
    printf("[CMW] DVP init defaulted to 1920x1080\r\n");
  }

  /* DVP path: configure parallel receiver instead of CSI host/VC. */
#if CMW_CAMERA_USE_DVP
  DCMIPP_ParallelConfTypeDef parallel_conf = { 0 };

  parallel_conf.Format = CMW_CAMERA_DVP_FORMAT;
  parallel_conf.VSPolarity = CMW_CAMERA_DVP_VSPOLARITY;
  parallel_conf.HSPolarity = CMW_CAMERA_DVP_HSPOLARITY;
  parallel_conf.PCKPolarity = CMW_CAMERA_DVP_PCKPOLARITY;
  parallel_conf.ExtendedDataMode = CMW_CAMERA_DVP_INTERFACE;
  parallel_conf.SynchroMode = DCMIPP_SYNCHRO_HARDWARE;
  parallel_conf.SwapBits = DCMIPP_SWAPBITS_DISABLE;
  parallel_conf.SwapCycles = DCMIPP_SWAPCYCLES_DISABLE;

  ret = HAL_DCMIPP_PARALLEL_SetConfig(&hcamera_dcmipp, &parallel_conf);
  if (ret != HAL_OK)
  {
    printf("[CMW] DVP parallel config failed=%ld\r\n", (long)ret);
    return CMW_ERROR_PERIPH_FAILURE;
  }

  printf("[CMW] DVP cfg fmt=%lu hspol=%lu vspol=%lu pckpol=%lu if=%lu\r\n",
         (unsigned long)parallel_conf.Format,
         (unsigned long)parallel_conf.HSPolarity,
         (unsigned long)parallel_conf.VSPolarity,
         (unsigned long)parallel_conf.PCKPolarity,
         (unsigned long)parallel_conf.ExtendedDataMode);
#else
  switch (sensor_config->pixel_format)
  {
    case CMW_PIXEL_FORMAT_DEFAULT:
    case CMW_PIXEL_FORMAT_RAW10:
    {
      dt_format = DCMIPP_CSI_DT_BPP10;
      dt = DCMIPP_DT_RAW10;
      break;
    }
    default:
      return CMW_ERROR_COMPONENT_FAILURE;
  }

  csi_conf.NumberOfLanes = DCMIPP_CSI_TWO_DATA_LANES;
  csi_conf.DataLaneMapping = DCMIPP_CSI_PHYSICAL_DATA_LANES;
  csi_conf.PHYBitrate = DCMIPP_CSI_PHY_BT_1600;
  ret = HAL_DCMIPP_CSI_SetConfig(&hcamera_dcmipp, &csi_conf);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  ret = HAL_DCMIPP_CSI_SetVCConfig(&hcamera_dcmipp, DCMIPP_VIRTUAL_CHANNEL0, dt_format);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_PERIPH_FAILURE;
  }

  csi_pipe_conf.DataTypeMode = DCMIPP_DTMODE_DTIDA;
  csi_pipe_conf.DataTypeIDA = dt;
  csi_pipe_conf.DataTypeIDB = 0;
  /* Pre-initialize CSI config for all the pipes */
  for (uint32_t i = DCMIPP_PIPE0; i <= DCMIPP_PIPE2; i++)
  {
    ret = HAL_DCMIPP_CSI_PIPE_SetConfig(&hcamera_dcmipp, i, &csi_pipe_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_PERIPH_FAILURE;
    }
  }
#endif

  printf("[CMW] DVP init done\r\n");


  return ret;
}

static int32_t CMW_CAMERA_SetPipe(DCMIPP_HandleTypeDef *hdcmipp, uint32_t pipe, CMW_DCMIPP_Conf_t *p_conf, uint32_t *pitch)
{
  DCMIPP_DecimationConfTypeDef dec_conf = { 0 };
  DCMIPP_PipeConfTypeDef pipe_conf = { 0 };
  DCMIPP_DownsizeTypeDef down_conf = { 0 };
  DCMIPP_CropConfTypeDef crop_conf = { 0 };
  int ret;

  /* specific case for pipe0 which is only a dump pipe */
  if (pipe == DCMIPP_PIPE0)
  {
    /*  TODO: properly configure the dump pipe with decimation and crop */
    pipe_conf.FrameRate = DCMIPP_FRAME_RATE_ALL;
    ret = HAL_DCMIPP_PIPE_SetConfig(hdcmipp, pipe, &pipe_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }

    return CMW_ERROR_NONE;
  }

  CMW_UTILS_GetPipeConfig(camera_conf.width, camera_conf.height, p_conf, &crop_conf, &dec_conf, &down_conf);

  if (crop_conf.VSize != 0 || crop_conf.HSize != 0)
  {
    ret = HAL_DCMIPP_PIPE_SetCropConfig(hdcmipp, pipe, &crop_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }

    ret = HAL_DCMIPP_PIPE_EnableCrop(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = HAL_DCMIPP_PIPE_DisableCrop(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }

  if (dec_conf.VRatio != 0 || dec_conf.HRatio != 0)
  {
    ret = HAL_DCMIPP_PIPE_SetDecimationConfig(hdcmipp, pipe, &dec_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }

    ret = HAL_DCMIPP_PIPE_EnableDecimation(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = HAL_DCMIPP_PIPE_DisableDecimation(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }

  ret = HAL_DCMIPP_PIPE_SetDownsizeConfig(hdcmipp, pipe, &down_conf);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  ret = HAL_DCMIPP_PIPE_EnableDownsize(hdcmipp, pipe);
  if (ret != HAL_OK)
  {
    return CMW_ERROR_COMPONENT_FAILURE;
  }

  if (p_conf->enable_swap)
  {
    /* Config pipe */
    ret = HAL_DCMIPP_PIPE_EnableRedBlueSwap(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = HAL_DCMIPP_PIPE_DisableRedBlueSwap(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }

  /* Ignore the configuration of gamma if -1
   * Activation is then done by the ISP Library
   */
  if (p_conf->enable_gamma_conversion > -1)
  {
    if (p_conf->enable_gamma_conversion)
    {
      ret = HAL_DCMIPP_PIPE_EnableGammaConversion(hdcmipp, pipe);
      if (ret != HAL_OK)
      {
        return CMW_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = HAL_DCMIPP_PIPE_DisableGammaConversion(hdcmipp, pipe);
      if (ret != HAL_OK)
      {
        return CMW_ERROR_COMPONENT_FAILURE;
      }
    }
  }

  if (pipe == DCMIPP_PIPE2)
  {
    if (!is_pipe1_2_shared)
    {
#if !CMW_CAMERA_USE_DVP
      /* CSI-only: pipe2 shares CSI input path with pipe1. */
      ret = HAL_DCMIPP_PIPE_CSI_EnableShare(hdcmipp, pipe);
      if (ret != HAL_OK)
      {
        return CMW_ERROR_COMPONENT_FAILURE;
      }
#endif
      /* Keep state aligned in both modes to avoid re-entering this block. */
      is_pipe1_2_shared++;
    }
  }

  pipe_conf.FrameRate = DCMIPP_FRAME_RATE_ALL;
  pipe_conf.PixelPipePitch = p_conf->output_width * p_conf->output_bpp;
  /* Hardware constraint, pitch must be multiple of 16 */
  pipe_conf.PixelPipePitch = (pipe_conf.PixelPipePitch + 15) & (uint32_t) ~15;
  pipe_conf.PixelPackerFormat = p_conf->output_format;

  /* Support of YUV pixel format */
  if (pipe_conf.PixelPackerFormat == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
  {
    if (pipe != DCMIPP_PIPE1)
    {
      /* Only pipe 1 support YUV conversion */
      return CMW_ERROR_FEATURE_NOT_SUPPORTED;
    }

    #define N10(val) (((val) ^ 0x7FF) + 1)
    DCMIPP_ColorConversionConfTypeDef yuv_color_conf = {
    .ClampOutputSamples = ENABLE,
    .OutputSamplesType = 0,
    .RR = 131,     .RG = N10(110), .RB = N10(21), .RA = 128,
    .GR = 77,      .GG = 150,      .GB = 29,      .GA = 0,
    .BR = N10(44), .BG = N10(87),  .BB = 131,     .BA = 128,
    };

    ret = HAL_DCMIPP_PIPE_SetYUVConversionConfig(hdcmipp, pipe, &yuv_color_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
    ret = HAL_DCMIPP_PIPE_EnableYUVConversion(hdcmipp, pipe);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }

  if (hcamera_dcmipp.PipeState[pipe] == HAL_DCMIPP_PIPE_STATE_RESET)
  {
    ret = HAL_DCMIPP_PIPE_SetConfig(hdcmipp, pipe, &pipe_conf);
    if (ret != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    if (HAL_DCMIPP_PIPE_SetPixelPackerFormat(hdcmipp, pipe, pipe_conf.PixelPackerFormat) != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }

    if (HAL_DCMIPP_PIPE_SetPitch(hdcmipp, pipe, pipe_conf.PixelPipePitch) != HAL_OK)
    {
      return CMW_ERROR_COMPONENT_FAILURE;
    }
  }

  /* Update the pitch field so that application can use this information for
   * buffer alignement */
  *pitch = pipe_conf.PixelPipePitch;

  return CMW_ERROR_NONE;
}

int32_t CMW_CAMERA_SetDefaultSensorValues( CMW_Advanced_Config_t *advanced_config )
{
  if (advanced_config == NULL)
  {
    return CMW_ERROR_WRONG_PARAM;
  }

  advanced_config->config_sensor.dvp_config.pixel_format = CMW_PIXEL_FORMAT_RAW10;

  return CMW_ERROR_NONE;
}

/**
  * @brief  Error callback on the pipe. Occurs when overrun occurs on the pipe.
  * @param  hdcmipp  Pointer to DCMIPP handle
  * @param  Pipe     Specifies the DCMIPP pipe, can be a value from @ref DCMIPP_Pipes
  * @retval None
  */
void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  UNUSED(hdcmipp);

  CMW_CAMERA_PIPE_ErrorCallback(Pipe);
}
