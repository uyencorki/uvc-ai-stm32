/**
 ******************************************************************************
 * @file    enc.c
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
#include "app_enc.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#if APP_ENC_USE_H264
#include "h264encapi.h"
#else
#include "jpegencapi.h"
#endif
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "ewl.h"
#include "app_config.h"

#define APP_ENC_INFO_LOG_ENABLE 0
#define APP_ENC_ERROR_LOG_ENABLE 1
#define APP_ENC_DEBUG_FRAME_LOG_ENABLE 1

#if APP_ENC_INFO_LOG_ENABLE
#define ENC_INFO_LOG(...) printf(__VA_ARGS__)
#else
#define ENC_INFO_LOG(...) do {} while (0)
#endif

#if APP_ENC_ERROR_LOG_ENABLE
#define ENC_ERR_LOG(...) printf(__VA_ARGS__)
#else
#define ENC_ERR_LOG(...) do {} while (0)
#endif

#if APP_ENC_DEBUG_FRAME_LOG_ENABLE
#define ENC_DBG_LOG(...) printf(__VA_ARGS__)
#else
#define ENC_DBG_LOG(...) do {} while (0)
#endif

#define VENC_CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

static int is_cache_enable(void)
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

#define VENC_ALLOCATOR_BUDGET_SIZE (8U * 1024U * 1024U)
#define VENC_LINEAR_ALIGN_BYTES 64U
#define VENC_LINEAR_MAX_BLOCKS 64U
#if APP_ENC_USE_H264
#define H264_TARGET_BITRATE_BPS 4000000U
#define H264_QP_HDR_DEFAULT 28
#define H264_QP_MIN_DEFAULT 10
#define H264_QP_MAX_DEFAULT 45
#define H264_START_BUF_SIZE 4096U
#else
/* 0 = strongest compression, 10 = highest quality (larger output). */
#define JPEG_QUALITY_LEVEL 0
#endif

typedef struct
{
  void *raw_ptr;
  uint32_t alloc_size;
} VENC_LinearBlock_t;

static VENC_LinearBlock_t venc_linear_blocks[VENC_LINEAR_MAX_BLOCKS];
static uint32_t venc_linear_block_count;
static uint32_t venc_linear_alloc_bytes;
static int venc_init_failed_logged;
static uint32_t venc_linear_alloc_count;
static uint32_t venc_encode_ok_count;
static uint32_t venc_encode_fail_count;
static uint32_t venc_encode_last_log_ms;
static uint32_t venc_copy_out_log_ms;
static uint32_t venc_debug_frame_id;
#if !APP_ENC_USE_H264
/* JFIF headers are CPU-written; entropy payload is HW-written. */
#define VENC_CPU_HEADER_FLUSH_BYTES_MAX 1024U
#endif

static void VENC_LinearAllocatorReset(void)
{
  uint32_t i;

  for (i = 0U; i < venc_linear_block_count; i++)
  {
    if (venc_linear_blocks[i].raw_ptr != NULL)
    {
      free(venc_linear_blocks[i].raw_ptr);
      venc_linear_blocks[i].raw_ptr = NULL;
      venc_linear_blocks[i].alloc_size = 0U;
    }
  }

  venc_linear_block_count = 0U;
  venc_linear_alloc_bytes = 0U;
}

static int VENC_LinearAllocatorAdd(void *raw_ptr, uint32_t alloc_size)
{
  if ((raw_ptr == NULL) || (alloc_size == 0U))
  {
    return -1;
  }

  if (venc_linear_block_count >= VENC_LINEAR_MAX_BLOCKS)
  {
    return -1;
  }

  if ((venc_linear_alloc_bytes + alloc_size) > VENC_ALLOCATOR_BUDGET_SIZE)
  {
    return -1;
  }

  venc_linear_blocks[venc_linear_block_count].raw_ptr = raw_ptr;
  venc_linear_blocks[venc_linear_block_count].alloc_size = alloc_size;
  venc_linear_block_count++;
  venc_linear_alloc_bytes += alloc_size;
  return 0;
}

static void VENC_LinearAllocatorRemove(void *raw_ptr)
{
  uint32_t i;

  if (raw_ptr == NULL)
  {
    return;
  }

  for (i = 0U; i < venc_linear_block_count; i++)
  {
    if (venc_linear_blocks[i].raw_ptr == raw_ptr)
    {
      if (venc_linear_alloc_bytes >= venc_linear_blocks[i].alloc_size)
      {
        venc_linear_alloc_bytes -= venc_linear_blocks[i].alloc_size;
      }
      else
      {
        venc_linear_alloc_bytes = 0U;
      }

      if ((i + 1U) < venc_linear_block_count)
      {
        venc_linear_blocks[i] = venc_linear_blocks[venc_linear_block_count - 1U];
      }
      venc_linear_blocks[venc_linear_block_count - 1U].raw_ptr = NULL;
      venc_linear_blocks[venc_linear_block_count - 1U].alloc_size = 0U;
      venc_linear_block_count--;
      return;
    }
  }
}

struct VENC_Context {
#if APP_ENC_USE_H264
  H264EncInst hdl;
  H264EncConfig cfg;
  H264EncCodingCtrl coding;
  H264EncRateCtrl rate;
  H264EncPreProcessingCfg preproc;
  H264EncPictureType input_type;
  uint32_t width;
  uint32_t height;
  uint32_t time_increment;
  uint32_t frame_count;
  uint8_t stream_started;
  uint8_t start_buf[H264_START_BUF_SIZE];
#else
  JpegEncInst hdl;
  JpegEncCfg cfg;
#endif
};

static struct VENC_Context VENC_Instance;

static const char *VENC_RetStr(int ret)
{
#if APP_ENC_USE_H264
  switch (ret)
  {
  case H264ENC_OK: return "H264ENC_OK";
  case H264ENC_FRAME_READY: return "H264ENC_FRAME_READY";
  case H264ENC_ERROR: return "H264ENC_ERROR";
  case H264ENC_NULL_ARGUMENT: return "H264ENC_NULL_ARGUMENT";
  case H264ENC_INVALID_ARGUMENT: return "H264ENC_INVALID_ARGUMENT";
  case H264ENC_MEMORY_ERROR: return "H264ENC_MEMORY_ERROR";
  case H264ENC_EWL_ERROR: return "H264ENC_EWL_ERROR";
  case H264ENC_EWL_MEMORY_ERROR: return "H264ENC_EWL_MEMORY_ERROR";
  case H264ENC_INVALID_STATUS: return "H264ENC_INVALID_STATUS";
  case H264ENC_OUTPUT_BUFFER_OVERFLOW: return "H264ENC_OUTPUT_BUFFER_OVERFLOW";
  case H264ENC_HW_BUS_ERROR: return "H264ENC_HW_BUS_ERROR";
  case H264ENC_HW_DATA_ERROR: return "H264ENC_HW_DATA_ERROR";
  case H264ENC_HW_TIMEOUT: return "H264ENC_HW_TIMEOUT";
  case H264ENC_HW_RESERVED: return "H264ENC_HW_RESERVED";
  case H264ENC_SYSTEM_ERROR: return "H264ENC_SYSTEM_ERROR";
  case H264ENC_INSTANCE_ERROR: return "H264ENC_INSTANCE_ERROR";
  case H264ENC_HRD_ERROR: return "H264ENC_HRD_ERROR";
  case H264ENC_HW_RESET: return "H264ENC_HW_RESET";
  case H264ENC_FUSE_ERROR: return "H264ENC_FUSE_ERROR";
  default: return "H264ENC_UNKNOWN";
  }
#else
  switch (ret)
  {
  case JPEGENC_OK: return "JPEGENC_OK";
  case JPEGENC_FRAME_READY: return "JPEGENC_FRAME_READY";
  case JPEGENC_RESTART_INTERVAL: return "JPEGENC_RESTART_INTERVAL";
  case JPEGENC_ERROR: return "JPEGENC_ERROR";
  case JPEGENC_NULL_ARGUMENT: return "JPEGENC_NULL_ARGUMENT";
  case JPEGENC_INVALID_ARGUMENT: return "JPEGENC_INVALID_ARGUMENT";
  case JPEGENC_MEMORY_ERROR: return "JPEGENC_MEMORY_ERROR";
  case JPEGENC_INVALID_STATUS: return "JPEGENC_INVALID_STATUS";
  case JPEGENC_OUTPUT_BUFFER_OVERFLOW: return "JPEGENC_OUTPUT_BUFFER_OVERFLOW";
  case JPEGENC_EWL_ERROR: return "JPEGENC_EWL_ERROR";
  case JPEGENC_EWL_MEMORY_ERROR: return "JPEGENC_EWL_MEMORY_ERROR";
  case JPEGENC_HW_BUS_ERROR: return "JPEGENC_HW_BUS_ERROR";
  case JPEGENC_HW_DATA_ERROR: return "JPEGENC_HW_DATA_ERROR";
  case JPEGENC_HW_TIMEOUT: return "JPEGENC_HW_TIMEOUT";
  case JPEGENC_HW_RESERVED: return "JPEGENC_HW_RESERVED";
  case JPEGENC_SYSTEM_ERROR: return "JPEGENC_SYSTEM_ERROR";
  case JPEGENC_INSTANCE_ERROR: return "JPEGENC_INSTANCE_ERROR";
  case JPEGENC_HW_RESET: return "JPEGENC_HW_RESET";
  default: return "JPEGENC_UNKNOWN";
  }
#endif
}

static void VENC_CacheClean(void *buf, size_t len)
{
  uintptr_t src_addr;
  uintptr_t clean_addr;
  uint32_t clean_len;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  src_addr = (uintptr_t)buf;
  clean_addr = src_addr & ~((uintptr_t)31U);
  clean_len = (uint32_t)(((src_addr - clean_addr) + len + 31U) & ~((size_t)31U));
  VENC_CACHE_OP(SCB_CleanDCache_by_Addr((uint8_t *)clean_addr, clean_len));
}

static void VENC_CacheInvalidate(void *buf, size_t len)
{
  uintptr_t src_addr;
  uintptr_t inval_addr;
  uint32_t inval_len;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  src_addr = (uintptr_t)buf;
  inval_addr = src_addr & ~((uintptr_t)31U);
  inval_len = (uint32_t)(((src_addr - inval_addr) + len + 31U) & ~((size_t)31U));
  VENC_CACHE_OP(SCB_InvalidateDCache_by_Addr((uint8_t *)inval_addr, inval_len));
}

#if APP_ENC_USE_H264
static H264EncPictureType VENC_SelectInputType(ENC_InputType_t input_type, const char **name)
{
  switch (input_type)
  {
  case ENC_INPUT_YUV422_YUYV:
    *name = "YUV422_YUYV";
    return H264ENC_YUV422_INTERLEAVED_YUYV;
  case ENC_INPUT_YUV422_UYVY:
    *name = "YUV422_UYVY";
    return H264ENC_YUV422_INTERLEAVED_UYVY;
  case ENC_INPUT_RGB888:
  default:
    *name = "RGB888";
    return H264ENC_RGB888;
  }
}
#else
static JpegEncFrameType VENC_SelectInputType(ENC_InputType_t input_type, const char **name)
{
  switch (input_type)
  {
  case ENC_INPUT_YUV422_YUYV:
    *name = "YUV422_YUYV";
    return JPEGENC_YUV422_INTERLEAVED_YUYV;
  case ENC_INPUT_YUV422_UYVY:
    *name = "YUV422_UYVY";
    return JPEGENC_YUV422_INTERLEAVED_UYVY;
  case ENC_INPUT_RGB888:
  default:
    *name = "RGB888";
    return JPEGENC_RGB888;
  }
}
#endif

#if !APP_ENC_USE_H264
static JpegEncCodingMode VENC_SelectCodingMode(ENC_InputType_t input_type)
{
  switch (input_type)
  {
  case ENC_INPUT_YUV422_YUYV:
  case ENC_INPUT_YUV422_UYVY:
    return JPEGENC_422_MODE;
  case ENC_INPUT_RGB888:
  default:
    return JPEGENC_420_MODE;
  }
}
#endif

static size_t VENC_GetInputFrameBytes(const struct VENC_Context *p_ctx)
{
#if APP_ENC_USE_H264
  size_t width;
  size_t height;

  if (p_ctx == NULL)
  {
    return 0U;
  }

  width = (size_t)p_ctx->width;
  height = (size_t)p_ctx->height;

  switch (p_ctx->input_type)
  {
  case H264ENC_YUV422_INTERLEAVED_YUYV:
  case H264ENC_YUV422_INTERLEAVED_UYVY:
    return width * height * 2U;
  case H264ENC_RGB888:
  case H264ENC_BGR888:
    return width * height * 3U;
  default:
    return 0U;
  }
#else
  size_t width;
  size_t height;

  if (p_ctx == NULL)
  {
    return 0U;
  }

  width = (size_t)p_ctx->cfg.inputWidth;
  height = (size_t)p_ctx->cfg.inputHeight;

  switch (p_ctx->cfg.frameType)
  {
  case JPEGENC_YUV422_INTERLEAVED_YUYV:
  case JPEGENC_YUV422_INTERLEAVED_UYVY:
    return width * height * 2U;
  case JPEGENC_RGB888:
  case JPEGENC_BGR888:
    return width * height * 3U;
  default:
    return 0U;
  }
#endif
}

void ENC_Init(ENC_Conf_t *p_conf)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  const char *input_name;
#if APP_ENC_USE_H264
  H264EncIn start_in;
  H264EncOut start_out;
#else
  const char *coding_name;
  JpegEncCodingMode coding_mode;
#endif
  uint32_t t0_ms;
  int ret;

  VENC_LinearAllocatorReset();
  venc_linear_alloc_count = 0;
  venc_init_failed_logged = 0;
  venc_encode_ok_count = 0U;
  venc_encode_fail_count = 0U;
  venc_encode_last_log_ms = 0U;
  venc_copy_out_log_ms = 0U;
  p_ctx->hdl = NULL;
  memset(p_ctx, 0, sizeof(*p_ctx));
#if APP_ENC_USE_H264
  p_ctx->input_type = VENC_SelectInputType(p_conf->input_type, &input_name);
  p_ctx->width = (uint32_t)p_conf->width;
  p_ctx->height = (uint32_t)p_conf->height;
  p_ctx->time_increment = 1U;
  p_ctx->frame_count = 0U;
  p_ctx->stream_started = 0U;

  p_ctx->cfg.streamType = H264ENC_BYTE_STREAM;
  p_ctx->cfg.viewMode = H264ENC_BASE_VIEW_DOUBLE_BUFFER;
  p_ctx->cfg.level = H264ENC_LEVEL_3_1;
  p_ctx->cfg.width = p_ctx->width;
  p_ctx->cfg.height = p_ctx->height;
  p_ctx->cfg.frameRateNum = (u32)((p_conf->fps > 0) ? p_conf->fps : 30);
  p_ctx->cfg.frameRateDenom = 1U;
  p_ctx->cfg.scaledWidth = 0U;
  p_ctx->cfg.scaledHeight = 0U;
  p_ctx->cfg.refFrameAmount = 1U;
  p_ctx->cfg.refFrameCompress = 0U;
  p_ctx->cfg.rfcLumBufLimit = 0U;
  p_ctx->cfg.rfcChrBufLimit = 0U;
  p_ctx->cfg.svctLevel = 0U;
  p_ctx->cfg.enableSvctPrefix = 0U;

  ENC_ERR_LOG("[ENC] cfg(H264) input=%s in=%lux%lu fps=%lu bitrate=%u\r\n",
              input_name,
              (unsigned long)p_ctx->cfg.width,
              (unsigned long)p_ctx->cfg.height,
              (unsigned long)p_ctx->cfg.frameRateNum,
              (unsigned int)H264_TARGET_BITRATE_BPS);

  ENC_ERR_LOG("==== ENC allocator: HEAP/AXI (NO_PSRAM) budget=%u max_blocks=%u ====\r\n",
              (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE,
              (unsigned int)VENC_LINEAR_MAX_BLOCKS);
  t0_ms = HAL_GetTick();
  ret = H264EncInit(&p_ctx->cfg, &p_ctx->hdl);
  ENC_INFO_LOG("[ENC] H264EncInit ret=%d (%s) dt_ms=%lu hdl=0x%08lX\r\n",
               ret, VENC_RetStr(ret),
               (unsigned long)(HAL_GetTick() - t0_ms),
               (unsigned long)p_ctx->hdl);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncInit failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    p_ctx->hdl = NULL;
    return;
  }

  ret = H264EncGetCodingCtrl(p_ctx->hdl, &p_ctx->coding);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncGetCodingCtrl failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }
  p_ctx->coding.sliceSize = 0U;
  p_ctx->coding.seiMessages = 0U;
  p_ctx->coding.idrHeader = 1U;
  p_ctx->coding.enableCabac = 0U;
  p_ctx->coding.cabacInitIdc = 0U;
  p_ctx->coding.transform8x8Mode = 0U;
  p_ctx->coding.inputLineBufEn = 0U;
  p_ctx->coding.inputLineBufLoopBackEn = 0U;
  p_ctx->coding.inputLineBufDepth = 0U;
  p_ctx->coding.inputLineBufHwModeEn = 0U;
  ret = H264EncSetCodingCtrl(p_ctx->hdl, &p_ctx->coding);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncSetCodingCtrl failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }

  ret = H264EncGetRateCtrl(p_ctx->hdl, &p_ctx->rate);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncGetRateCtrl failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }
  p_ctx->rate.pictureRc = 1U;
  p_ctx->rate.mbRc = 1U;
  p_ctx->rate.pictureSkip = 0U;
  p_ctx->rate.qpHdr = H264_QP_HDR_DEFAULT;
  p_ctx->rate.qpMin = H264_QP_MIN_DEFAULT;
  p_ctx->rate.qpMax = H264_QP_MAX_DEFAULT;
  p_ctx->rate.bitPerSecond = H264_TARGET_BITRATE_BPS;
  p_ctx->rate.hrd = 0U;
  p_ctx->rate.gopLen = (u32)((p_conf->fps > 0) ? p_conf->fps : 30);
  ret = H264EncSetRateCtrl(p_ctx->hdl, &p_ctx->rate);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncSetRateCtrl failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }

  ret = H264EncGetPreProcessing(p_ctx->hdl, &p_ctx->preproc);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncGetPreProcessing failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }
  p_ctx->preproc.origWidth = p_ctx->width;
  p_ctx->preproc.origHeight = p_ctx->height;
  p_ctx->preproc.xOffset = 0U;
  p_ctx->preproc.yOffset = 0U;
  p_ctx->preproc.inputType = p_ctx->input_type;
  p_ctx->preproc.rotation = H264ENC_ROTATE_0;
  p_ctx->preproc.videoStabilization = 0U;
  p_ctx->preproc.colorConversion.type = H264ENC_RGBTOYUV_BT601;
  p_ctx->preproc.scaledOutput = 0U;
  p_ctx->preproc.interlacedFrame = 0U;
  ret = H264EncSetPreProcessing(p_ctx->hdl, &p_ctx->preproc);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncSetPreProcessing failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }

  memset(&start_in, 0, sizeof(start_in));
  memset(&start_out, 0, sizeof(start_out));
  start_in.pOutBuf = (u32 *)p_ctx->start_buf;
  start_in.busOutBuf = (size_t)p_ctx->start_buf;
  start_in.outBufSize = (u32)sizeof(p_ctx->start_buf);
  VENC_CacheClean((void *)start_in.pOutBuf, (size_t)start_in.outBufSize);
  VENC_CacheInvalidate((void *)start_in.pOutBuf, (size_t)start_in.outBufSize);
  __DSB();
  __ISB();
  ret = H264EncStrmStart(p_ctx->hdl, &start_in, &start_out);
  if (ret != H264ENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] H264EncStrmStart failed ret=%d (%s)\r\n", ret, VENC_RetStr(ret));
    (void)H264EncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }
  p_ctx->stream_started = 1U;
  ENC_ERR_LOG("==== ENC MODE: H264 stream started (sps_pps=%lu bytes) ====\r\n",
              (unsigned long)start_out.streamSize);
#else
  memset(&p_ctx->cfg, 0, sizeof(p_ctx->cfg));
  p_ctx->cfg.inputWidth = (u32)p_conf->width;
  p_ctx->cfg.inputHeight = (u32)p_conf->height;
  p_ctx->cfg.xOffset = 0;
  p_ctx->cfg.yOffset = 0;
  p_ctx->cfg.codingWidth = (u32)p_conf->width;
  p_ctx->cfg.codingHeight = (u32)p_conf->height;
  p_ctx->cfg.restartInterval = 0;
  p_ctx->cfg.qLevel = JPEG_QUALITY_LEVEL;
  p_ctx->cfg.qTableLuma = NULL;
  p_ctx->cfg.qTableChroma = NULL;
  p_ctx->cfg.frameType = VENC_SelectInputType(p_conf->input_type, &input_name);
  p_ctx->cfg.colorConversion.type = JPEGENC_RGBTOYUV_BT601;
  p_ctx->cfg.rotation = JPEGENC_ROTATE_0;
  p_ctx->cfg.codingType = JPEGENC_WHOLE_FRAME;
  coding_mode = VENC_SelectCodingMode(p_conf->input_type);
  p_ctx->cfg.codingMode = coding_mode;
  coding_name = (coding_mode == JPEGENC_422_MODE) ? "422" : "420";
  p_ctx->cfg.unitsType = JPEGENC_NO_UNITS;
  p_ctx->cfg.markerType = JPEGENC_SINGLE_MARKER;
  p_ctx->cfg.xDensity = 1;
  p_ctx->cfg.yDensity = 1;
  p_ctx->cfg.comLength = 0;
  p_ctx->cfg.pCom = NULL;
  p_ctx->cfg.inputLineBufEn = 0;
  p_ctx->cfg.inputLineBufLoopBackEn = 0;
  p_ctx->cfg.inputLineBufDepth = 0;
  p_ctx->cfg.inputLineBufHwModeEn = 0;

  ENC_ERR_LOG("[ENC] cfg input=%s frameType=%d codingMode=%s(%d) wh=%lux%lu q=%d\r\n",
              input_name,
              (int)p_ctx->cfg.frameType,
              coding_name,
              (int)p_ctx->cfg.codingMode,
              (unsigned long)p_ctx->cfg.inputWidth,
              (unsigned long)p_ctx->cfg.inputHeight,
              (int)p_ctx->cfg.qLevel);

  ENC_INFO_LOG("[ENC] init begin cfg=%dx%d@%d input=%s q=%d alloc=heap_no_psram budget=%u blocks=%u\r\n",
               p_conf->width, p_conf->height, p_conf->fps, input_name,
               JPEG_QUALITY_LEVEL,
               (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE,
               (unsigned int)VENC_LINEAR_MAX_BLOCKS);
  ENC_ERR_LOG("==== ENC allocator: HEAP/AXI (NO_PSRAM) budget=%u max_blocks=%u ====\r\n",
              (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE,
              (unsigned int)VENC_LINEAR_MAX_BLOCKS);
  t0_ms = HAL_GetTick();
  ENC_INFO_LOG("[ENC] call JpegEncInit cfg_ptr=0x%08lX\r\n", (unsigned long)&p_ctx->cfg);
  ret = JpegEncInit(&p_ctx->cfg, &p_ctx->hdl);
  ENC_INFO_LOG("[ENC] JpegEncInit ret=%d (%s) dt_ms=%lu hdl=0x%08lX\r\n",
               ret, VENC_RetStr(ret),
               (unsigned long)(HAL_GetTick() - t0_ms),
               (unsigned long)p_ctx->hdl);
  if (ret != JPEGENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] JpegEncInit failed ret=%d (%s) cfg=%dx%d input=%s q=%d\r\n",
                ret, VENC_RetStr(ret),
                p_conf->width, p_conf->height, input_name, JPEG_QUALITY_LEVEL);
    p_ctx->hdl = NULL;
    return;
  }

  ret = JpegEncSetPictureSize(p_ctx->hdl, &p_ctx->cfg);
  ENC_INFO_LOG("[ENC] JpegEncSetPictureSize ret=%d (%s) in=%lux%lu code=%lux%lu\r\n",
               ret, VENC_RetStr(ret),
               (unsigned long)p_ctx->cfg.inputWidth,
               (unsigned long)p_ctx->cfg.inputHeight,
               (unsigned long)p_ctx->cfg.codingWidth,
               (unsigned long)p_ctx->cfg.codingHeight);
  if (ret != JPEGENC_OK)
  {
    ENC_ERR_LOG("[ENC][ERR] JpegEncSetPictureSize failed ret=%d (%s)\r\n",
                ret, VENC_RetStr(ret));
    (void)JpegEncRelease(p_ctx->hdl);
    p_ctx->hdl = NULL;
    return;
  }
#endif
}

void ENC_DeInit(void)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  int ret;

  if (p_ctx->hdl == NULL)
  {
    return;
  }

#if APP_ENC_USE_H264
  ret = H264EncRelease(p_ctx->hdl);
  assert(ret == H264ENC_OK);
#else
  ret = JpegEncRelease(p_ctx->hdl);
  assert(ret == JPEGENC_OK);
#endif
  p_ctx->hdl = NULL;
  VENC_LinearAllocatorReset();
}

int ENC_EncodeFrame(uint8_t *p_in, uint8_t *p_out, size_t out_len, int is_intra_force)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
#if APP_ENC_USE_H264
  H264EncOut enc_out;
  H264EncIn enc_in;
#else
  JpegEncOut enc_out;
  JpegEncIn enc_in;
#endif
  int ret;
  uint32_t now_ms;
  size_t in_frame_bytes;
#if !APP_ENC_USE_H264
  size_t header_flush_bytes;
  uint8_t h0 = 0U, h1 = 0U, h2 = 0U, h3 = 0U;
  uint8_t t0 = 0U, t1 = 0U, t2 = 0U, t3 = 0U;
  int has_soi = 0;
  int has_eoi = 0;
#endif

  venc_debug_frame_id++;

  if (p_ctx->hdl == NULL)
  {
    if (venc_init_failed_logged == 0)
    {
      venc_init_failed_logged = 1;
      ENC_ERR_LOG("[ENC][ERR] encode requested while encoder is not initialized\r\n");
    }
    return -1;
  }

  if ((p_out == NULL) || (out_len == 0U))
  {
    return -1;
  }
  if (p_in == NULL)
  {
    return -1;
  }

  memset(&enc_in, 0, sizeof(enc_in));
  memset(&enc_out, 0, sizeof(enc_out));

#if APP_ENC_USE_H264
  enc_in.busLuma = (ptr_t)p_in;
  enc_in.busChromaU = 0U;
  enc_in.busChromaV = 0U;
  enc_in.timeIncrement = p_ctx->time_increment;
  enc_in.pOutBuf = (u32 *)p_out;
  enc_in.busOutBuf = (size_t)p_out;
  enc_in.outBufSize = (u32)out_len;
  enc_in.codingType = (is_intra_force || (p_ctx->frame_count == 0U)) ? H264ENC_INTRA_FRAME : H264ENC_PREDICTED_FRAME;
  enc_in.busLumaStab = 0U;
  enc_in.ipf = H264ENC_REFERENCE_AND_REFRESH;
  enc_in.ltrf = H264ENC_NO_REFERENCE_NO_REFRESH;
  enc_in.lineBufWrCnt = 0U;
  enc_in.sendAUD = 0U;
#else
  enc_in.frameHeader = 1;
  enc_in.busLum = (size_t)p_in;
  enc_in.pLum = p_in;
  enc_in.pOutBuf = p_out;
  enc_in.busOutBuf = (size_t)p_out;
  enc_in.outBufSize = (u32)out_len;
  enc_in.lineBufWrCnt = 0;
#endif

  /* Make sure HW sees latest source frame and starts from clean output memory. */
  in_frame_bytes = VENC_GetInputFrameBytes(p_ctx);
  if (in_frame_bytes > 0U)
  {
    VENC_CacheClean((void *)p_in, in_frame_bytes);
  }
  VENC_CacheClean((void *)enc_in.pOutBuf, (size_t)enc_in.outBufSize);
  VENC_CacheInvalidate((void *)enc_in.pOutBuf, (size_t)enc_in.outBufSize);
  __DSB();
  __ISB();

#if APP_ENC_USE_H264
  ret = H264EncStrmEncode(p_ctx->hdl, &enc_in, &enc_out, NULL, NULL, NULL);
  if (ret != H264ENC_FRAME_READY)
  {
    venc_encode_fail_count++;
    now_ms = HAL_GetTick();
    if ((now_ms - venc_encode_last_log_ms) >= 1000U)
    {
      ENC_INFO_LOG("[ENC][RUN] ret=%d (%s) h264=%lu ok=%lu fail=%lu coding=%d out=%lu\r\n",
                   ret,
                   VENC_RetStr(ret),
                   (unsigned long)enc_out.streamSize,
                   (unsigned long)venc_encode_ok_count,
                   (unsigned long)venc_encode_fail_count,
                   (int)enc_in.codingType,
                   (unsigned long)enc_in.outBufSize);
      venc_encode_last_log_ms = now_ms;
    }
    ENC_DBG_LOG("[ENC][DBG][RET] id=%lu ret=%d(%s) h264=%lu out=%lu in_ptr=0x%08lX out_ptr=0x%08lX\r\n",
                (unsigned long)venc_debug_frame_id,
                ret,
                VENC_RetStr(ret),
                (unsigned long)enc_out.streamSize,
                (unsigned long)enc_in.outBufSize,
                (unsigned long)enc_in.busLuma,
                (unsigned long)enc_in.pOutBuf);
    return -1;
  }

  venc_encode_ok_count++;
  p_ctx->frame_count++;
  now_ms = HAL_GetTick();
  if ((now_ms - venc_encode_last_log_ms) >= 1000U)
  {
    ENC_INFO_LOG("[ENC][RUN] ret=%d (%s) h264=%lu ok=%lu fail=%lu coding=%d out=%lu\r\n",
                 ret,
                 VENC_RetStr(ret),
                 (unsigned long)enc_out.streamSize,
                 (unsigned long)venc_encode_ok_count,
                 (unsigned long)venc_encode_fail_count,
                 (int)enc_in.codingType,
                 (unsigned long)enc_in.outBufSize);
    venc_encode_last_log_ms = now_ms;
  }

  if ((size_t)enc_out.streamSize > out_len)
  {
    venc_encode_fail_count++;
    if ((now_ms - venc_encode_last_log_ms) >= 1000U)
    {
      ENC_ERR_LOG("[ENC][ERR] output copy overflow h264=%lu out_len=%lu\r\n",
                  (unsigned long)enc_out.streamSize,
                  (unsigned long)out_len);
      venc_encode_last_log_ms = now_ms;
    }
    return -1;
  }

  VENC_CacheInvalidate(p_out, (size_t)enc_out.streamSize);
  __DSB();
  __ISB();

  if ((now_ms - venc_copy_out_log_ms) >= 1000U)
  {
    ENC_INFO_LOG("[ENC][OUT] out_va=0x%08lX out_bus=0x%08lX h264=%lu out=%lu storage=AXI(no-PSRAM)\r\n",
                 (unsigned long)p_out,
                 (unsigned long)p_out,
                 (unsigned long)enc_out.streamSize,
                 (unsigned long)out_len);
    venc_copy_out_log_ms = now_ms;
  }

  return (int)enc_out.streamSize;
#else
  ret = JpegEncEncode(p_ctx->hdl, &enc_in, &enc_out, NULL, NULL);
  if (ret != JPEGENC_FRAME_READY)
  {
    venc_encode_fail_count++;
    now_ms = HAL_GetTick();
    if ((now_ms - venc_encode_last_log_ms) >= 1000U)
    {
      ENC_INFO_LOG("[ENC][RUN] ret=%d (%s) jfif=%lu ok=%lu fail=%lu frame_header=%lu out=%lu\r\n",
                   ret,
                   VENC_RetStr(ret),
                   (unsigned long)enc_out.jfifSize,
                   (unsigned long)venc_encode_ok_count,
                   (unsigned long)venc_encode_fail_count,
                   (unsigned long)enc_in.frameHeader,
                   (unsigned long)enc_in.outBufSize);
      venc_encode_last_log_ms = now_ms;
    }
    ENC_DBG_LOG("[ENC][DBG][RET] id=%lu ret=%d(%s) jfif=%lu out=%lu in_ptr=0x%08lX out_ptr=0x%08lX\r\n",
                (unsigned long)venc_debug_frame_id,
                ret,
                VENC_RetStr(ret),
                (unsigned long)enc_out.jfifSize,
                (unsigned long)enc_in.outBufSize,
                (unsigned long)enc_in.pLum,
                (unsigned long)enc_in.pOutBuf);
    return -1;
  }

  venc_encode_ok_count++;
  now_ms = HAL_GetTick();
  if ((now_ms - venc_encode_last_log_ms) >= 1000U)
  {
    ENC_INFO_LOG("[ENC][RUN] ret=%d (%s) jfif=%lu ok=%lu fail=%lu frame_header=%lu out=%lu\r\n",
                 ret,
                 VENC_RetStr(ret),
                 (unsigned long)enc_out.jfifSize,
                 (unsigned long)venc_encode_ok_count,
                 (unsigned long)venc_encode_fail_count,
                 (unsigned long)enc_in.frameHeader,
                 (unsigned long)enc_in.outBufSize);
    venc_encode_last_log_ms = now_ms;
  }

  if ((size_t)enc_out.jfifSize > out_len)
  {
    venc_encode_fail_count++;
    if ((now_ms - venc_encode_last_log_ms) >= 1000U)
    {
      ENC_ERR_LOG("[ENC][ERR] output copy overflow jfif=%lu out_len=%lu\r\n",
                  (unsigned long)enc_out.jfifSize,
                  (unsigned long)out_len);
      venc_encode_last_log_ms = now_ms;
    }
    return -1;
  }

  /* JPEG headers are generated by CPU path; flush that part only, then refetch stream. */
  header_flush_bytes = (size_t)enc_out.jfifSize;
  if (header_flush_bytes > (size_t)VENC_CPU_HEADER_FLUSH_BYTES_MAX)
  {
    header_flush_bytes = (size_t)VENC_CPU_HEADER_FLUSH_BYTES_MAX;
  }
  VENC_CacheClean(p_out, header_flush_bytes);
  VENC_CacheInvalidate(p_out, (size_t)enc_out.jfifSize);
  __DSB();
  __ISB();

  if (enc_out.jfifSize >= 4U)
  {
    h0 = p_out[0];
    h1 = p_out[1];
    h2 = p_out[2];
    h3 = p_out[3];
    t0 = p_out[enc_out.jfifSize - 4U];
    t1 = p_out[enc_out.jfifSize - 3U];
    t2 = p_out[enc_out.jfifSize - 2U];
    t3 = p_out[enc_out.jfifSize - 1U];
    has_soi = ((h0 == 0xFFU) && (h1 == 0xD8U)) ? 1 : 0;
    has_eoi = ((t2 == 0xFFU) && (t3 == 0xD9U)) ? 1 : 0;
  }

  ENC_DBG_LOG("[ENC][DBG][OUT] id=%lu jfif=%lu soi=%d eoi=%d in_ptr=0x%08lX out_ptr=0x%08lX head=%02X %02X %02X %02X tail=%02X %02X %02X %02X\r\n",
              (unsigned long)venc_debug_frame_id,
              (unsigned long)enc_out.jfifSize,
              has_soi,
              has_eoi,
              (unsigned long)enc_in.pLum,
              (unsigned long)enc_in.pOutBuf,
              h0, h1, h2, h3,
              t0, t1, t2, t3);

  if ((now_ms - venc_copy_out_log_ms) >= 1000U)
  {
    ENC_INFO_LOG("[ENC][OUT] out_va=0x%08lX out_bus=0x%08lX jfif=%lu out=%lu storage=AXI(no-PSRAM)\r\n",
                 (unsigned long)p_out,
                 (unsigned long)p_out,
                 (unsigned long)enc_out.jfifSize,
                 (unsigned long)out_len);
    venc_copy_out_log_ms = now_ms;
  }

  return (int)enc_out.jfifSize;
#endif
}

void *EWLmalloc(u32 n)
{
  void *res = malloc(n);
  if (res == NULL)
  {
    ENC_ERR_LOG("[ENC][ERR] EWLmalloc failed size=%lu\r\n", (unsigned long)n);
  }

  return res;
}

void EWLfree(void *p)
{
  free(p);
}

void *EWLcalloc(u32 n, u32 s)
{
  void *res = calloc(n, s);
  if (res == NULL)
  {
    ENC_ERR_LOG("[ENC][ERR] EWLcalloc failed n=%lu size=%lu\r\n",
                (unsigned long)n, (unsigned long)s);
  }

  return res;
}

/* Implement EWLMallocLinear via heap-backed aligned blocks (no PSRAM static pool). */
i32 EWLMallocLinear(const void *instance, u32 size, EWLLinearMem_t *info)
{
  void *raw_ptr;
  uintptr_t raw_addr;
  uintptr_t alloc_addr;
  uint32_t alloc_size;
  size_t raw_size;

  (void)instance;
  if (info == NULL)
  {
    return -1;
  }

  alloc_size = (uint32_t)((size + (VENC_LINEAR_ALIGN_BYTES - 1U)) & ~((u32)(VENC_LINEAR_ALIGN_BYTES - 1U)));
  venc_linear_alloc_count++;
  ENC_INFO_LOG("[ENC][ALLOC] #%lu req=%lu align=%lu used=%lu/%u blocks=%lu/%u\r\n",
               (unsigned long)venc_linear_alloc_count,
               (unsigned long)size,
               (unsigned long)alloc_size,
               (unsigned long)venc_linear_alloc_bytes,
               (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE,
               (unsigned long)venc_linear_block_count,
               (unsigned int)VENC_LINEAR_MAX_BLOCKS);
  if ((venc_linear_block_count >= VENC_LINEAR_MAX_BLOCKS) ||
      ((venc_linear_alloc_bytes + alloc_size) > VENC_ALLOCATOR_BUDGET_SIZE))
  {
    ENC_ERR_LOG("[ENC][ERR] EWLMallocLinear budget/block exceeded req=%lu aligned=%lu used=%lu/%u blocks=%lu/%u\r\n",
                (unsigned long)size,
                (unsigned long)alloc_size,
                (unsigned long)venc_linear_alloc_bytes,
                (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE,
                (unsigned long)venc_linear_block_count,
                (unsigned int)VENC_LINEAR_MAX_BLOCKS);
    return -1;
  }

  raw_size = (size_t)alloc_size + (size_t)VENC_LINEAR_ALIGN_BYTES + sizeof(void *);
  raw_ptr = malloc(raw_size);
  if (raw_ptr == NULL)
  {
    ENC_ERR_LOG("[ENC][ERR] EWLMallocLinear malloc failed req=%lu aligned=%lu raw_size=%lu\r\n",
                (unsigned long)size,
                (unsigned long)alloc_size,
                (unsigned long)raw_size);
    return -1;
  }

  raw_addr = (uintptr_t)raw_ptr;
  alloc_addr = (raw_addr + sizeof(void *) + (uintptr_t)(VENC_LINEAR_ALIGN_BYTES - 1U)) &
               ~((uintptr_t)(VENC_LINEAR_ALIGN_BYTES - 1U));
  ((void **)alloc_addr)[-1] = raw_ptr;
  if (VENC_LinearAllocatorAdd(raw_ptr, alloc_size) != 0)
  {
    free(raw_ptr);
    ENC_ERR_LOG("[ENC][ERR] EWLMallocLinear allocator add failed req=%lu aligned=%lu\r\n",
                (unsigned long)size,
                (unsigned long)alloc_size);
    return -1;
  }

  memset((void *)alloc_addr, 0, alloc_size);
  ENC_INFO_LOG("[ENC][ALLOC] #%lu req=%lu align=%lu used=%lu/%u\r\n",
               (unsigned long)venc_linear_alloc_count,
               (unsigned long)size,
               (unsigned long)alloc_size,
               (unsigned long)venc_linear_alloc_bytes,
               (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE);

  info->size = size;
  info->virtualAddress = (u32 *)alloc_addr;
  info->busAddress = (ptr_t)info->virtualAddress;
  ENC_INFO_LOG("[ENC][ALLOC] #%lu ok va=0x%08lX bus=0x%08lX next_used=%lu/%u\r\n",
               (unsigned long)venc_linear_alloc_count,
               (unsigned long)info->virtualAddress,
               (unsigned long)info->busAddress,
               (unsigned long)venc_linear_alloc_bytes,
               (unsigned int)VENC_ALLOCATOR_BUDGET_SIZE);

  return 0;
}

void EWLFreeLinear(const void *instance, EWLLinearMem_t *info)
{
  void *raw_ptr;

  (void)instance;
  if ((info == NULL) || (info->virtualAddress == NULL))
  {
    return;
  }

  raw_ptr = ((void **)info->virtualAddress)[-1];
  if (raw_ptr != NULL)
  {
    VENC_LinearAllocatorRemove(raw_ptr);
    free(raw_ptr);
  }

  info->virtualAddress = NULL;
  info->busAddress = (ptr_t)0;
  info->size = 0U;
}
