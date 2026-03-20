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

#include "jpegencapi.h"
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "ewl.h"
#include "app_config.h"

#define APP_ENC_INFO_LOG_ENABLE 0
#define APP_ENC_ERROR_LOG_ENABLE 1

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

#define VENC_ALLOCATOR_SIZE (8 * 1024 * 1024)
/* 0 = strongest compression, 10 = highest quality (larger output). */
#define JPEG_QUALITY_LEVEL 0

static uint8_t venc_hw_allocator_buffer[VENC_ALLOCATOR_SIZE] ALIGN_32 IN_PSRAM;
static uint8_t *venc_hw_allocator_pos = venc_hw_allocator_buffer;
static int venc_init_failed_logged;
static uint32_t venc_linear_alloc_count;
static uint32_t venc_encode_ok_count;
static uint32_t venc_encode_fail_count;
static uint32_t venc_encode_last_log_ms;
static uint32_t venc_copy_out_log_ms;
/* JFIF headers are CPU-written; entropy payload is HW-written. */
#define VENC_CPU_HEADER_FLUSH_BYTES_MAX 1024U

struct VENC_Context {
  JpegEncInst hdl;
  JpegEncCfg cfg;
};

static struct VENC_Context VENC_Instance;

static const char *VENC_RetStr(int ret)
{
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

static size_t VENC_GetInputFrameBytes(const struct VENC_Context *p_ctx)
{
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
}

void ENC_Init(ENC_Conf_t *p_conf)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  const char *input_name;
  const char *coding_name;
  JpegEncCodingMode coding_mode;
  uint32_t t0_ms;
  int ret;

  venc_hw_allocator_pos = venc_hw_allocator_buffer;
  venc_linear_alloc_count = 0;
  venc_init_failed_logged = 0;
  venc_encode_ok_count = 0U;
  venc_encode_fail_count = 0U;
  venc_encode_last_log_ms = 0U;
  venc_copy_out_log_ms = 0U;
  p_ctx->hdl = NULL;
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

  ENC_INFO_LOG("[ENC] init begin cfg=%dx%d@%d input=%s q=%d alloc=%u\r\n",
               p_conf->width, p_conf->height, p_conf->fps, input_name,
               JPEG_QUALITY_LEVEL, (unsigned int)VENC_ALLOCATOR_SIZE);
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
}

void ENC_DeInit(void)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  int ret;

  if (p_ctx->hdl == NULL)
  {
    return;
  }

  ret = JpegEncRelease(p_ctx->hdl);
  assert(ret == JPEGENC_OK);
  p_ctx->hdl = NULL;
}

int ENC_EncodeFrame(uint8_t *p_in, uint8_t *p_out, size_t out_len, int is_intra_force)
{
  struct VENC_Context *p_ctx = &VENC_Instance;
  JpegEncOut enc_out;
  JpegEncIn enc_in;
  int ret;
  uint32_t now_ms;
  size_t in_frame_bytes;
  size_t header_flush_bytes;

  (void)is_intra_force;

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

  memset(&enc_in, 0, sizeof(enc_in));
  memset(&enc_out, 0, sizeof(enc_out));

  enc_in.frameHeader = 1;
  enc_in.busLum = (size_t)p_in;
  enc_in.pLum = p_in;
  enc_in.pOutBuf = p_out;
  enc_in.busOutBuf = (size_t)p_out;
  enc_in.outBufSize = (u32)out_len;
  enc_in.lineBufWrCnt = 0;

  /* Make sure HW sees latest source frame and starts from clean output memory. */
  in_frame_bytes = VENC_GetInputFrameBytes(p_ctx);
  if (in_frame_bytes > 0U)
  {
    VENC_CacheClean((void *)enc_in.pLum, in_frame_bytes);
  }
  VENC_CacheClean((void *)enc_in.pOutBuf, (size_t)enc_in.outBufSize);
  VENC_CacheInvalidate((void *)enc_in.pOutBuf, (size_t)enc_in.outBufSize);
  __DSB();
  __ISB();

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

/* Implement simple EWLMallocLinear. No dealloc supported */
i32 EWLMallocLinear(const void *instance, u32 size, EWLLinearMem_t *info)
{
  uintptr_t pos = (uintptr_t)venc_hw_allocator_pos;
  uintptr_t base = (uintptr_t)venc_hw_allocator_buffer;
  uintptr_t end = base + (uintptr_t)VENC_ALLOCATOR_SIZE;
  uintptr_t alloc_addr;
  uintptr_t next_pos;
  uint32_t alloc_size;

  (void)instance;
  if (info == NULL)
  {
    return -1;
  }

  alloc_addr = (pos + 63U) & ~((uintptr_t)63U);
  alloc_size = (uint32_t)((size + 63U) & ~((u32)63U));
  next_pos = alloc_addr + (uintptr_t)alloc_size;
  venc_linear_alloc_count++;
  ENC_INFO_LOG("[ENC][ALLOC] #%lu req=%lu align=%lu used=%lu/%u\r\n",
               (unsigned long)venc_linear_alloc_count,
               (unsigned long)size,
               (unsigned long)alloc_size,
               (unsigned long)(pos - base),
               (unsigned int)VENC_ALLOCATOR_SIZE);
  if (next_pos > end)
  {
    ENC_ERR_LOG("[ENC][ERR] EWLMallocLinear OOM req=%lu aligned=%lu used=%lu cap=%u\r\n",
                (unsigned long)size,
                (unsigned long)alloc_size,
                (unsigned long)(pos - base),
                (unsigned int)VENC_ALLOCATOR_SIZE);
    return -1;
  }

  info->size = size;
  info->virtualAddress = (u32 *)alloc_addr;
  info->busAddress = (ptr_t)info->virtualAddress;
  venc_hw_allocator_pos = (uint8_t *)next_pos;
  ENC_INFO_LOG("[ENC][ALLOC] #%lu ok va=0x%08lX bus=0x%08lX next_used=%lu/%u\r\n",
               (unsigned long)venc_linear_alloc_count,
               (unsigned long)info->virtualAddress,
               (unsigned long)info->busAddress,
               (unsigned long)(next_pos - base),
               (unsigned int)VENC_ALLOCATOR_SIZE);

  return 0;
}
