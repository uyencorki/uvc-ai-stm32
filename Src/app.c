 /**
 ******************************************************************************
 * @file    app.c
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

#include "app.h"

#include <stdint.h>
#include <math.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dma.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_xspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app_enc.h"
#include "utils.h"
#include "uvcl.h"
#include "draw.h"
#include "stai.h"
#include "stai_network.h"
/* VideoEncoder tables use legacy bool/true/false definitions from basetype.h. */
#ifdef bool
#undef bool
#endif
#ifdef true
#undef true
#endif
#ifdef false
#undef false
#endif
#include "../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/source/jpeg/EncJpegDhtTables.h"
#include "../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/source/jpeg/EncJpegQuantTables.h"

#include "figs.h"

/*
 * Link-compat fallback:
 * HAL_XSPI_Abort() references HAL_DMA_Abort(), but some project variants do
 * not compile stm32n6xx_hal_dma.c. Keep a weak fallback here so link succeeds.
 * If real HAL DMA module is linked later, its strong symbol overrides this one.
 */
__WEAK HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *const hdma)
{
  UNUSED(hdma);
  return HAL_OK;
}

#ifndef APP_VERSION_STRING
#define APP_VERSION_STRING "dev"
#endif

#define FREERTOS_PRIORITY(p) ((UBaseType_t)((int)tskIDLE_PRIORITY + configMAX_PRIORITIES / 2 + (p)))

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

#define DBG_INFO_FONT font_12
#define CONF_LEVEL_FONT font_16
#define INF_INFO_FONT font_16
#define OBJ_RECT_COLOR 0xffffffff

#define BQUEUE_MAX_BUFFERS 2
#define CPU_LOAD_HISTORY_DEPTH 8

#define CAPTURE_BUFFER_NB (CAPTURE_DELAY + 2)

/* Lightweight frame dump: ISR only sets a flag, print happens in thread context. */
#define APP_PIPE1_FRAME_DUMP_ENABLE 0
#define APP_PIPE1_FRAME_DUMP_BYTES 20
#define APP_PIPE1_FRAME_DUMP_EVERY_FRAME 1
#define APP_PIPE1_FRAME_DUMP_PERIOD_MS 5000U
#define APP_CAM_HEALTH_LOG_ENABLE 0
#define APP_DCMIPP_ERR_LOG_ENABLE 0
/*
 * Keep OVR soft-recover disabled by default:
 * HAL already disables PIPE1_OVR IRQ on first hit; aggressive re-enable here
 * can create interrupt storms (err1/recov jumping by thousands per second).
 */
#define APP_DCMIPP_ERR_SOFT_RECOVER 0
#define APP_DCMIPP_ERR_LOG_PERIOD_MS 1000U
#define APP_DCMIPP_ERR_POLL_ENABLE 1
#define APP_DCMIPP_ERR_POLL_PERIOD_MS 1000U
#define APP_DCMIPP_ERR_PROBE_CLEAR_ENABLE 1
#define APP_DCMIPP_ERR_POLL_HOLD_LOG_PERIOD_MS 5000U
#define APP_DCMIPP_ERR_PROBE_LOG_PERIOD_MS 5000U
#define APP_DCMIPP_ERR_AUTO_RECOVER_ENABLE 1
#define APP_DCMIPP_ERR_AUTO_RECOVER_STABLE_POLLS 2U

/* Test mode: bypass camera capture and stream a synthetic B/W frame with digit '1'. */
#define APP_UVC_TEST_PATTERN_MODE 1
/* In test mode, stream a prebuilt JPEG directly over UVC (bypass runtime encode). */
#define APP_UVC_TEST_STATIC_JPEG_MODE 0
/* Debug mode: inject prebuilt JPEG into venc_out_buffer and send it (skip runtime encode). */
#define APP_UVC_TEST_INJECT_JPEG_TO_VENC_OUT 0
/* Custom test encoder path: consume YUV from app_uvc_test_yuyv.h and generate JPEG for UVC. */
#define APP_UVC_TEST_CUSTOM_ENCODER_MODE 1
/* In encoder-path test mode, use prebuilt YUYV frame from header as source. */
#define APP_UVC_TEST_PREBUILT_YUYV_MODE 1
#if APP_UVC_TEST_PATTERN_MODE && (APP_UVC_TEST_STATIC_JPEG_MODE || APP_UVC_TEST_INJECT_JPEG_TO_VENC_OUT)
#include "app_uvc_test_jpeg.h"
#endif
#if APP_UVC_TEST_PATTERN_MODE && !APP_UVC_TEST_STATIC_JPEG_MODE && APP_UVC_TEST_PREBUILT_YUYV_MODE
#include "app_uvc_test_yuyv.h"
#endif

/* Encoded bitstream debug: log at low rate to avoid flooding UART/CPU. */
#if APP_UVC_TEST_PATTERN_MODE
#define APP_ENC_DUMP_ENABLE 0
#else
#define APP_ENC_DUMP_ENABLE 0
#endif
#define APP_ENC_DUMP_PERIOD_MS 1000U
#define APP_ENC_DUMP_BYTES 16
#define APP_ENC_DUMP_SUM_BYTES 64
#define APP_ENC_RUNTIME_VALIDATE 1
#define APP_ENC_VALIDATE_LOG_PERIOD_MS 1000U
#define APP_ENC_FORCE_SEND_AFTER_ENCODE 1
#define APP_UVC_TEST_FORCE_DEDICATED_SRC 1
#define APP_UVC_TEST_SMALL_FRAME_ENABLE 1
#define APP_UVC_TEST_SMALL_WIDTH 960
#define APP_UVC_TEST_SMALL_HEIGHT 540
#define APP_UVC_TEST_STREAM_FPS 30
#define APP_UVC_TEST_SRC_IN_AXI 1
#define APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE 0
/* 1: keep test pipeline but use live DCMIPP pipe1 frame as source into g_uvc_test_src_buffer */
#define APP_UVC_TEST_USE_DCMIPP_SOURCE 1
/* Use a dedicated PSRAM MMP window (known-good in probe) for test YUV source buffer. */
#define APP_UVC_TEST_FIXED_PSRAM_SRC_ADDR 0x90100000UL
#define APP_UVC_SEND_DIRECT_ENC_BUF 1
#define APP_UVC_TEST_FREEZE_AFTER_FIRST_ENCODE 0
#define APP_UVC_SRC_LOG_PERIOD_MS 1000U
#define APP_PSRAM_SRC_GUARD_ENABLE 0
#define APP_PSRAM_SRC_GUARD_PERIOD_MS 1000U
/*
 * PSRAM window debug:
 * - XSPI1 memory window starts at 0x9000_0000 (CMSIS XSPI1_BASE).
 * - Linker currently places .psram_bss at 0x9100_0000 (offset inside same window).
 * Do not treat 0x9100_0000 as a direct alias of 0x9000_0000 at same offset.
 */
#define APP_PSRAM_ALIAS_DEBUG_ENABLE 0
#define APP_PSRAM_TEST_SRC_USE_MMP_ALIAS 0
#define APP_ENC_OUT_SENTINEL_DEBUG 0
#define APP_UVC_TEST_SRC_FMT_YUYV 1
#define APP_UVC_TEST_SRC_FMT_UYVY 2
#define APP_UVC_TEST_SRC_FMT_YVYU 3
#define APP_UVC_TEST_SRC_FMT_VYUY 4
#define APP_UVC_TEST_SRC_FMT APP_UVC_TEST_SRC_FMT_YUYV
/* 0=auto from CAPTURE_FORMAT, else force one of APP_UVC_TEST_SRC_FMT_* for live DCMIPP debug. */
#define APP_UVC_LIVE_SRC_FMT_OVERRIDE APP_UVC_TEST_SRC_FMT_YVYU
/* 0: normal path (YUV -> JPEG encoder -> UVC), 1: bypass encoder and stream raw YUY2 over UVC. */
#define APP_UVC_TEST_VIEW_RAW_YUY2 0
/* Software JPEG encoder quality index [0..10], 10 = highest quality in Quant* tables. */
#define APP_SWJPEG_QUALITY_LEVEL 6
#if APP_UVC_TEST_CUSTOM_ENCODER_MODE && (APP_UVC_TEST_STATIC_JPEG_MODE || APP_UVC_TEST_INJECT_JPEG_TO_VENC_OUT)
#error "Custom software encoder mode cannot be combined with static/inject JPEG header modes."
#endif
#if APP_UVC_TEST_PATTERN_MODE
#if APP_UVC_TEST_SMALL_FRAME_ENABLE
#define APP_UVC_ENC_WIDTH APP_UVC_TEST_SMALL_WIDTH
#define APP_UVC_ENC_HEIGHT APP_UVC_TEST_SMALL_HEIGHT
#else
/* Keep test-mode dimensions compile-time constants for static buffer sizing. */
#define APP_UVC_ENC_WIDTH VENC_DVP_WIDTH
#define APP_UVC_ENC_HEIGHT VENC_DVP_HEIGHT
#endif
#else
#define APP_UVC_ENC_WIDTH VENC_WIDTH
#define APP_UVC_ENC_HEIGHT VENC_HEIGHT
#endif
#define APP_UVC_ENC_FRAME_BYTES (APP_UVC_ENC_WIDTH * APP_UVC_ENC_HEIGHT * CAPTURE_BPP)
#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
#define APP_UVC_TEST_SRC_FMT_NAME "YUYV"
#elif (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_UYVY)
#define APP_UVC_TEST_SRC_FMT_NAME "UYVY"
#else
#error "Unsupported APP_UVC_TEST_SRC_FMT"
#endif

/* Runtime source format used by software JPEG path (live DCMIPP can override compile-time test fmt). */
static uint32_t g_uvc_runtime_src_fmt = APP_UVC_TEST_SRC_FMT;

static const char *app_uvc_src_fmt_name(uint32_t fmt)
{
  switch (fmt)
  {
    case APP_UVC_TEST_SRC_FMT_UYVY: return "UYVY";
    case APP_UVC_TEST_SRC_FMT_YVYU: return "YVYU";
    case APP_UVC_TEST_SRC_FMT_VYUY: return "VYUY";
    case APP_UVC_TEST_SRC_FMT_YUYV:
    default: return "YUYV";
  }
}

/* Master switch for any UVC-prefixed log print. */
#define APP_UVC_LOG_ENABLE 0
/* Keep callback-level stream logs even when APP_UVC_LOG_ENABLE is off. */
#define APP_UVC_CB_LOG_ENABLE 0
#define APP_UVC_TEST_BOOT_LOG_ENABLE 0

/* UVC TX-path debug (after encode, before/after USB submit). */
#define APP_UVC_TX_DEBUG_ENABLE 0
#define APP_UVC_TX_DEBUG_PERIOD_MS 1000U
#define APP_UVC_TX_SIG_BYTES 64
#define APP_UVC_CLEAN_DCACHE_BEFORE_SHOW 1
#define APP_UVC_VERBOSE_SRC_LOG_ENABLE 0
#define APP_UVC_COLOR_AUDIT_ENABLE 0
#define APP_UVC_COLOR_AUDIT_PERIOD_MS 1000U
#define APP_UVC_COLOR_AUDIT_SUM_BYTES 512
#define APP_UVC_JPEG_BUF_VERIFY_ENABLE 0
#define APP_UVC_JPEG_BUF_VERIFY_PERIOD_MS 1000U
#define APP_UVC_JPEG_FULL_DUMP_ENABLE 0
#define APP_UVC_JPEG_FULL_DUMP_ONCE 1
#define APP_UVC_JPEG_FULL_DUMP_LINE_BYTES 16
#define APP_UVC_PRESEND_JPEG_CHECK 0
#define APP_UVC_PRESEND_BLOCK_ON_INVALID 1
#define APP_UVC_PRESEND_LOG_PERIOD_MS 5000U
#define APP_UVC_PERF_LOG_ENABLE 1
#define APP_UVC_PERF_LOG_PERIOD_MS 1000U
#if APP_UVC_TEST_PATTERN_MODE
#define APP_UVC_START_WARMUP_FRAMES 0U
#define APP_UVC_START_THROTTLE_ENABLE 0
#define APP_UVC_START_THROTTLE_WINDOW_FRAMES 0U
#define APP_UVC_START_THROTTLE_DIV 1U
#define APP_UVC_PATH_DEBUG_ENABLE 0
#define APP_UVC_PATH_DEBUG_PERIOD_MS 200U
#else
/* Drop first completed frames right after STREAMON to smooth startup burst. */
#define APP_UVC_START_WARMUP_FRAMES 3U
/* During early stream-start window, encode only 1/N frames to reduce PSRAM/AXI burst load. */
#define APP_UVC_START_THROTTLE_ENABLE 1
#define APP_UVC_START_THROTTLE_WINDOW_FRAMES 45U
#define APP_UVC_START_THROTTLE_DIV 3U
/* Try to clear/re-arm PIPE1 overrun state immediately at UVC start. */
#define APP_UVC_START_OVR_RECOVER_ENABLE 1
/* Optional startup-only soft-recover in ISR callback (bounded to avoid interrupt storms). */
#define APP_DCMIPP_ERR_START_SOFT_RECOVER_ENABLE 1
#define APP_DCMIPP_ERR_START_SOFT_RECOVER_WINDOW_MS 3000U
#define APP_DCMIPP_ERR_START_SOFT_RECOVER_MAX 4U
/* Path telemetry from camera frame -> encode -> UVC submit. */
#define APP_UVC_PATH_DEBUG_ENABLE 1
#define APP_UVC_PATH_DEBUG_PERIOD_MS 200U
#endif

static int is_cache_enable(void);
static uint8_t *app_get_test_src_buffer(void);
static uint8_t *app_psram_mmp_alias(uint8_t *buf);
static uint8_t *app_get_test_src_cpu_buffer(void);
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC && APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI && !APP_UVC_TEST_STATIC_JPEG_MODE
static int app_psram_ptr_to_off(const uint8_t *ptr, uint32_t *off);
static int app_psram_indirect_write(uint32_t off, const uint8_t *src, uint32_t len);
static int app_psram_indirect_read(uint32_t off, uint8_t *dst, uint32_t len);
static int app_testpat_pull_dedicated_psram_to_axi(int len);
/* AXI staging buffer used by custom encoder; source of truth remains in PSRAM. */
static uint8_t g_uvc_test_src_axi_staging[APP_UVC_ENC_FRAME_BYTES] ALIGN_32;
#endif
#if APP_PSRAM_SRC_GUARD_ENABLE && APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_PREBUILT_YUYV_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC
static int app_psram_src_compare_against_header(const uint8_t *src, int len,
                                                uint32_t *mismatch_off, uint8_t *got, uint8_t *exp);
#endif

extern int g_psram_init_ret;
extern int g_psram_readid_ret;
extern int g_psram_write_ret;
extern int g_psram_read_ret;
extern int g_psram_verify_mismatch;
extern int g_psram_mmp_ret;
extern uint8_t g_psram_id[6];
extern uint8_t g_psram_probe_rd8[8];

static int BOARD_PinIsAf(GPIO_TypeDef *port, uint32_t pin_index, uint32_t af)
{
  uint32_t mode = (port->MODER >> (pin_index * 2U)) & 0x3U;
  uint32_t afr = (pin_index < 8U) ?
                 ((port->AFR[0] >> (pin_index * 4U)) & 0xFU) :
                 ((port->AFR[1] >> ((pin_index - 8U) * 4U)) & 0xFU);

  return (mode == 0x2U) && (afr == af);
}

static int BOARD_PinIsOutput(GPIO_TypeDef *port, uint32_t pin_index)
{
  uint32_t mode = (port->MODER >> (pin_index * 2U)) & 0x3U;
  return (mode == 0x1U);
}

void BOARD_Pins_Init_DCMIPP(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  int iomux_ok = 1;

  printf("[IOMUX] PX9210K DVP pinmux start\r\n");

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPION_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  /* PB6/PB7/PB8/PB9 => D6/D7/VSYNC/D3 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Alternate = GPIO_AF9_DCMIPP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PC6 => D1 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PE0/PE8 => D2/D4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* PN9 => D5 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPION, &GPIO_InitStruct);

  /* PD0/PD5/PD7 => HSYNC/PIXCLK/D0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PC8 => RESET_N (keep de-asserted) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0U;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  iomux_ok &= BOARD_PinIsAf(GPIOB, 6U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOB, 7U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOB, 8U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOB, 9U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOC, 6U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOE, 0U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOE, 8U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPION, 9U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOD, 0U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOD, 5U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsAf(GPIOD, 7U, GPIO_AF9_DCMIPP);
  iomux_ok &= BOARD_PinIsOutput(GPIOC, 8U);

  if (iomux_ok != 0)
  {
    printf("[IOMUX] PX9210K DVP/RESET pinmux OK\r\n");
  }
  else
  {
    printf("[IOMUX] PX9210K pinmux verify FAILED\r\n");
    assert(0);
  }

  printf("[IOMUX] DVP: PD7(D0) PC6(D1) PE0(D2) PB9(D3) PE8(D4) PN9(D5) PB6(D6) PB7(D7) PD5(PIXCLK) PD0(HSYNC) PB8(VSYNC)\r\n");
  printf("[IOMUX] CTRL: PC8(RESET_N), I2C1(PH9/PC1) owned by BSP\r\n");
}

/* venc conf */
#define VENC_MAX_WIDTH 1920
#define VENC_MAX_HEIGHT 1080
/* Encoded bitstream staging for 1080p JPEG frames. */
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_SMALL_FRAME_ENABLE
/* Small-frame debug mode: keep output buffer compact so we can place it in UNCACHED AXI SRAM. */
#define VENC_OUT_BUFFER_SIZE (128 * 1024)
#else
#define VENC_OUT_BUFFER_SIZE (512 * 1024)
#endif

/* Model Related Info */
#define NN_BUFFER_OUT_SIZE STAI_NETWORK_OUT_1_SIZE_BYTES

/* Align so we are sure nn_output_buffers[0] and nn_output_buffers[1] are aligned on STAI_NETWORK_OUT_1_ALIGNMENT bytes */
#define NN_BUFFER_OUT_SIZE_ALIGN ALIGN_VALUE(NN_BUFFER_OUT_SIZE, STAI_NETWORK_OUT_1_ALIGNMENT)

typedef struct {
  int last;
  int total;
  uint64_t acc;
  float mean;
} time_stat_t;

typedef struct {
  time_stat_t nn_total_time;
  time_stat_t nn_inference_time;
  time_stat_t disp_total_time;
  time_stat_t nn_pp_time;
  time_stat_t disp_display_time;
  time_stat_t disp_enc_time;
} stat_info_t;

typedef struct {
  float conf;
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
} box_t;

typedef struct {
  SemaphoreHandle_t free;
  StaticSemaphore_t free_buffer;
  SemaphoreHandle_t ready;
  StaticSemaphore_t ready_buffer;
  int buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int free_idx;
  int ready_idx;
} bqueue_t;

typedef struct {
  uint64_t current_total;
  uint64_t current_thread_total;
  uint64_t prev_total;
  uint64_t prev_thread_total;
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

/* Globals */
/* display */
static DRAW_Font_t font_12;
static DRAW_Font_t font_16;
static SemaphoreHandle_t stat_info_lock;
static StaticSemaphore_t stat_info_lock_buffer;
static stat_info_t stat_info;
static cpuload_info_t cpu_load;
static int g_pipe1_first_frame_logged;
static int g_pipe2_first_frame_logged;
static int g_pipe1_first_vsync_logged;
static uint32_t g_pipe1_frame_irq_count;
static uint32_t g_pipe2_frame_irq_count;
static uint32_t g_pipe1_vsync_irq_count;
static uint32_t g_pipe_error_irq_count[3];
static uint32_t g_cam_health_last_ms;
static uint32_t g_cam_health_last_pipe1_frame;
static uint32_t g_cam_health_last_pipe2_frame;
static uint32_t g_cam_health_last_pipe1_vsync;
static uint32_t g_cam_health_last_pipe_err[3];
static uint32_t g_pipe1_rearm_ok_count;
static uint32_t g_pipe1_rearm_fail_count;
static volatile int g_pipe1_dump_pending;
static volatile int g_pipe1_dump_idx = -1;
#if !APP_PIPE1_FRAME_DUMP_EVERY_FRAME
static uint32_t g_pipe1_dump_last_ms;
#endif
static int g_pipe1_first_frame_pending_log;
static int g_pipe2_first_frame_pending_log;
static int g_pipe1_first_vsync_pending_log;
static int g_pipe1_first_rearm_ok_pending_log;
static int g_pipe1_rearm_fail_pending_log;
static int g_pipe1_rearm_fail_last_ret;
static uint32_t g_pipe1_rearm_fail_last_state;
static int g_pipe1_rearm_fail_last_disp_idx;
static int g_pipe1_rearm_fail_last_capt_idx;
static int g_pipe1_rearm_fail_last_next_idx;
static int g_pipe1_first_rearm_ok_next_idx;
static uint32_t g_pipe1_first_rearm_ok_addr;
static volatile int g_pipe_error_pending_log;
static uint32_t g_pipe_error_last_pipe;
static uint32_t g_pipe_error_last_state;
static uint32_t g_pipe_error_last_code;
static uint32_t g_pipe_error_last_cmsr2;
static uint32_t g_pipe_error_last_cmier;
static uint32_t g_pipe_error_last_p1fctcr;
static uint32_t g_pipe_error_soft_recover_count;
static uint32_t g_pipe_error_last_log_ms;
static uint32_t g_pipe_error_poll_last_ms;
static uint32_t g_pipe_error_poll_last_err;
static uint32_t g_pipe_error_poll_last_cmsr2;
static uint32_t g_pipe_error_poll_last_cmier;
static uint32_t g_pipe_error_poll_was_active;
static uint32_t g_pipe_error_poll_last_hold_log_ms;
static uint32_t g_pipe_error_probe_clear_ok_count;
static uint32_t g_pipe_error_probe_reassert_count;
static uint32_t g_pipe_error_probe_last_log_ms;
static uint32_t g_pipe_error_stable_clear_polls;
static uint32_t g_pipe_error_auto_recover_count;
static uint32_t g_pipe_error_start_recover_count;
static volatile int g_pipe1_last_completed_idx = -1;
static volatile uint32_t g_pipe1_completed_seq;

/* UVC pipeline debug counters */
static uint32_t g_uvc_stream_active_count;
static uint32_t g_uvc_stream_inactive_count;
static uint32_t g_uvc_enc_ok_count;
static uint32_t g_uvc_enc_fail_count;
static uint32_t g_uvc_drop_busy_count;
static uint32_t g_uvc_show_req_count;
static uint32_t g_uvc_show_ok_count;
static uint32_t g_uvc_show_fail_count;
static uint32_t g_uvc_frame_release_count;
static uint32_t g_uvc_bringup_submit_count;
static int g_uvc_last_submit_idx = -1;
static int g_uvc_last_enc_ret;
static int g_uvc_last_show_ret;
static int g_uvc_last_show_len;
static uint32_t g_uvc_health_last_ms;
static uint32_t g_uvc_health_last_stream_active_count;
static uint32_t g_uvc_health_last_stream_inactive_count;
static uint32_t g_uvc_health_last_enc_ok_count;
static uint32_t g_uvc_health_last_enc_fail_count;
static uint32_t g_uvc_health_last_drop_busy_count;
static uint32_t g_uvc_health_last_show_req_count;
static uint32_t g_uvc_health_last_show_ok_count;
static uint32_t g_uvc_health_last_show_fail_count;
static uint32_t g_uvc_health_last_frame_release_count;
static uint32_t g_uvc_health_last_bringup_submit_count;
static uint32_t g_uvc_health_last_pipe1_completed_seq;
static uint32_t g_enc_dump_last_ms;
static uint32_t g_enc_bs_ok_count;
static uint32_t g_enc_bs_bad_count;
static uint32_t g_enc_bs_last_ok_count;
static uint32_t g_enc_bs_last_bad_count;
static uint32_t g_enc_runtime_valid_count;
static uint32_t g_enc_runtime_invalid_count;
static uint32_t g_enc_runtime_copy_mismatch_count;
static uint32_t g_enc_runtime_last_log_ms;
static uint32_t g_uvc_src_last_log_ms;
static uint32_t g_psram_src_guard_last_ms;
static uint32_t g_psram_src_guard_ok_count;
static uint32_t g_psram_src_guard_bad_count;
static uint32_t g_psram_src_guard_recover_ok_count;
static uint32_t g_psram_src_guard_recover_fail_count;
static uint32_t g_enc_out_sentinel_bad_count;
static uint32_t g_uvc_tx_last_log_ms;
static uint32_t g_uvc_tx_submit_id;
static uint32_t g_uvc_tx_release_id;
static uint32_t g_uvc_tx_last_submit_len;
static uint32_t g_uvc_tx_last_submit_sig;
static uint32_t g_uvc_tx_last_release_sig;
static uint32_t g_uvc_tx_ptr_mismatch_count;
static uint32_t g_uvc_tx_sig_mismatch_count;
static uint32_t g_uvc_presend_jpeg_ok_count;
static uint32_t g_uvc_presend_jpeg_bad_count;
static uint32_t g_uvc_presend_jpeg_last_log_ms;
static uintptr_t g_uvc_tx_last_release_ptr;
static uint32_t g_uvc_stream_session_id;
static uint32_t g_uvc_stream_start_tick_ms;
static uint32_t g_uvc_start_warmup_left;
static uint32_t g_uvc_start_throttle_left;
static uint32_t g_uvc_start_throttle_skip_count;
static uint32_t g_uvc_start_throttle_keep_count;
static uint32_t g_uvc_path_last_log_ms;
static uint32_t g_uvc_last_pipe1_seq;
static uint32_t g_uvc_last_enc_ms;
static uint32_t g_uvc_last_send_ms;
static uint32_t g_uvc_perf_last_ms;
static uint32_t g_uvc_perf_last_pipe1_completed_seq;
static uint32_t g_uvc_perf_last_bringup_submit_count;
static uint32_t g_uvc_perf_last_enc_ok_count;
static uint32_t g_uvc_perf_last_enc_fail_count;
static uint32_t g_uvc_perf_last_drop_busy_count;
static uint32_t g_uvc_perf_last_show_ok_count;
static uint32_t g_uvc_perf_last_show_fail_count;
static uint32_t g_uvc_perf_last_frame_release_count;
static int uvc_is_active;
static volatile int buffer_flying;

/* Forward declarations: used by early debug dump helper before full definitions below. */
static uint8_t capture_buffer[CAPTURE_BUFFER_NB][VENC_MAX_WIDTH * VENC_MAX_HEIGHT * CAPTURE_BPP];
static int capture_buffer_disp_idx;
static uint32_t app_sum_prefix_u32(const uint8_t *buf, int len, int max_len);

static void app_dump_pipe1_frame_data_1s(void)
{
#if APP_PIPE1_FRAME_DUMP_ENABLE
  uint8_t *src;
  uintptr_t src_addr;
  uintptr_t inval_addr;
  uint32_t inval_len;
#if !APP_PIPE1_FRAME_DUMP_EVERY_FRAME
  uint32_t now_ms;
#endif
  uint32_t sum;
  int idx;
  int i;
  uint8_t sample[APP_PIPE1_FRAME_DUMP_BYTES];
  char hexbuf[APP_PIPE1_FRAME_DUMP_BYTES * 3];
  static const char hexdig[] = "0123456789ABCDEF";

  if (g_pipe1_dump_pending == 0)
  {
    return;
  }

#if !APP_PIPE1_FRAME_DUMP_EVERY_FRAME
  now_ms = HAL_GetTick();
  if ((now_ms - g_pipe1_dump_last_ms) < APP_PIPE1_FRAME_DUMP_PERIOD_MS)
  {
    return;
  }
#endif

  idx = g_pipe1_dump_idx;
  if ((idx < 0) || (idx >= CAPTURE_BUFFER_NB))
  {
    g_pipe1_dump_pending = 0;
    return;
  }

  src = capture_buffer[idx];
  src_addr = (uintptr_t)src;
  inval_addr = src_addr & ~((uintptr_t)31U);
  inval_len = ALIGN_VALUE((uint32_t)((src_addr - inval_addr) + APP_PIPE1_FRAME_DUMP_BYTES), 32U);
  CACHE_OP(SCB_InvalidateDCache_by_Addr((uint8_t *)inval_addr, inval_len));

  sum = 0U;
  for (i = 0; i < APP_PIPE1_FRAME_DUMP_BYTES; i++)
  {
    sample[i] = src[i];
    sum += sample[i];
    hexbuf[(i * 3) + 0] = hexdig[(sample[i] >> 4) & 0x0F];
    hexbuf[(i * 3) + 1] = hexdig[sample[i] & 0x0F];
    hexbuf[(i * 3) + 2] = ((i + 1) < APP_PIPE1_FRAME_DUMP_BYTES) ? ' ' : '\0';
  }

  printf("[CAM][DATA] p1 idx=%d addr=0x%08lX b0..b%u=%s sum%u=%lu\r\n",
         idx,
         (unsigned long)capture_buffer[idx],
         (unsigned)(APP_PIPE1_FRAME_DUMP_BYTES - 1U),
         hexbuf,
         APP_PIPE1_FRAME_DUMP_BYTES,
         (unsigned long)sum);

#if !APP_PIPE1_FRAME_DUMP_EVERY_FRAME
  g_pipe1_dump_last_ms = now_ms;
#endif
  g_pipe1_dump_pending = 0;
#endif
}

static void app_log_cam_health_1s(void)
{
#if !APP_CAM_HEALTH_LOG_ENABLE
  return;
#else
  uint32_t now_ms = HAL_GetTick();
  uint32_t cmsr2 = 0U;
  uint32_t cmier = 0U;
  uint32_t err = 0U;
  uint32_t p1_state = 0U;
  uint32_t p1_ovr_flag = 0U;
  uint32_t p1_ovr_irq_en = 0U;
  DCMIPP_HandleTypeDef *hdcmipp = CMW_CAMERA_GetDCMIPPHandle();

  if ((now_ms - g_cam_health_last_ms) < 1000U)
  {
    return;
  }

  if ((hdcmipp != NULL) && (hdcmipp->Instance != NULL))
  {
    cmsr2 = READ_REG(hdcmipp->Instance->CMSR2);
    cmier = READ_REG(hdcmipp->Instance->CMIER);
    err = hdcmipp->ErrorCode;
    p1_state = HAL_DCMIPP_PIPE_GetState(hdcmipp, DCMIPP_PIPE1);
    p1_ovr_flag = ((cmsr2 & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U;
    p1_ovr_irq_en = ((cmier & DCMIPP_IT_PIPE1_OVR) != 0U) ? 1U : 0U;
  }

    printf("[CAM][HEALTH] 1s p1_frame=+%lu p2_frame=+%lu p1_vsync=+%lu rearm_ok=%lu rearm_fail=%lu err0=+%lu err1=+%lu err2=+%lu tot_err=(%lu,%lu,%lu) p1_state=%lu err=0x%08lX p1_ovrf=%lu p1_ovrie=%lu probe_ok=%lu probe_re=%lu auto_rec=%lu\r\n",
         (unsigned long)(g_pipe1_frame_irq_count - g_cam_health_last_pipe1_frame),
         (unsigned long)(g_pipe2_frame_irq_count - g_cam_health_last_pipe2_frame),
         (unsigned long)(g_pipe1_vsync_irq_count - g_cam_health_last_pipe1_vsync),
      (unsigned long)g_pipe1_rearm_ok_count,
      (unsigned long)g_pipe1_rearm_fail_count,
         (unsigned long)(g_pipe_error_irq_count[0] - g_cam_health_last_pipe_err[0]),
         (unsigned long)(g_pipe_error_irq_count[1] - g_cam_health_last_pipe_err[1]),
         (unsigned long)(g_pipe_error_irq_count[2] - g_cam_health_last_pipe_err[2]),
         (unsigned long)g_pipe_error_irq_count[0],
         (unsigned long)g_pipe_error_irq_count[1],
         (unsigned long)g_pipe_error_irq_count[2],
         (unsigned long)p1_state,
         (unsigned long)err,
         (unsigned long)p1_ovr_flag,
         (unsigned long)p1_ovr_irq_en,
         (unsigned long)g_pipe_error_probe_clear_ok_count,
         (unsigned long)g_pipe_error_probe_reassert_count,
         (unsigned long)g_pipe_error_auto_recover_count);

  g_cam_health_last_ms = now_ms;
  g_cam_health_last_pipe1_frame = g_pipe1_frame_irq_count;
  g_cam_health_last_pipe2_frame = g_pipe2_frame_irq_count;
  g_cam_health_last_pipe1_vsync = g_pipe1_vsync_irq_count;
  g_cam_health_last_pipe_err[0] = g_pipe_error_irq_count[0];
  g_cam_health_last_pipe_err[1] = g_pipe_error_irq_count[1];
  g_cam_health_last_pipe_err[2] = g_pipe_error_irq_count[2];
#endif
}

static void app_log_dcmipp_error_poll_1s(void)
{
#if !APP_DCMIPP_ERR_POLL_ENABLE
  return;
#else
  uint32_t now_ms = HAL_GetTick();
  DCMIPP_HandleTypeDef *hdcmipp = CMW_CAMERA_GetDCMIPPHandle();
  uint32_t cmsr2;
  uint32_t cmier;
  uint32_t err;
  uint32_t p1_state;
  uint32_t p1_ovrf;
  uint32_t p1_ovrie;
  uint32_t p1_ovrerr;
  uint32_t cmsr2_after;
  uint32_t p1_ovrf_after;
  int changed;
  int has_error;

  if ((now_ms - g_pipe_error_poll_last_ms) < APP_DCMIPP_ERR_POLL_PERIOD_MS)
  {
    return;
  }
  g_pipe_error_poll_last_ms = now_ms;

  if ((hdcmipp == NULL) || (hdcmipp->Instance == NULL))
  {
    return;
  }

  cmsr2 = READ_REG(hdcmipp->Instance->CMSR2);
  cmier = READ_REG(hdcmipp->Instance->CMIER);
  err = hdcmipp->ErrorCode;
  p1_state = HAL_DCMIPP_PIPE_GetState(hdcmipp, DCMIPP_PIPE1);
  p1_ovrf = ((cmsr2 & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U;
  p1_ovrie = ((cmier & DCMIPP_IT_PIPE1_OVR) != 0U) ? 1U : 0U;
  p1_ovrerr = ((err & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U) ? 1U : 0U;
  has_error = (p1_ovrf != 0U) || (p1_ovrerr != 0U) || (p1_state == HAL_DCMIPP_PIPE_STATE_ERROR);
  changed = (err != g_pipe_error_poll_last_err) ||
            (cmsr2 != g_pipe_error_poll_last_cmsr2) ||
            (cmier != g_pipe_error_poll_last_cmier);

  if (!has_error)
  {
    if (g_pipe_error_poll_was_active != 0U)
    {
      printf("[CAM][ERR][CLR] p1_state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX err_irq_tot=%lu\r\n",
             (unsigned long)p1_state,
             (unsigned long)err,
             (unsigned long)cmsr2,
             (unsigned long)cmier,
             (unsigned long)g_pipe_error_irq_count[1]);
    }
    g_pipe_error_poll_was_active = 0U;
    g_pipe_error_poll_last_hold_log_ms = 0U;
  }
  else
  {
    const char *tag = NULL;
    if ((g_pipe_error_poll_was_active == 0U) || changed)
    {
      tag = "new";
      g_pipe_error_poll_last_hold_log_ms = now_ms;
    }
    else if ((g_pipe_error_poll_last_hold_log_ms == 0U) ||
             ((now_ms - g_pipe_error_poll_last_hold_log_ms) >= APP_DCMIPP_ERR_POLL_HOLD_LOG_PERIOD_MS))
    {
      tag = "hold";
      g_pipe_error_poll_last_hold_log_ms = now_ms;
    }

    if (tag != NULL)
    {
      printf("[CAM][ERR][POLL] %s p1_state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX p1_ovrf=%lu p1_ovrie=%lu p1_ovrerr=%lu err_irq_tot=%lu\r\n",
             tag,
             (unsigned long)p1_state,
             (unsigned long)err,
             (unsigned long)cmsr2,
             (unsigned long)cmier,
             (unsigned long)p1_ovrf,
             (unsigned long)p1_ovrie,
             (unsigned long)p1_ovrerr,
             (unsigned long)g_pipe_error_irq_count[1]);
    }
    g_pipe_error_poll_was_active = 1U;
  }

#if APP_DCMIPP_ERR_PROBE_CLEAR_ENABLE
  /* Probe path: when OVR IRQ is masked, clear OVR flag and check if it re-asserts.
   * This distinguishes a sticky latched flag from a true continuous overrun condition. */
  if ((p1_ovrf != 0U) && (p1_ovrie == 0U))
  {
    int do_probe_log;

    __HAL_DCMIPP_CLEAR_FLAG(hdcmipp, DCMIPP_FLAG_PIPE1_OVR);
    __DSB();
    cmsr2_after = READ_REG(hdcmipp->Instance->CMSR2);
    p1_ovrf_after = ((cmsr2_after & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U;
    if (p1_ovrf_after == 0U)
    {
      g_pipe_error_probe_clear_ok_count++;
    }
    else
    {
      g_pipe_error_probe_reassert_count++;
    }

    do_probe_log = 0;
    if (p1_ovrf_after != 0U)
    {
      do_probe_log = 1; /* important: flag re-asserted immediately */
    }
    else if ((g_pipe_error_probe_last_log_ms == 0U) ||
             ((now_ms - g_pipe_error_probe_last_log_ms) >= APP_DCMIPP_ERR_PROBE_LOG_PERIOD_MS))
    {
      do_probe_log = 1; /* keep only periodic success probe logs */
    }

    if (do_probe_log != 0)
    {
      printf("[CAM][ERR][PROBE] clr_p1ovr before=1 after=%lu state=%lu err=0x%08lX cmsr2=0x%08lX->0x%08lX ok=%lu re=%lu\r\n",
             (unsigned long)p1_ovrf_after,
             (unsigned long)p1_state,
             (unsigned long)err,
             (unsigned long)cmsr2,
             (unsigned long)cmsr2_after,
             (unsigned long)g_pipe_error_probe_clear_ok_count,
             (unsigned long)g_pipe_error_probe_reassert_count);
      g_pipe_error_probe_last_log_ms = now_ms;
    }
  }
#endif

#if APP_DCMIPP_ERR_AUTO_RECOVER_ENABLE
  /* If raw OVR flag is gone for several polls but HAL state/error are still latched,
   * recover to BUSY and re-arm OVR IRQ to monitor new real errors. */
  if ((p1_ovrf == 0U) && (p1_ovrerr != 0U))
  {
    g_pipe_error_stable_clear_polls++;
  }
  else
  {
    g_pipe_error_stable_clear_polls = 0U;
  }

  if ((g_pipe_error_stable_clear_polls >= APP_DCMIPP_ERR_AUTO_RECOVER_STABLE_POLLS) &&
      (p1_state == HAL_DCMIPP_PIPE_STATE_ERROR))
  {
    __HAL_DCMIPP_CLEAR_FLAG(hdcmipp, DCMIPP_FLAG_PIPE1_OVR);
    hdcmipp->ErrorCode &= ~HAL_DCMIPP_ERROR_PIPE1_OVR;
    hdcmipp->PipeState[1] = HAL_DCMIPP_PIPE_STATE_BUSY;
    __HAL_DCMIPP_ENABLE_IT(hdcmipp, DCMIPP_IT_PIPE1_OVR);
    g_pipe_error_auto_recover_count++;
    g_pipe_error_stable_clear_polls = 0U;
    printf("[CAM][ERR][AUTO-RECOVER] pipe1 state->BUSY clear_err=0x%08lX rearm_ovr_irq=1 cnt=%lu\r\n",
           (unsigned long)hdcmipp->ErrorCode,
           (unsigned long)g_pipe_error_auto_recover_count);
  }
#endif

  g_pipe_error_poll_last_err = err;
  g_pipe_error_poll_last_cmsr2 = cmsr2;
  g_pipe_error_poll_last_cmier = cmier;
#endif
}

static void app_log_uvc_health_1s(void)
{
#if !APP_UVC_LOG_ENABLE
  return;
#else
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - g_uvc_health_last_ms) < 1000U)
  {
    return;
  }

  printf("[UVC][HEALTH] 1s active=%d flying=%d p1_seq=+%lu on=+%lu off=+%lu submit=+%lu enc_ok=+%lu enc_fail=+%lu drop=+%lu show_req=+%lu show_ok=+%lu show_fail=+%lu rel=+%lu last(enc=%d show_ret=%d len=%d idx=%d)\r\n",
         uvc_is_active,
         buffer_flying,
         (unsigned long)(g_pipe1_completed_seq - g_uvc_health_last_pipe1_completed_seq),
         (unsigned long)(g_uvc_stream_active_count - g_uvc_health_last_stream_active_count),
         (unsigned long)(g_uvc_stream_inactive_count - g_uvc_health_last_stream_inactive_count),
         (unsigned long)(g_uvc_bringup_submit_count - g_uvc_health_last_bringup_submit_count),
         (unsigned long)(g_uvc_enc_ok_count - g_uvc_health_last_enc_ok_count),
         (unsigned long)(g_uvc_enc_fail_count - g_uvc_health_last_enc_fail_count),
         (unsigned long)(g_uvc_drop_busy_count - g_uvc_health_last_drop_busy_count),
         (unsigned long)(g_uvc_show_req_count - g_uvc_health_last_show_req_count),
         (unsigned long)(g_uvc_show_ok_count - g_uvc_health_last_show_ok_count),
         (unsigned long)(g_uvc_show_fail_count - g_uvc_health_last_show_fail_count),
         (unsigned long)(g_uvc_frame_release_count - g_uvc_health_last_frame_release_count),
         g_uvc_last_enc_ret,
         g_uvc_last_show_ret,
         g_uvc_last_show_len,
         g_uvc_last_submit_idx);

  printf("[UVC][STATE] stream_on=%lu stream_off=%lu total_show_ok=%lu total_rel=%lu tx_sub=%lu tx_rel=%lu ptr_mis=%lu sig_mis=%lu sub_sig=%lu rel_sig=%lu\r\n",
         (unsigned long)g_uvc_stream_active_count,
         (unsigned long)g_uvc_stream_inactive_count,
         (unsigned long)g_uvc_show_ok_count,
         (unsigned long)g_uvc_frame_release_count,
         (unsigned long)g_uvc_tx_submit_id,
         (unsigned long)g_uvc_tx_release_id,
         (unsigned long)g_uvc_tx_ptr_mismatch_count,
         (unsigned long)g_uvc_tx_sig_mismatch_count,
         (unsigned long)g_uvc_tx_last_submit_sig,
         (unsigned long)g_uvc_tx_last_release_sig);

#if APP_ENC_RUNTIME_VALIDATE
  printf("[ENC][CHK] total valid=%lu invalid=%lu copy_mis=%lu out_head_bad=%lu\r\n",
         (unsigned long)g_enc_runtime_valid_count,
         (unsigned long)g_enc_runtime_invalid_count,
         (unsigned long)g_enc_runtime_copy_mismatch_count,
         (unsigned long)g_enc_out_sentinel_bad_count);
#endif

  g_uvc_health_last_ms = now_ms;
  g_uvc_health_last_stream_active_count = g_uvc_stream_active_count;
  g_uvc_health_last_stream_inactive_count = g_uvc_stream_inactive_count;
  g_uvc_health_last_enc_ok_count = g_uvc_enc_ok_count;
  g_uvc_health_last_enc_fail_count = g_uvc_enc_fail_count;
  g_uvc_health_last_drop_busy_count = g_uvc_drop_busy_count;
  g_uvc_health_last_show_req_count = g_uvc_show_req_count;
  g_uvc_health_last_show_ok_count = g_uvc_show_ok_count;
  g_uvc_health_last_show_fail_count = g_uvc_show_fail_count;
  g_uvc_health_last_frame_release_count = g_uvc_frame_release_count;
  g_uvc_health_last_bringup_submit_count = g_uvc_bringup_submit_count;
  g_uvc_health_last_pipe1_completed_seq = g_pipe1_completed_seq;
#endif
}

typedef struct
{
  int has_soi;
  int has_eoi;
  int sof_off;
  int sos_off;
  int sof_w;
  int sof_h;
  int sof_ok;
  uint8_t sof_b[10];
} app_jpeg_meta_t;

static void app_jpeg_parse_meta(const uint8_t *p_bs, int p_len, app_jpeg_meta_t *m)
{
  int i;

  memset(m, 0, sizeof(*m));
  m->sof_off = -1;
  m->sos_off = -1;
  m->sof_w = -1;
  m->sof_h = -1;
  if ((p_bs == NULL) || (p_len < 4) || (p_len > (int)VENC_OUT_BUFFER_SIZE))
  {
    return;
  }

  m->has_soi = (p_bs[0] == 0xFFU) && (p_bs[1] == 0xD8U);
  m->has_eoi = (p_bs[p_len - 2] == 0xFFU) && (p_bs[p_len - 1] == 0xD9U);
  for (i = 2; i < (p_len - 1); i++)
  {
    if (p_bs[i] != 0xFFU)
    {
      continue;
    }
    if ((p_bs[i + 1] == 0xC0U) || (p_bs[i + 1] == 0xC2U))
    {
      if (m->sof_off < 0)
      {
        m->sof_off = i;
      }
    }
    if (p_bs[i + 1] == 0xDAU)
    {
      if (m->sos_off < 0)
      {
        m->sos_off = i;
      }
    }
  }
  if ((m->sof_off >= 0) && ((m->sof_off + 8) < p_len))
  {
    int seg_len = ((int)p_bs[m->sof_off + 2] << 8) | (int)p_bs[m->sof_off + 3];
    int n_sof = ((m->sof_off + (int)sizeof(m->sof_b)) <= p_len) ? (int)sizeof(m->sof_b) : (p_len - m->sof_off);
    for (i = 0; i < n_sof; i++)
    {
      m->sof_b[i] = p_bs[m->sof_off + i];
    }
    m->sof_h = ((int)p_bs[m->sof_off + 5] << 8) | (int)p_bs[m->sof_off + 6];
    m->sof_w = ((int)p_bs[m->sof_off + 7] << 8) | (int)p_bs[m->sof_off + 8];
    m->sof_ok = (seg_len >= 8) ? 1 : 0;
  }
}

static int app_jpeg_is_valid(const app_jpeg_meta_t *m)
{
  return (m->has_soi != 0) && (m->has_eoi != 0) && (m->sof_off > 0) &&
         (m->sos_off > 0) && (m->sos_off > m->sof_off) &&
         (m->sof_ok != 0) && (m->sof_w > 0) && (m->sof_h > 0);
}

static int app_uvc_presend_check_jpeg(const uint8_t *buf, int len)
{
#if APP_UVC_PRESEND_JPEG_CHECK
  app_jpeg_meta_t meta;
  int valid;
  uint32_t now_ms;
  uint32_t sum;
  uint8_t b0 = 0U;
  uint8_t b1 = 0U;
  uint8_t b2 = 0U;
  uint8_t b3 = 0U;
  int force_log = 0;

  app_jpeg_parse_meta(buf, len, &meta);
  valid = app_jpeg_is_valid(&meta);
  now_ms = HAL_GetTick();
  if (valid != 0)
  {
    g_uvc_presend_jpeg_ok_count++;
  }
  else
  {
    g_uvc_presend_jpeg_bad_count++;
    force_log = 1;
  }

  if ((now_ms - g_uvc_presend_jpeg_last_log_ms) >= APP_UVC_PRESEND_LOG_PERIOD_MS)
  {
    force_log = 1;
  }

  if (force_log != 0)
  {
    sum = app_sum_prefix_u32(buf, len, APP_ENC_DUMP_SUM_BYTES);
    if ((buf != NULL) && (len >= 4))
    {
      b0 = buf[0];
      b1 = buf[1];
      b2 = buf[2];
      b3 = buf[3];
    }
    printf("[UVC][PRE] jpeg_%s buf=0x%08lX len=%d soi=%d sof=%d wh=%dx%d sos=%d eoi=%d b0..b3=%02X %02X %02X %02X sum%u=%lu ok=%lu bad=%lu\r\n",
           (valid != 0) ? "ok" : "bad",
           (unsigned long)buf,
           len,
           meta.has_soi,
           meta.sof_off,
           meta.sof_w,
           meta.sof_h,
           meta.sos_off,
           meta.has_eoi,
           b0, b1, b2, b3,
           APP_ENC_DUMP_SUM_BYTES,
           (unsigned long)sum,
           (unsigned long)g_uvc_presend_jpeg_ok_count,
           (unsigned long)g_uvc_presend_jpeg_bad_count);
    g_uvc_presend_jpeg_last_log_ms = now_ms;
  }

  return (valid != 0) ? 0 : -1;
#else
  (void)buf;
  (void)len;
  return 0;
#endif
}

static void app_log_uvc_perf_1s(void)
{
#if !APP_UVC_PERF_LOG_ENABLE
  return;
#else
  uint32_t now_ms = HAL_GetTick();
  uint32_t d_cap;
  uint32_t d_submit;
  uint32_t d_enc_ok;
  uint32_t d_enc_fail;
  uint32_t d_drop_busy;
  uint32_t d_show_ok;
  uint32_t d_show_fail;
  uint32_t d_release;

  if ((now_ms - g_uvc_perf_last_ms) < APP_UVC_PERF_LOG_PERIOD_MS)
  {
    return;
  }

  d_cap = g_pipe1_completed_seq - g_uvc_perf_last_pipe1_completed_seq;
  d_submit = g_uvc_bringup_submit_count - g_uvc_perf_last_bringup_submit_count;
  d_enc_ok = g_uvc_enc_ok_count - g_uvc_perf_last_enc_ok_count;
  d_enc_fail = g_uvc_enc_fail_count - g_uvc_perf_last_enc_fail_count;
  d_drop_busy = g_uvc_drop_busy_count - g_uvc_perf_last_drop_busy_count;
  d_show_ok = g_uvc_show_ok_count - g_uvc_perf_last_show_ok_count;
  d_show_fail = g_uvc_show_fail_count - g_uvc_perf_last_show_fail_count;
  d_release = g_uvc_frame_release_count - g_uvc_perf_last_frame_release_count;

  printf("==== PERF 1s: cap=%lu submit=%lu enc_ok=%lu enc_fail=%lu drop_busy=%lu show_ok=%lu show_fail=%lu release=%lu flying=%d enc_ms=%lu send_ms=%lu ====\r\n",
         (unsigned long)d_cap,
         (unsigned long)d_submit,
         (unsigned long)d_enc_ok,
         (unsigned long)d_enc_fail,
         (unsigned long)d_drop_busy,
         (unsigned long)d_show_ok,
         (unsigned long)d_show_fail,
         (unsigned long)d_release,
         buffer_flying,
         (unsigned long)g_uvc_last_enc_ms,
         (unsigned long)g_uvc_last_send_ms);

  g_uvc_perf_last_ms = now_ms;
  g_uvc_perf_last_pipe1_completed_seq = g_pipe1_completed_seq;
  g_uvc_perf_last_bringup_submit_count = g_uvc_bringup_submit_count;
  g_uvc_perf_last_enc_ok_count = g_uvc_enc_ok_count;
  g_uvc_perf_last_enc_fail_count = g_uvc_enc_fail_count;
  g_uvc_perf_last_drop_busy_count = g_uvc_drop_busy_count;
  g_uvc_perf_last_show_ok_count = g_uvc_show_ok_count;
  g_uvc_perf_last_show_fail_count = g_uvc_show_fail_count;
  g_uvc_perf_last_frame_release_count = g_uvc_frame_release_count;
#endif
}

static void app_log_encoded_data_1s(const uint8_t *bs, int len)
{
#if APP_ENC_DUMP_ENABLE
  app_jpeg_meta_t meta;
  uint32_t now_ms;
  uint32_t sum = 0U;
  uint8_t sample[8] = {0};
  int sum_bytes;
  int i;
  int valid;

  app_jpeg_parse_meta(bs, len, &meta);
  valid = app_jpeg_is_valid(&meta);

  if (valid)
  {
    g_enc_bs_ok_count++;
  }
  else
  {
    g_enc_bs_bad_count++;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - g_enc_dump_last_ms) < APP_ENC_DUMP_PERIOD_MS)
  {
    return;
  }

  if ((bs != NULL) && (len > 0))
  {
    int n = (len > 8) ? 8 : len;
    sum_bytes = (len > APP_ENC_DUMP_SUM_BYTES) ? APP_ENC_DUMP_SUM_BYTES : len;

    for (i = 0; i < n; i++)
    {
      sample[i] = bs[i];
    }
    for (i = 0; i < sum_bytes; i++)
    {
      sum += bs[i];
    }
  }
  else
  {
    sum_bytes = 0;
  }

  printf("[ENC][MJPEG] size=%d soi=%d sof=%d wh=%dx%d sos=%d eoi=%d sof_b=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X b0..b7=%02X %02X %02X %02X %02X %02X %02X %02X sum%u=%lu ok=+%lu bad=+%lu tot=(%lu,%lu)\r\n",
         len,
         meta.has_soi,
         meta.sof_off,
         meta.sof_w,
         meta.sof_h,
         meta.sos_off,
         meta.has_eoi,
         meta.sof_b[0], meta.sof_b[1], meta.sof_b[2], meta.sof_b[3], meta.sof_b[4],
         meta.sof_b[5], meta.sof_b[6], meta.sof_b[7], meta.sof_b[8], meta.sof_b[9],
         sample[0], sample[1], sample[2], sample[3],
         sample[4], sample[5], sample[6], sample[7],
         (unsigned int)sum_bytes,
         (unsigned long)sum,
         (unsigned long)(g_enc_bs_ok_count - g_enc_bs_last_ok_count),
         (unsigned long)(g_enc_bs_bad_count - g_enc_bs_last_bad_count),
         (unsigned long)g_enc_bs_ok_count,
         (unsigned long)g_enc_bs_bad_count);

  g_enc_bs_last_ok_count = g_enc_bs_ok_count;
  g_enc_bs_last_bad_count = g_enc_bs_bad_count;
  g_enc_dump_last_ms = now_ms;
#else
  (void)bs;
  (void)len;
#endif
}

static uint32_t app_sum_prefix_u32(const uint8_t *buf, int len, int max_len)
{
  uint32_t sum = 0U;
  int n;
  int i;

  if ((buf == NULL) || (len <= 0) || (max_len <= 0))
  {
    return 0U;
  }

  n = (len < max_len) ? len : max_len;
  for (i = 0; i < n; i++)
  {
    sum += buf[i];
  }

  return sum;
}

static int app_clamp_u8_i32(int v)
{
  if (v < 0)
  {
    return 0;
  }
  if (v > 255)
  {
    return 255;
  }
  return v;
}

static void app_yuv_to_rgb_bt601_limited(uint8_t y, uint8_t u, uint8_t v, int *r, int *g, int *b)
{
  int c = (int)y - 16;
  int d = (int)u - 128;
  int e = (int)v - 128;
  int rr;
  int gg;
  int bb;

  if (c < 0)
  {
    c = 0;
  }

  rr = (298 * c + 409 * e + 128) >> 8;
  gg = (298 * c - 100 * d - 208 * e + 128) >> 8;
  bb = (298 * c + 516 * d + 128) >> 8;

  *r = app_clamp_u8_i32(rr);
  *g = app_clamp_u8_i32(gg);
  *b = app_clamp_u8_i32(bb);
}

static int app_read_yuv422_sample(const uint8_t *buf, int width, int height, int x, int y, uint8_t *yy, uint8_t *uu, uint8_t *vv)
{
  const uint8_t *pix;
  int x_even;

  if ((buf == NULL) || (width <= 0) || (height <= 0) || (yy == NULL) || (uu == NULL) || (vv == NULL))
  {
    return -1;
  }
  if ((x < 0) || (x >= width) || (y < 0) || (y >= height))
  {
    return -1;
  }

  x_even = x & ~1;
  pix = buf + ((size_t)y * (size_t)width * 2U) + ((size_t)x_even * 2U);

#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
  *yy = ((x & 1) == 0) ? pix[0] : pix[2];
  *uu = pix[1];
  *vv = pix[3];
#else
  *yy = ((x & 1) == 0) ? pix[1] : pix[3];
  *uu = pix[0];
  *vv = pix[2];
#endif

  return 0;
}

static void app_uvc_log_yuv_probe_1s(const uint8_t *buf, int width, int height)
{
#if APP_UVC_TEST_PATTERN_MODE
  static uint32_t last_ms;
  uint32_t now_ms = HAL_GetTick();
  uint8_t y0, u0, v0;
  uint8_t y1, u1, v1;
  int r0, g0, b0;
  int r1, g1, b1;
  int x_left;
  int x_right;
  int y_mid;

  if ((buf == NULL) || (width <= 0) || (height <= 0))
  {
    return;
  }
  if ((now_ms - last_ms) < 1000U)
  {
    return;
  }
  last_ms = now_ms;

  x_left = width / 4;
  x_right = (3 * width) / 4;
  y_mid = height / 2;

  if (app_read_yuv422_sample(buf, width, height, x_left, y_mid, &y0, &u0, &v0) != 0)
  {
    return;
  }
  if (app_read_yuv422_sample(buf, width, height, x_right, y_mid, &y1, &u1, &v1) != 0)
  {
    return;
  }

  app_yuv_to_rgb_bt601_limited(y0, u0, v0, &r0, &g0, &b0);
  app_yuv_to_rgb_bt601_limited(y1, u1, v1, &r1, &g1, &b1);

  printf("[UVC][YUV] fmt=%s left(yuv=%u,%u,%u rgb=%d,%d,%d) right(yuv=%u,%u,%u rgb=%d,%d,%d)\r\n",
         APP_UVC_TEST_SRC_FMT_NAME,
         (unsigned int)y0, (unsigned int)u0, (unsigned int)v0, r0, g0, b0,
         (unsigned int)y1, (unsigned int)u1, (unsigned int)v1, r1, g1, b1);
#else
  (void)buf;
  (void)width;
  (void)height;
#endif
}

static void app_uvc_log_color_audit_1s(const uint8_t *src_yuv, int src_len, const uint8_t *jpeg_bs, int jpeg_len)
{
#if APP_UVC_COLOR_AUDIT_ENABLE
  static uint32_t last_ms;
  static uint32_t prev_yuv_sig;
  static uint32_t prev_jpeg_sig;
  static int prev_valid;
  uint32_t now_ms = HAL_GetTick();
  uint32_t yuv_sig;
  uint32_t jpeg_sig;
  int yuv_changed = 0;
  int jpeg_changed = 0;
  int x_left;
  int x_right;
  int y_mid;
  uint8_t y0, u0, v0;
  uint8_t y1, u1, v1;
  int r0 = 0, g0 = 0, b0 = 0;
  int r1 = 0, g1 = 0, b1 = 0;
  int got_left = 0;
  int got_right = 0;
  app_jpeg_meta_t meta;
  int jpeg_ok;

  if ((src_yuv == NULL) || (jpeg_bs == NULL) || (src_len <= 0) || (jpeg_len <= 0))
  {
    return;
  }

  if ((now_ms - last_ms) < APP_UVC_COLOR_AUDIT_PERIOD_MS)
  {
    return;
  }
  last_ms = now_ms;

  yuv_sig = app_sum_prefix_u32(src_yuv, src_len, APP_UVC_COLOR_AUDIT_SUM_BYTES);
  jpeg_sig = app_sum_prefix_u32(jpeg_bs, jpeg_len, APP_UVC_COLOR_AUDIT_SUM_BYTES);

  if (prev_valid != 0)
  {
    yuv_changed = (yuv_sig != prev_yuv_sig) ? 1 : 0;
    jpeg_changed = (jpeg_sig != prev_jpeg_sig) ? 1 : 0;
  }

  x_left = APP_UVC_ENC_WIDTH / 4;
  x_right = (3 * APP_UVC_ENC_WIDTH) / 4;
  y_mid = APP_UVC_ENC_HEIGHT / 2;

  if (app_read_yuv422_sample(src_yuv, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT, x_left, y_mid, &y0, &u0, &v0) == 0)
  {
    app_yuv_to_rgb_bt601_limited(y0, u0, v0, &r0, &g0, &b0);
    got_left = 1;
  }
  if (app_read_yuv422_sample(src_yuv, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT, x_right, y_mid, &y1, &u1, &v1) == 0)
  {
    app_yuv_to_rgb_bt601_limited(y1, u1, v1, &r1, &g1, &b1);
    got_right = 1;
  }

  app_jpeg_parse_meta(jpeg_bs, jpeg_len, &meta);
  jpeg_ok = app_jpeg_is_valid(&meta);

  printf("[ENC][AUDIT] yuv_sum%u=%lu jpg_sum%u=%lu ychg=%d jchg=%d jpg_ok=%d wh=%dx%d len=%d Lrgb=%d,%d,%d Rrgb=%d,%d,%d src=0x%08lX jpg=0x%08lX\r\n",
         APP_UVC_COLOR_AUDIT_SUM_BYTES,
         (unsigned long)yuv_sig,
         APP_UVC_COLOR_AUDIT_SUM_BYTES,
         (unsigned long)jpeg_sig,
         yuv_changed,
         jpeg_changed,
         jpeg_ok,
         meta.sof_w,
         meta.sof_h,
         jpeg_len,
         got_left ? r0 : -1, got_left ? g0 : -1, got_left ? b0 : -1,
         got_right ? r1 : -1, got_right ? g1 : -1, got_right ? b1 : -1,
         (unsigned long)src_yuv,
         (unsigned long)jpeg_bs);

  prev_yuv_sig = yuv_sig;
  prev_jpeg_sig = jpeg_sig;
  prev_valid = 1;
#else
  (void)src_yuv;
  (void)src_len;
  (void)jpeg_bs;
  (void)jpeg_len;
#endif
}

static uint32_t app_hash_fnv1a32(const uint8_t *buf, int len)
{
  uint32_t h = 2166136261UL;
  int i;

  if ((buf == NULL) || (len <= 0))
  {
    return 0U;
  }

  for (i = 0; i < len; i++)
  {
    h ^= (uint32_t)buf[i];
    h *= 16777619UL;
  }

  return h;
}

static int app_rgb_is_blue_like(int r, int g, int b)
{
  return (b > 180) && (r < 80) && (g < 100);
}

static int app_rgb_is_red_like(int r, int g, int b)
{
  return (r > 180) && (g < 100) && (b < 100);
}

static void app_dump_jpeg_full_hex(const char *name, const uint8_t *buf, int len)
{
#if APP_UVC_JPEG_FULL_DUMP_ENABLE
  int i;
  int j;
  int line_n;

  if ((name == NULL) || (buf == NULL) || (len <= 0))
  {
    return;
  }

  printf("[ENC][DUMP] %s len=%d begin\r\n", name, len);
  for (i = 0; i < len; i += APP_UVC_JPEG_FULL_DUMP_LINE_BYTES)
  {
    line_n = ((len - i) > APP_UVC_JPEG_FULL_DUMP_LINE_BYTES) ? APP_UVC_JPEG_FULL_DUMP_LINE_BYTES : (len - i);
    printf("[ENC][DUMP] %s +0x%04X:", name, i);
    for (j = 0; j < line_n; j++)
    {
      printf(" %02X", buf[i + j]);
    }
    printf("\r\n");
  }
  printf("[ENC][DUMP] %s end\r\n", name);
#else
  (void)name;
  (void)buf;
  (void)len;
#endif
}

static void app_uvc_verify_jpeg_buffers_1s(const uint8_t *src_yuv, int src_len,
                                           const uint8_t *jpeg_enc, const uint8_t *jpeg_uvc, int jpeg_len)
{
#if APP_UVC_JPEG_BUF_VERIFY_ENABLE
  static uint32_t last_ms;
  static uint32_t ref_hash;
  static int ref_len;
  static int ref_valid;
#if APP_UVC_JPEG_FULL_DUMP_ONCE
  static int full_dump_done;
#endif
  uint32_t now_ms = HAL_GetTick();
  app_jpeg_meta_t meta_enc;
  app_jpeg_meta_t meta_uvc;
  int enc_ok;
  int uvc_ok;
  int copy_eq;
  uint32_t hash_enc;
  uint32_t hash_uvc;
  int ref_eq;
  int yuv_ok = 0;
  int color_infer_ok;
  const char *color_label = "unknown";
  const char *uvc_label = "unknown";
  int x_left;
  int x_right;
  int y_mid;
  uint8_t y0, u0, v0;
  uint8_t y1, u1, v1;
  int r0 = 0, g0 = 0, b0 = 0;
  int r1 = 0, g1 = 0, b1 = 0;

  if ((src_yuv == NULL) || (jpeg_enc == NULL) || (jpeg_uvc == NULL) || (src_len <= 0) || (jpeg_len <= 0))
  {
    return;
  }
  if ((now_ms - last_ms) < APP_UVC_JPEG_BUF_VERIFY_PERIOD_MS)
  {
    return;
  }
  last_ms = now_ms;

  app_jpeg_parse_meta(jpeg_enc, jpeg_len, &meta_enc);
  app_jpeg_parse_meta(jpeg_uvc, jpeg_len, &meta_uvc);
  enc_ok = app_jpeg_is_valid(&meta_enc) &&
           (meta_enc.sof_w == APP_UVC_ENC_WIDTH) &&
           (meta_enc.sof_h == APP_UVC_ENC_HEIGHT);
  uvc_ok = app_jpeg_is_valid(&meta_uvc) &&
           (meta_uvc.sof_w == APP_UVC_ENC_WIDTH) &&
           (meta_uvc.sof_h == APP_UVC_ENC_HEIGHT);

  copy_eq = (memcmp(jpeg_enc, jpeg_uvc, (size_t)jpeg_len) == 0) ? 1 : 0;
  hash_enc = app_hash_fnv1a32(jpeg_enc, jpeg_len);
  hash_uvc = app_hash_fnv1a32(jpeg_uvc, jpeg_len);

  if (ref_valid == 0)
  {
    ref_valid = 1;
    ref_hash = hash_enc;
    ref_len = jpeg_len;
  }
  ref_eq = (ref_valid != 0) && (jpeg_len == ref_len) && (hash_enc == ref_hash);

  x_left = APP_UVC_ENC_WIDTH / 4;
  x_right = (3 * APP_UVC_ENC_WIDTH) / 4;
  y_mid = APP_UVC_ENC_HEIGHT / 2;
  if ((app_read_yuv422_sample(src_yuv, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT, x_left, y_mid, &y0, &u0, &v0) == 0) &&
      (app_read_yuv422_sample(src_yuv, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT, x_right, y_mid, &y1, &u1, &v1) == 0))
  {
    app_yuv_to_rgb_bt601_limited(y0, u0, v0, &r0, &g0, &b0);
    app_yuv_to_rgb_bt601_limited(y1, u1, v1, &r1, &g1, &b1);
    if (app_rgb_is_blue_like(r0, g0, b0) && app_rgb_is_red_like(r1, g1, b1))
    {
      yuv_ok = 1;
      color_label = "LEFT_BLUE_RIGHT_RED";
    }
    else if (app_rgb_is_red_like(r0, g0, b0) && app_rgb_is_blue_like(r1, g1, b1))
    {
      yuv_ok = 1;
      color_label = "LEFT_RED_RIGHT_BLUE";
    }
    else
    {
      yuv_ok = 0;
      color_label = "OTHER";
    }
  }

  color_infer_ok = enc_ok && uvc_ok && copy_eq && yuv_ok;
  if (copy_eq)
  {
    uvc_label = color_label;
  }
  else
  {
    uvc_label = "DIFF_FROM_ENC";
  }

  printf("[ENC][COLOR] venc_out=%s uvc_in=%s ok(enc=%d uvc=%d copy=%d ref=%d) wh=%dx%d len=%d hash=%08lX/%08lX infer=%d\r\n",
         color_label,
         uvc_label,
         enc_ok,
         uvc_ok,
         copy_eq,
         ref_eq,
         meta_enc.sof_w,
         meta_enc.sof_h,
         jpeg_len,
         (unsigned long)hash_enc,
         (unsigned long)hash_uvc,
         color_infer_ok);

#if APP_UVC_JPEG_FULL_DUMP_ENABLE
#if APP_UVC_JPEG_FULL_DUMP_ONCE
  if (full_dump_done == 0)
  {
    app_dump_jpeg_full_hex("venc_out_buffer", jpeg_enc, jpeg_len);
    app_dump_jpeg_full_hex("uvc_in_buffers", jpeg_uvc, jpeg_len);
    full_dump_done = 1;
  }
#else
  app_dump_jpeg_full_hex("venc_out_buffer", jpeg_enc, jpeg_len);
  app_dump_jpeg_full_hex("uvc_in_buffers", jpeg_uvc, jpeg_len);
#endif
#endif
#else
  (void)src_yuv;
  (void)src_len;
  (void)jpeg_enc;
  (void)jpeg_uvc;
  (void)jpeg_len;
#endif
}

static void app_uvc_clean_dcache_for_show(uint8_t *buf, int len)
{
#if APP_UVC_CLEAN_DCACHE_BEFORE_SHOW
  uintptr_t src_addr;
  uintptr_t clean_addr;
  uint32_t clean_len;

  if (len <= 0)
  {
    return;
  }

  src_addr = (uintptr_t)buf;
  clean_addr = src_addr & ~((uintptr_t)31U);
  clean_len = ALIGN_VALUE((uint32_t)((src_addr - clean_addr) + (uintptr_t)len), 32U);
  CACHE_OP(SCB_CleanDCache_by_Addr((uint8_t *)clean_addr, clean_len));
#else
  (void)buf;
  (void)len;
#endif
}

static void app_uvc_log_path_event(const char *tag, uint32_t seq, int idx, int len, int show_ret)
{
#if APP_UVC_PATH_DEBUG_ENABLE
  uint32_t now_ms = HAL_GetTick();
  int force_log = 0;

  if ((len <= 0) || (show_ret != 0))
  {
    force_log = 1;
  }
  if ((g_uvc_path_last_log_ms == 0U) || ((now_ms - g_uvc_path_last_log_ms) >= APP_UVC_PATH_DEBUG_PERIOD_MS))
  {
    force_log = 1;
  }

  if (force_log != 0)
  {
    printf("[UVC][PATH] %s seq=%lu idx=%d len=%d enc_ms=%lu send_ms=%lu show_ret=%d flying=%d sub=%lu rel=%lu drop=%lu warm=%lu thr_left=%lu thr_skip=%lu thr_keep=%lu\r\n",
           (tag != NULL) ? tag : "na",
           (unsigned long)seq,
           idx,
           len,
           (unsigned long)g_uvc_last_enc_ms,
           (unsigned long)g_uvc_last_send_ms,
           show_ret,
           buffer_flying,
           (unsigned long)g_uvc_tx_submit_id,
           (unsigned long)g_uvc_tx_release_id,
           (unsigned long)g_uvc_drop_busy_count,
           (unsigned long)g_uvc_start_warmup_left,
           (unsigned long)g_uvc_start_throttle_left,
           (unsigned long)g_uvc_start_throttle_skip_count,
           (unsigned long)g_uvc_start_throttle_keep_count);
    g_uvc_path_last_log_ms = now_ms;
  }
#else
  (void)tag;
  (void)seq;
  (void)idx;
  (void)len;
  (void)show_ret;
#endif
}

static void app_clean_dcache_for_hw_read(uint8_t *buf, int len)
{
  uintptr_t src_addr;
  uintptr_t clean_addr;
  uint32_t clean_len;

  if (len <= 0)
  {
    return;
  }

  src_addr = (uintptr_t)buf;
  clean_addr = src_addr & ~((uintptr_t)31U);
  clean_len = ALIGN_VALUE((uint32_t)((src_addr - clean_addr) + (uintptr_t)len), 32U);
  CACHE_OP(SCB_CleanDCache_by_Addr((uint8_t *)clean_addr, clean_len));
}

static void app_invalidate_dcache_for_cpu_read(uint8_t *buf, int len)
{
  uintptr_t src_addr;
  uintptr_t inval_addr;
  uint32_t inval_len;

  if (len <= 0)
  {
    return;
  }

  src_addr = (uintptr_t)buf;
  inval_addr = src_addr & ~((uintptr_t)31U);
  inval_len = ALIGN_VALUE((uint32_t)((src_addr - inval_addr) + (uintptr_t)len), 32U);
  CACHE_OP(SCB_InvalidateDCache_by_Addr((uint8_t *)inval_addr, inval_len));
}

static void app_uvc_log_src_1s(const uint8_t *buf, int len, uint32_t seq, int idx)
{
#if APP_UVC_TEST_PATTERN_MODE
  uint32_t now_ms;
  uint32_t sum;
  uint8_t b0 = 0U;
  uint8_t b1 = 0U;
  uint8_t b2 = 0U;
  uint8_t b3 = 0U;

  now_ms = HAL_GetTick();
  if ((now_ms - g_uvc_src_last_log_ms) < APP_UVC_SRC_LOG_PERIOD_MS)
  {
    return;
  }

  sum = app_sum_prefix_u32(buf, len, APP_ENC_DUMP_SUM_BYTES);
  if ((buf != NULL) && (len >= 4))
  {
    b0 = buf[0];
    b1 = buf[1];
    b2 = buf[2];
    b3 = buf[3];
  }

  printf("[UVC][SRC] seq=%lu idx=%d ptr=0x%08lX cap0=0x%08lX sum%u=%lu b0..b3=%02X %02X %02X %02X len=%d\r\n",
         (unsigned long)seq,
         idx,
         (unsigned long)buf,
         (unsigned long)capture_buffer[0],
         APP_ENC_DUMP_SUM_BYTES,
         (unsigned long)sum,
         b0, b1, b2, b3,
         len);
  g_uvc_src_last_log_ms = now_ms;
#else
  (void)buf;
  (void)len;
  (void)seq;
  (void)idx;
#endif
}

static void app_testpat_fill_white_yuyv(uint8_t *buf, int width, int height)
{
  int y;
  int x;

  if ((buf == NULL) || (width <= 0) || (height <= 0))
  {
    return;
  }

  for (y = 0; y < height; y++)
  {
    uint8_t *row = buf + ((size_t)y * (size_t)width * 2U);
    for (x = 0; x < width; x += 2)
    {
      row[0] = 235U;
      row[1] = 128U;
      row[2] = 235U;
      row[3] = 128U;
      row += 4;
    }
  }
}

static void app_testpat_set_y_yuyv(uint8_t *buf, int width, int height, int x, int y, uint8_t yv)
{
  uint8_t *pix;
  int x_even;

  if ((buf == NULL) || (width <= 0) || (height <= 0))
  {
    return;
  }
  if ((x < 0) || (x >= width) || (y < 0) || (y >= height))
  {
    return;
  }

  x_even = x & ~1;
  pix = buf + ((size_t)y * (size_t)width * 2U) + ((size_t)x_even * 2U);
  if ((x & 1) == 0)
  {
    pix[0] = yv;
  }
  else
  {
    pix[2] = yv;
  }
  pix[1] = 128U;
  pix[3] = 128U;
}

static void app_testpat_fill_rect_yuyv(uint8_t *buf, int width, int height,
                                       int x0, int y0, int rw, int rh, uint8_t yv)
{
  int x1;
  int y1;
  int x;
  int y;

  if ((rw <= 0) || (rh <= 0))
  {
    return;
  }

  if (x0 < 0)
  {
    rw += x0;
    x0 = 0;
  }
  if (y0 < 0)
  {
    rh += y0;
    y0 = 0;
  }
  x1 = x0 + rw;
  y1 = y0 + rh;
  if (x1 > width)
  {
    x1 = width;
  }
  if (y1 > height)
  {
    y1 = height;
  }
  if ((x0 >= x1) || (y0 >= y1))
  {
    return;
  }

  for (y = y0; y < y1; y++)
  {
    for (x = x0; x < x1; x++)
    {
      app_testpat_set_y_yuyv(buf, width, height, x, y, yv);
    }
  }
}

static void app_testpat_build_frame_yuyv(uint8_t *buf, int width, int height)
{
  int y;
  int x;
  int half;
  uint8_t y_blue = 41U;
  uint8_t u_blue = 240U;
  uint8_t v_blue = 110U;
  uint8_t y_red = 81U;
  uint8_t u_red = 90U;
  uint8_t v_red = 240U;

  if ((buf == NULL) || (width <= 0) || (height <= 0))
  {
    return;
  }

  half = width / 2;
  for (y = 0; y < height; y++)
  {
    uint8_t *row = buf + ((size_t)y * (size_t)width * 2U);
    for (x = 0; x < width; x += 2)
    {
      if (x < half)
      {
#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
        row[0] = y_blue;
        row[1] = u_blue;
        row[2] = y_blue;
        row[3] = v_blue;
#else
        row[0] = u_blue;
        row[1] = y_blue;
        row[2] = v_blue;
        row[3] = y_blue;
#endif
      }
      else
      {
#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
        row[0] = y_red;
        row[1] = u_red;
        row[2] = y_red;
        row[3] = v_red;
#else
        row[0] = u_red;
        row[1] = y_red;
        row[2] = v_red;
        row[3] = y_red;
#endif
      }
      row += 4;
    }
  }
}

static void app_testpat_build_frame_from_prebuilt_yuyv(uint8_t *dst, int dst_w, int dst_h)
{
#if APP_UVC_TEST_PREBUILT_YUYV_MODE
  const uint8_t *src = g_app_uvc_test_yuyv_320x240;
  const int src_w = 320;
  const int src_h = 240;
  int y;
  int x;

  if ((dst == NULL) || (dst_w <= 0) || (dst_h <= 0))
  {
    return;
  }

  if ((dst_w == src_w) && (dst_h == src_h))
  {
    memcpy(dst, src, (size_t)src_w * (size_t)src_h * 2U);
    return;
  }

  /*
   * For full-HD debug path, synthesize YUV directly at target resolution
   * (avoid upscaling artifacts from 320x240 prebuilt frame).
   */
  if ((dst_w == 1920) && (dst_h == 1080))
  {
    int yy;
    uint8_t *dst_row = dst;
    for (yy = 0; yy < dst_h; yy++)
    {
      memcpy(dst_row, g_app_uvc_test_yuyv_1920x1080_row, g_app_uvc_test_yuyv_1920x1080_row_len);
      dst_row += g_app_uvc_test_yuyv_1920x1080_row_len;
    }
    return;
  }

  for (y = 0; y < dst_h; y++)
  {
    int sy = (y * src_h) / dst_h;
    const uint8_t *src_row = src + ((size_t)sy * (size_t)src_w * 2U);
    uint8_t *dst_row = dst + ((size_t)y * (size_t)dst_w * 2U);

    for (x = 0; x < dst_w; x += 2)
    {
      int sx = ((x * src_w) / dst_w) & ~1;
      const uint8_t *sp = src_row + ((size_t)sx * 2U);
#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
      dst_row[0] = sp[0];
      dst_row[1] = sp[1];
      dst_row[2] = sp[2];
      dst_row[3] = sp[3];
#else
      dst_row[0] = sp[0];
      dst_row[1] = sp[1];
      dst_row[2] = sp[2];
      dst_row[3] = sp[3];
#endif
      dst_row += 4;
    }
  }
#else
  (void)dst;
  (void)dst_w;
  (void)dst_h;
#endif
}

static int app_testpat_reload_capture0_from_header(void)
{
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_PREBUILT_YUYV_MODE && !APP_UVC_TEST_FORCE_DEDICATED_SRC
  uint8_t *dst = capture_buffer[0];
  const uint32_t expect_len = (uint32_t)APP_UVC_ENC_WIDTH * (uint32_t)APP_UVC_ENC_HEIGHT * (uint32_t)CAPTURE_BPP;
  static uint8_t s_checked_once = 0U;
  int ok = 1;

  if ((APP_UVC_ENC_WIDTH == 320) && (APP_UVC_ENC_HEIGHT == 240))
  {
    if (g_app_uvc_test_yuyv_320x240_len != expect_len)
    {
      ok = 0;
    }
    else
    {
      memcpy(dst, g_app_uvc_test_yuyv_320x240, (size_t)expect_len);
    }
  }
  else if ((APP_UVC_ENC_WIDTH == 1920) && (APP_UVC_ENC_HEIGHT == 1080))
  {
    if ((g_app_uvc_test_yuyv_1920x1080_len != expect_len) ||
        (g_app_uvc_test_yuyv_1920x1080_row_len != ((uint32_t)APP_UVC_ENC_WIDTH * 2U)))
    {
      ok = 0;
    }
    else
    {
      int y;
      uint8_t *row = dst;
      for (y = 0; y < APP_UVC_ENC_HEIGHT; y++)
      {
        memcpy(row, g_app_uvc_test_yuyv_1920x1080_row, g_app_uvc_test_yuyv_1920x1080_row_len);
        row += g_app_uvc_test_yuyv_1920x1080_row_len;
      }
    }
  }
  else
  {
    /* Fallback for non-prebuilt sizes. */
    app_testpat_build_frame_from_prebuilt_yuyv(dst, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
  }

  if (ok != 0)
  {
    app_clean_dcache_for_hw_read(dst, (int)expect_len);
  }

  if (s_checked_once == 0U)
  {
    s_checked_once = 1U;
    printf("[UVC][TEST] capture_buffer[0] source=app_uvc_test_yuyv.h wh=%dx%d expect=%lu src320=%lu src1080=%lu row1080=%lu ok=%d\r\n",
           APP_UVC_ENC_WIDTH,
           APP_UVC_ENC_HEIGHT,
           (unsigned long)expect_len,
           (unsigned long)g_app_uvc_test_yuyv_320x240_len,
           (unsigned long)g_app_uvc_test_yuyv_1920x1080_len,
           (unsigned long)g_app_uvc_test_yuyv_1920x1080_row_len,
           ok);
  }

  return (ok != 0) ? 0 : -1;
#else
  return 0;
#endif
}

#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_PREBUILT_YUYV_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC && !APP_UVC_TEST_STATIC_JPEG_MODE
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
static int app_psram_ptr_to_off(const uint8_t *ptr, uint32_t *off)
{
  uintptr_t a;
  if ((ptr == NULL) || (off == NULL))
  {
    return -1;
  }
  a = (uintptr_t)ptr;
  if ((a < XSPI1_BASE) || (a >= (XSPI1_BASE + 0x02000000UL)))
  {
    return -1;
  }
  *off = (uint32_t)(a - XSPI1_BASE);
  return 0;
}

static int app_psram_indirect_write(uint32_t off, const uint8_t *src, uint32_t len)
{
  int32_t ret;
  int was_mmp = 0;
  uint32_t pos = 0U;

  if ((src == NULL) || (len == 0U))
  {
    return -1;
  }

  ret = BSP_XSPI_RAM_DisableMemoryMappedMode(0U);
  if (ret == BSP_ERROR_NONE)
  {
    was_mmp = 1;
  }
  else if (ret != BSP_ERROR_XSPI_MMP_UNLOCK_FAILURE)
  {
    return -1;
  }

  while (pos < len)
  {
    uint32_t chunk = len - pos;
    if (chunk > 4096U)
    {
      chunk = 4096U;
    }
    if (BSP_XSPI_RAM_Write(0U, (uint8_t *)&src[pos], off + pos, chunk) != BSP_ERROR_NONE)
    {
      if (was_mmp != 0)
      {
        (void)BSP_XSPI_RAM_EnableMemoryMappedMode(0U);
      }
      return -1;
    }
    pos += chunk;
  }

  if ((was_mmp != 0) && (BSP_XSPI_RAM_EnableMemoryMappedMode(0U) != BSP_ERROR_NONE))
  {
    return -1;
  }
  return 0;
}

static int app_psram_indirect_read(uint32_t off, uint8_t *dst, uint32_t len)
{
  int32_t ret;
  int was_mmp = 0;
  uint32_t pos = 0U;

  if ((dst == NULL) || (len == 0U))
  {
    return -1;
  }

  ret = BSP_XSPI_RAM_DisableMemoryMappedMode(0U);
  if (ret == BSP_ERROR_NONE)
  {
    was_mmp = 1;
  }
  else if (ret != BSP_ERROR_XSPI_MMP_UNLOCK_FAILURE)
  {
    return -1;
  }

  while (pos < len)
  {
    uint32_t chunk = len - pos;
    if (chunk > 4096U)
    {
      chunk = 4096U;
    }
    if (BSP_XSPI_RAM_Read(0U, &dst[pos], off + pos, chunk) != BSP_ERROR_NONE)
    {
      if (was_mmp != 0)
      {
        (void)BSP_XSPI_RAM_EnableMemoryMappedMode(0U);
      }
      return -1;
    }
    pos += chunk;
  }

  if ((was_mmp != 0) && (BSP_XSPI_RAM_EnableMemoryMappedMode(0U) != BSP_ERROR_NONE))
  {
    return -1;
  }
  return 0;
}

static int app_testpat_pull_dedicated_psram_to_axi(int len)
{
  uint32_t off = 0U;
  uint8_t *psram_src = app_get_test_src_cpu_buffer();
  if ((psram_src == NULL) || (len <= 0))
  {
    return -1;
  }
  if (app_psram_ptr_to_off(psram_src, &off) != 0)
  {
    return -1;
  }
  if (app_psram_indirect_read(off, g_uvc_test_src_axi_staging, (uint32_t)len) != 0)
  {
    return -1;
  }
  app_invalidate_dcache_for_cpu_read(g_uvc_test_src_axi_staging, len);
  return 0;
}
#endif

static int app_testpat_copy_header_to_dedicated(uint8_t *dst, uint32_t expect_len)
{
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  uint32_t dst_off = 0U;
#endif
  if ((dst == NULL) || (expect_len == 0U))
  {
    return -1;
  }

#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  if (app_psram_ptr_to_off(dst, &dst_off) != 0)
  {
    return -1;
  }
#endif

  if ((APP_UVC_ENC_WIDTH == 320) && (APP_UVC_ENC_HEIGHT == 240))
  {
    if (g_app_uvc_test_yuyv_320x240_len != expect_len)
    {
      return -1;
    }
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
    if (app_psram_indirect_write(dst_off, g_app_uvc_test_yuyv_320x240, expect_len) != 0)
    {
      return -1;
    }
#else
    memcpy(dst, g_app_uvc_test_yuyv_320x240, (size_t)expect_len);
#endif
    return 0;
  }

  if ((APP_UVC_ENC_WIDTH == 1920) && (APP_UVC_ENC_HEIGHT == 1080))
  {
    if ((g_app_uvc_test_yuyv_1920x1080_len != expect_len) ||
        (g_app_uvc_test_yuyv_1920x1080_row_len != ((uint32_t)APP_UVC_ENC_WIDTH * 2U)))
    {
      return -1;
    }
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
    {
      uint32_t y;
      uint32_t row_len = g_app_uvc_test_yuyv_1920x1080_row_len;
      for (y = 0U; y < (uint32_t)APP_UVC_ENC_HEIGHT; y++)
      {
        if (app_psram_indirect_write(dst_off + (y * row_len), g_app_uvc_test_yuyv_1920x1080_row, row_len) != 0)
        {
          return -1;
        }
      }
    }
#else
    {
      int y;
      uint8_t *row = dst;
      for (y = 0; y < APP_UVC_ENC_HEIGHT; y++)
      {
        memcpy(row, g_app_uvc_test_yuyv_1920x1080_row, g_app_uvc_test_yuyv_1920x1080_row_len);
        row += g_app_uvc_test_yuyv_1920x1080_row_len;
      }
    }
#endif
    return 0;
  }

  /* Fallback for non-prebuilt sizes. */
  app_testpat_build_frame_from_prebuilt_yuyv(dst, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
  return 0;
}

static int app_testpat_reload_dedicated_src_from_header(void)
{
  uint8_t *dst = app_get_test_src_cpu_buffer();
  const uint32_t expect_len = (uint32_t)APP_UVC_ENC_WIDTH * (uint32_t)APP_UVC_ENC_HEIGHT * (uint32_t)CAPTURE_BPP;
  static uint8_t s_fixed_psram_log_once = 0U;
  uint32_t mismatch_off = 0U;
  uint8_t got = 0U;
  uint8_t exp = 0U;
  int cmp_ret;

  if ((dst == NULL) || (expect_len == 0U))
  {
    return -1;
  }

#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE
  if ((((uintptr_t)dst) < XSPI1_BASE) || ((((uintptr_t)dst) + (uintptr_t)expect_len) > (XSPI1_BASE + 0x02000000UL)))
  {
    return -1;
  }
  if (s_fixed_psram_log_once == 0U)
  {
    s_fixed_psram_log_once = 1U;
    printf("[UVC][TEST] dedicated_src PSRAM fixed-window addr=0x%08lX len=%lu (capture_buffer unused)\r\n",
           (unsigned long)dst,
           (unsigned long)expect_len);
  }
#endif

  if (app_testpat_copy_header_to_dedicated(dst, expect_len) != 0)
  {
    return -1;
  }

#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  {
    uint32_t dst_off = 0U;
    if (app_psram_ptr_to_off(dst, &dst_off) != 0)
    {
      return -1;
    }
    if (app_psram_indirect_read(dst_off, g_uvc_test_src_axi_staging, expect_len) != 0)
    {
      printf("[RAM][LOAD] FAIL reason=psram_indirect_read dst=0x%08lX len=%lu\r\n",
             (unsigned long)dst,
             (unsigned long)expect_len);
      return -1;
    }
    cmp_ret = app_psram_src_compare_against_header(g_uvc_test_src_axi_staging, (int)expect_len, &mismatch_off, &got, &exp);
    if (cmp_ret != 0)
    {
      printf("[RAM][LOAD] FAIL off=%lu got=%02X exp=%02X dst=0x%08lX b0..b3=%02X %02X %02X %02X\r\n",
             (unsigned long)mismatch_off,
             got,
             exp,
             (unsigned long)dst,
             g_uvc_test_src_axi_staging[0], g_uvc_test_src_axi_staging[1],
             g_uvc_test_src_axi_staging[2], g_uvc_test_src_axi_staging[3]);
      return -1;
    }
    printf("[RAM][LOAD] OK dst=0x%08lX b0..b3=%02X %02X %02X %02X len=%lu\r\n",
           (unsigned long)dst,
           g_uvc_test_src_axi_staging[0], g_uvc_test_src_axi_staging[1],
           g_uvc_test_src_axi_staging[2], g_uvc_test_src_axi_staging[3],
           (unsigned long)expect_len);
    return 0;
  }
#else
  app_clean_dcache_for_hw_read(dst, (int)expect_len);
  app_invalidate_dcache_for_cpu_read(dst, (int)expect_len);
  cmp_ret = app_psram_src_compare_against_header(dst, (int)expect_len, &mismatch_off, &got, &exp);
  if (cmp_ret != 0)
  {
    printf("[RAM][LOAD] FAIL off=%lu got=%02X exp=%02X dst=0x%08lX b0..b3=%02X %02X %02X %02X\r\n",
           (unsigned long)mismatch_off,
           got,
           exp,
           (unsigned long)dst,
           dst[0], dst[1], dst[2], dst[3]);
    return -1;
  }
  printf("[RAM][LOAD] OK dst=0x%08lX b0..b3=%02X %02X %02X %02X len=%lu\r\n",
         (unsigned long)dst,
         dst[0], dst[1], dst[2], dst[3],
         (unsigned long)expect_len);
  return 0;
#endif
}
#else
static int app_testpat_reload_dedicated_src_from_header(void)
{
  return 0;
}
#endif

#if APP_PSRAM_SRC_GUARD_ENABLE && APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_PREBUILT_YUYV_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC
static int app_psram_src_compare_against_header(const uint8_t *src,
                                                int len,
                                                uint32_t *mismatch_off,
                                                uint8_t *got,
                                                uint8_t *exp)
{
  uint32_t i;
  uint32_t expected_len;

  if ((src == NULL) || (len <= 0))
  {
    return -1;
  }

  expected_len = (uint32_t)APP_UVC_ENC_WIDTH * (uint32_t)APP_UVC_ENC_HEIGHT * (uint32_t)CAPTURE_BPP;
  if ((uint32_t)len != expected_len)
  {
    return -1;
  }

  if ((APP_UVC_ENC_WIDTH == 320) && (APP_UVC_ENC_HEIGHT == 240))
  {
    if (g_app_uvc_test_yuyv_320x240_len != expected_len)
    {
      return -1;
    }

    for (i = 0; i < expected_len; i++)
    {
      if (src[i] != g_app_uvc_test_yuyv_320x240[i])
      {
        if (mismatch_off != NULL)
        {
          *mismatch_off = i;
        }
        if (got != NULL)
        {
          *got = src[i];
        }
        if (exp != NULL)
        {
          *exp = g_app_uvc_test_yuyv_320x240[i];
        }
        return -1;
      }
    }
    return 0;
  }

  if ((APP_UVC_ENC_WIDTH == 1920) && (APP_UVC_ENC_HEIGHT == 1080))
  {
    uint32_t y;
    uint32_t row_len = (uint32_t)APP_UVC_ENC_WIDTH * 2U;
    if ((g_app_uvc_test_yuyv_1920x1080_len != expected_len) ||
        (g_app_uvc_test_yuyv_1920x1080_row_len != row_len))
    {
      return -1;
    }

    for (y = 0; y < (uint32_t)APP_UVC_ENC_HEIGHT; y++)
    {
      const uint8_t *row = src + (y * row_len);
      uint32_t x;
      for (x = 0; x < row_len; x++)
      {
        if (row[x] != g_app_uvc_test_yuyv_1920x1080_row[x])
        {
          uint32_t off = (y * row_len) + x;
          if (mismatch_off != NULL)
          {
            *mismatch_off = off;
          }
          if (got != NULL)
          {
            *got = row[x];
          }
          if (exp != NULL)
          {
            *exp = g_app_uvc_test_yuyv_1920x1080_row[x];
          }
          return -1;
        }
      }
    }
    return 0;
  }

  /*
   * Generic path for non-prebuilt sizes (e.g. 960x540):
   * expected frame is generated by nearest-neighbor upscale from 320x240 source
   * in app_testpat_build_frame_from_prebuilt_yuyv().
   */
  {
    const uint32_t src_w = 320U;
    const uint32_t src_h = 240U;
    const uint32_t src_row_len = src_w * 2U;
    const uint32_t dst_w = (uint32_t)APP_UVC_ENC_WIDTH;
    const uint32_t dst_h = (uint32_t)APP_UVC_ENC_HEIGHT;
    const uint32_t dst_row_len = dst_w * 2U;
    uint32_t y;

    if ((dst_w == 0U) || (dst_h == 0U) || ((dst_w & 1U) != 0U))
    {
      return -1;
    }
    if (g_app_uvc_test_yuyv_320x240_len != (src_w * src_h * 2U))
    {
      return -1;
    }

    for (y = 0U; y < dst_h; y++)
    {
      uint32_t sy = (y * src_h) / dst_h;
      const uint8_t *src_row = g_app_uvc_test_yuyv_320x240 + (sy * src_row_len);
      const uint8_t *dst_row = src + (y * dst_row_len);
      uint32_t x;

      for (x = 0U; x < dst_w; x += 2U)
      {
        uint32_t sx = ((x * src_w) / dst_w) & ~1U;
        const uint8_t *exp4 = src_row + (sx * 2U);
        const uint8_t *got4 = dst_row + (x * 2U);
        uint32_t b;

        for (b = 0U; b < 4U; b++)
        {
          if (got4[b] != exp4[b])
          {
            uint32_t off = (y * dst_row_len) + (x * 2U) + b;
            if (mismatch_off != NULL)
            {
              *mismatch_off = off;
            }
            if (got != NULL)
            {
              *got = got4[b];
            }
            if (exp != NULL)
            {
              *exp = exp4[b];
            }
            return -1;
          }
        }
      }
    }
  }

  return 0;
}

static void app_psram_src_guard_verify_1s(uint8_t *src_buf, int len)
{
  uint32_t now_ms;
  uint32_t mismatch_off = 0U;
  uint8_t got = 0U;
  uint8_t exp = 0U;
  uint32_t hash = 0U;
  int cmp_ret;
  static uint8_t s_addr_logged_once = 0U;
  static uint32_t s_ok_last_log_ms = 0U;
  static uint32_t s_last_hash = 0U;
  static uint32_t s_stable_count = 0U;

  if ((src_buf == NULL) || (len <= 0))
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((g_psram_src_guard_last_ms != 0U) &&
      ((now_ms - g_psram_src_guard_last_ms) < APP_PSRAM_SRC_GUARD_PERIOD_MS))
  {
    return;
  }
  g_psram_src_guard_last_ms = now_ms;

  if (s_addr_logged_once == 0U)
  {
    uint8_t *dedicated = app_get_test_src_buffer();
    uint8_t *dedicated_mmp = app_psram_mmp_alias(dedicated);
    s_addr_logged_once = 1U;
    printf("[PSRAM][YUV] addr src=0x%08lX dedicated=0x%08lX dedicated_mmp=0x%08lX cap0=0x%08lX eq(src,ded)=%d eq(src,ded_mmp)=%d eq(src,cap0)=%d len=%d\r\n",
           (unsigned long)src_buf,
           (unsigned long)dedicated,
           (unsigned long)dedicated_mmp,
           (unsigned long)capture_buffer[0],
           (src_buf == dedicated) ? 1 : 0,
           (src_buf == dedicated_mmp) ? 1 : 0,
           (src_buf == capture_buffer[0]) ? 1 : 0,
           len);
  }

#if APP_PSRAM_ALIAS_DEBUG_ENABLE
  if (len >= 64)
  {
    uint8_t *src_mmp = app_psram_mmp_alias(src_buf);
    uint32_t sum_91 = app_sum_prefix_u32(src_buf, len, 64);
    uint32_t sum_90 = app_sum_prefix_u32(src_mmp, len, 64);
    int eq64 = (memcmp(src_buf, src_mmp, 64U) == 0) ? 1 : 0;
    printf("[PSRAM][ALIAS] src91=0x%08lX src90=0x%08lX b91=%02X b90=%02X sum64_91=%lu sum64_90=%lu eq64=%d\r\n",
           (unsigned long)src_buf,
           (unsigned long)src_mmp,
           src_buf[0],
           src_mmp[0],
           (unsigned long)sum_91,
           (unsigned long)sum_90,
           eq64);
  }
#endif

  /* Validate source data. For fixed PSRAM window, use indirect read into AXI staging. */
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  {
    uint32_t src_off = 0U;
    if (app_psram_ptr_to_off(src_buf, &src_off) != 0)
    {
      g_psram_src_guard_bad_count++;
      printf("[RAM][SRC] BAD cnt=%lu reason=ptr_to_off_failed src=0x%08lX\r\n",
             (unsigned long)g_psram_src_guard_bad_count,
             (unsigned long)src_buf);
      return;
    }
    if (app_psram_indirect_read(src_off, g_uvc_test_src_axi_staging, (uint32_t)len) != 0)
    {
      g_psram_src_guard_bad_count++;
      printf("[RAM][SRC] BAD cnt=%lu reason=indirect_read_failed src=0x%08lX\r\n",
             (unsigned long)g_psram_src_guard_bad_count,
             (unsigned long)src_buf);
      return;
    }
    cmp_ret = app_psram_src_compare_against_header(g_uvc_test_src_axi_staging, len, &mismatch_off, &got, &exp);
    hash = app_hash_fnv1a32(g_uvc_test_src_axi_staging, len);
  }
#else
  app_clean_dcache_for_hw_read(src_buf, len);
  app_invalidate_dcache_for_cpu_read(src_buf, len);
  cmp_ret = app_psram_src_compare_against_header(src_buf, len, &mismatch_off, &got, &exp);
  hash = app_hash_fnv1a32(src_buf, len);
#endif
  if (cmp_ret == 0)
  {
    g_psram_src_guard_ok_count++;
    if ((g_psram_src_guard_ok_count == 1U) || (hash != s_last_hash))
    {
      s_stable_count = 1U;
      s_last_hash = hash;
      s_ok_last_log_ms = now_ms;
      printf("[RAM][SRC] ok hash=0x%08lX stable=%lu len=%d src=0x%08lX b0..b3=%02X %02X %02X %02X\r\n",
             (unsigned long)hash,
             (unsigned long)s_stable_count,
             len,
             (unsigned long)src_buf,
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
             g_uvc_test_src_axi_staging[0], g_uvc_test_src_axi_staging[1],
             g_uvc_test_src_axi_staging[2], g_uvc_test_src_axi_staging[3]);
#else
             src_buf[0], src_buf[1], src_buf[2], src_buf[3]);
#endif
    }
    else
    {
      s_stable_count++;
      if ((now_ms - s_ok_last_log_ms) >= 5000U)
      {
        s_ok_last_log_ms = now_ms;
        printf("[RAM][SRC] stable hash=0x%08lX stable=%lu ok=%lu bad=%lu src=0x%08lX\r\n",
               (unsigned long)hash,
               (unsigned long)s_stable_count,
               (unsigned long)g_psram_src_guard_ok_count,
               (unsigned long)g_psram_src_guard_bad_count,
               (unsigned long)src_buf);
      }
    }
    return;
  }

  g_psram_src_guard_bad_count++;
  s_stable_count = 0U;
  s_last_hash = hash;
  printf("[RAM][SRC] BAD cnt=%lu off=%lu got=%02X exp=%02X hash=0x%08lX src=0x%08lX\r\n",
         (unsigned long)g_psram_src_guard_bad_count,
         (unsigned long)mismatch_off,
         got,
         exp,
         (unsigned long)hash,
         (unsigned long)src_buf);

  /* Self-recover source buffer to keep encoder input deterministic. */
  if (app_testpat_copy_header_to_dedicated(src_buf, (uint32_t)len) != 0)
  {
    g_psram_src_guard_recover_fail_count++;
    printf("[RAM][SRC] recover=FAIL cnt=%lu reason=copy_header_to_psram_failed\r\n",
           (unsigned long)g_psram_src_guard_recover_fail_count);
    return;
  }
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  {
    uint32_t src_off = 0U;
    if ((app_psram_ptr_to_off(src_buf, &src_off) != 0) ||
        (app_psram_indirect_read(src_off, g_uvc_test_src_axi_staging, (uint32_t)len) != 0))
    {
      g_psram_src_guard_recover_fail_count++;
      printf("[RAM][SRC] recover=FAIL cnt=%lu reason=indirect_read_failed\r\n",
             (unsigned long)g_psram_src_guard_recover_fail_count);
      return;
    }
    cmp_ret = app_psram_src_compare_against_header(g_uvc_test_src_axi_staging, len, &mismatch_off, &got, &exp);
    hash = app_hash_fnv1a32(g_uvc_test_src_axi_staging, len);
  }
#else
  app_clean_dcache_for_hw_read(src_buf, len);
  app_invalidate_dcache_for_cpu_read(src_buf, len);
  cmp_ret = app_psram_src_compare_against_header(src_buf, len, &mismatch_off, &got, &exp);
  hash = app_hash_fnv1a32(src_buf, len);
#endif
  if (cmp_ret == 0)
  {
    g_psram_src_guard_recover_ok_count++;
    printf("[RAM][SRC] recover=OK cnt=%lu hash=0x%08lX\r\n",
           (unsigned long)g_psram_src_guard_recover_ok_count,
           (unsigned long)hash);
  }
  else
  {
    g_psram_src_guard_recover_fail_count++;
    printf("[RAM][SRC] recover=FAIL cnt=%lu off=%lu got=%02X exp=%02X hash=0x%08lX\r\n",
           (unsigned long)g_psram_src_guard_recover_fail_count,
           (unsigned long)mismatch_off,
           got,
           exp,
           (unsigned long)hash);
  }
}
#else
static void app_psram_src_guard_verify_1s(uint8_t *src_buf, int len)
{
  (void)src_buf;
  (void)len;
}
#endif

/* Software-only custom JPEG encoder for test mode (no ENC_EncodeFrame, no prebuilt JPEG headers). */
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_CUSTOM_ENCODER_MODE
typedef struct
{
  uint16_t code[256];
  uint8_t size[256];
} app_swjpg_huff_t;

typedef struct
{
  uint8_t *buf;
  int max;
  int pos;
  uint32_t bitbuf;
  int bitcnt;
  int overflow;
} app_swjpg_bw_t;

static const uint8_t g_swjpg_zigzag[64] = {
  0, 1, 8, 16, 9, 2, 3, 10,
  17, 24, 32, 25, 18, 11, 4, 5,
  12, 19, 26, 33, 40, 48, 41, 34,
  27, 20, 13, 6, 7, 14, 21, 28,
  35, 42, 49, 56, 57, 50, 43, 36,
  29, 22, 15, 23, 30, 37, 44, 51,
  58, 59, 52, 45, 38, 31, 39, 46,
  53, 60, 61, 54, 47, 55, 62, 63
};

static uint8_t g_swjpg_bits_dc[2][16];
static uint8_t g_swjpg_bits_ac[2][16];
static uint8_t g_swjpg_vals_dc[2][12];
static uint8_t g_swjpg_vals_ac[2][162];
static app_swjpg_huff_t g_swjpg_dc_huff[2];
static app_swjpg_huff_t g_swjpg_ac_huff[2];
static float g_swjpg_cos[8][8];
static uint8_t g_swjpg_init_done;

static void app_swjpg_build_huff(const uint8_t bits[16], const uint8_t *vals, app_swjpg_huff_t *tbl)
{
  int i;
  int j;
  int k = 0;
  uint16_t code = 0U;

  memset(tbl, 0, sizeof(*tbl));
  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < bits[i]; j++)
    {
      uint8_t sym = vals[k++];
      tbl->code[sym] = code;
      tbl->size[sym] = (uint8_t)(i + 1);
      code++;
    }
    code <<= 1;
  }
}

static void app_swjpg_init_tables(void)
{
  int i;
  int x;
  int u;
  const float pi_over_16 = 0.1963495408493621f; /* PI/16 */

  if (g_swjpg_init_done != 0U)
  {
    return;
  }

  for (i = 0; i < 16; i++)
  {
    g_swjpg_bits_dc[0][i] = (uint8_t)Dc_Li[i].DcLumLi;
    g_swjpg_bits_dc[1][i] = (uint8_t)Dc_Li[i].DcChromLi;
    g_swjpg_bits_ac[0][i] = (uint8_t)Ac_Li[i].AcLumLi;
    g_swjpg_bits_ac[1][i] = (uint8_t)Ac_Li[i].AcChromLi;
  }
  for (i = 0; i < 12; i++)
  {
    g_swjpg_vals_dc[0][i] = (uint8_t)Vij_Dc[i].DcLumVij;
    g_swjpg_vals_dc[1][i] = (uint8_t)Vij_Dc[i].DcChromVij;
  }
  for (i = 0; i < 162; i++)
  {
    g_swjpg_vals_ac[0][i] = (uint8_t)Vij_Ac[i].AcLumVij;
    g_swjpg_vals_ac[1][i] = (uint8_t)Vij_Ac[i].AcChromVij;
  }

  app_swjpg_build_huff(g_swjpg_bits_dc[0], g_swjpg_vals_dc[0], &g_swjpg_dc_huff[0]);
  app_swjpg_build_huff(g_swjpg_bits_dc[1], g_swjpg_vals_dc[1], &g_swjpg_dc_huff[1]);
  app_swjpg_build_huff(g_swjpg_bits_ac[0], g_swjpg_vals_ac[0], &g_swjpg_ac_huff[0]);
  app_swjpg_build_huff(g_swjpg_bits_ac[1], g_swjpg_vals_ac[1], &g_swjpg_ac_huff[1]);

  for (x = 0; x < 8; x++)
  {
    for (u = 0; u < 8; u++)
    {
      g_swjpg_cos[x][u] = cosf((2.0f * (float)x + 1.0f) * (float)u * pi_over_16);
    }
  }

  g_swjpg_init_done = 1U;
}

static void app_swjpg_bw_put_byte(app_swjpg_bw_t *bw, uint8_t b, int entropy)
{
  if (bw->overflow != 0)
  {
    return;
  }

  if (bw->pos >= bw->max)
  {
    bw->overflow = 1;
    return;
  }
  bw->buf[bw->pos++] = b;

  if ((entropy != 0) && (b == 0xFFU))
  {
    if (bw->pos >= bw->max)
    {
      bw->overflow = 1;
      return;
    }
    bw->buf[bw->pos++] = 0x00U;
  }
}

static void app_swjpg_bw_put_word(app_swjpg_bw_t *bw, uint16_t w)
{
  app_swjpg_bw_put_byte(bw, (uint8_t)(w >> 8), 0);
  app_swjpg_bw_put_byte(bw, (uint8_t)(w & 0xFFU), 0);
}

static void app_swjpg_bw_put_bits(app_swjpg_bw_t *bw, uint16_t bits, int nbits)
{
  bw->bitbuf = (bw->bitbuf << nbits) | (uint32_t)(bits & ((1U << nbits) - 1U));
  bw->bitcnt += nbits;

  while (bw->bitcnt >= 8)
  {
    int shift = bw->bitcnt - 8;
    uint8_t byte = (uint8_t)((bw->bitbuf >> shift) & 0xFFU);
    app_swjpg_bw_put_byte(bw, byte, 1);
    bw->bitcnt -= 8;
    if (bw->bitcnt > 0)
    {
      bw->bitbuf &= (1U << bw->bitcnt) - 1U;
    }
    else
    {
      bw->bitbuf = 0U;
    }
  }
}

static void app_swjpg_bw_flush_bits(app_swjpg_bw_t *bw)
{
  if (bw->bitcnt > 0)
  {
    uint8_t byte = (uint8_t)((bw->bitbuf << (8 - bw->bitcnt)) | ((1U << (8 - bw->bitcnt)) - 1U));
    app_swjpg_bw_put_byte(bw, byte, 1);
    bw->bitcnt = 0;
    bw->bitbuf = 0U;
  }
}

static int app_swjpg_cat(int v)
{
  int a = (v < 0) ? -v : v;
  int n = 0;
  while (a != 0)
  {
    a >>= 1;
    n++;
  }
  return n;
}

static uint16_t app_swjpg_amp_bits(int v, int cat)
{
  if (cat == 0)
  {
    return 0U;
  }
  if (v >= 0)
  {
    return (uint16_t)v;
  }
  return (uint16_t)(((1 << cat) - 1) + v);
}

static int app_swjpg_emit_huff(app_swjpg_bw_t *bw, const app_swjpg_huff_t *tbl, uint8_t sym)
{
  uint8_t nbits = tbl->size[sym];
  if (nbits == 0U)
  {
    return -1;
  }
  app_swjpg_bw_put_bits(bw, tbl->code[sym], nbits);
  return (bw->overflow == 0) ? 0 : -1;
}

static int app_swjpg_encode_block(app_swjpg_bw_t *bw,
                                  const uint8_t samples[64],
                                  const uint8_t *qt,
                                  int16_t *prev_dc,
                                  const app_swjpg_huff_t *dc_huff,
                                  const app_swjpg_huff_t *ac_huff)
{
  float dct[64];
  int16_t qcoef[64];
  int16_t zz[64];
  int v;
  int u;
  int y;
  int x;
  int k;
  int dc_diff;
  int dc_cat;
  int run;

  for (v = 0; v < 8; v++)
  {
    for (u = 0; u < 8; u++)
    {
      float sum = 0.0f;
      float cu = (u == 0) ? 0.70710678118f : 1.0f;
      float cv = (v == 0) ? 0.70710678118f : 1.0f;
      for (y = 0; y < 8; y++)
      {
        float cy = g_swjpg_cos[y][v];
        for (x = 0; x < 8; x++)
        {
          float sx = g_swjpg_cos[x][u];
          float s = (float)samples[y * 8 + x] - 128.0f;
          sum += s * sx * cy;
        }
      }
      dct[v * 8 + u] = 0.25f * cu * cv * sum;
    }
  }

  for (k = 0; k < 64; k++)
  {
    float q = dct[k] / (float)qt[k];
    qcoef[k] = (int16_t)((q >= 0.0f) ? (q + 0.5f) : (q - 0.5f));
  }

  for (k = 0; k < 64; k++)
  {
    zz[k] = qcoef[g_swjpg_zigzag[k]];
  }

  dc_diff = (int)zz[0] - (int)(*prev_dc);
  *prev_dc = zz[0];
  dc_cat = app_swjpg_cat(dc_diff);
  if (app_swjpg_emit_huff(bw, dc_huff, (uint8_t)dc_cat) != 0)
  {
    return -1;
  }
  if (dc_cat > 0)
  {
    app_swjpg_bw_put_bits(bw, app_swjpg_amp_bits(dc_diff, dc_cat), dc_cat);
    if (bw->overflow != 0)
    {
      return -1;
    }
  }

  run = 0;
  for (k = 1; k < 64; k++)
  {
    int ac = zz[k];
    if (ac == 0)
    {
      run++;
      continue;
    }

    while (run >= 16)
    {
      if (app_swjpg_emit_huff(bw, ac_huff, 0xF0U) != 0)
      {
        return -1;
      }
      run -= 16;
    }

    {
      int ac_cat = app_swjpg_cat(ac);
      uint8_t sym = (uint8_t)((run << 4) | ac_cat);
      if (app_swjpg_emit_huff(bw, ac_huff, sym) != 0)
      {
        return -1;
      }
      app_swjpg_bw_put_bits(bw, app_swjpg_amp_bits(ac, ac_cat), ac_cat);
      if (bw->overflow != 0)
      {
        return -1;
      }
    }

    run = 0;
  }

  if (run > 0)
  {
    if (app_swjpg_emit_huff(bw, ac_huff, 0x00U) != 0)
    {
      return -1;
    }
  }

  return 0;
}

static void app_swjpg_write_dht(app_swjpg_bw_t *bw, uint8_t tc_th, const uint8_t bits[16], const uint8_t *vals)
{
  int i;
  int cnt = 0;
  for (i = 0; i < 16; i++)
  {
    cnt += bits[i];
  }

  app_swjpg_bw_put_word(bw, 0xFFC4U);
  app_swjpg_bw_put_word(bw, (uint16_t)(2 + 1 + 16 + cnt));
  app_swjpg_bw_put_byte(bw, tc_th, 0);
  for (i = 0; i < 16; i++)
  {
    app_swjpg_bw_put_byte(bw, bits[i], 0);
  }
  for (i = 0; i < cnt; i++)
  {
    app_swjpg_bw_put_byte(bw, vals[i], 0);
  }
}

static int app_swjpg_get_yuv(const uint8_t *src, int width, int x, int y, uint8_t *yy, uint8_t *uu, uint8_t *vv)
{
  const uint8_t *row;
  const uint8_t *p;

  if ((x < 0) || (y < 0) || (x >= width))
  {
    return -1;
  }

  row = src + ((size_t)y * (size_t)width * 2U);
  p = row + ((size_t)(x >> 1) * 4U);

  if (g_uvc_runtime_src_fmt == APP_UVC_TEST_SRC_FMT_UYVY)
  {
    *yy = (x & 1) ? p[3] : p[1];
    *uu = p[0];
    *vv = p[2];
  }
  else if (g_uvc_runtime_src_fmt == APP_UVC_TEST_SRC_FMT_YVYU)
  {
    *yy = (x & 1) ? p[2] : p[0];
    *uu = p[3];
    *vv = p[1];
  }
  else if (g_uvc_runtime_src_fmt == APP_UVC_TEST_SRC_FMT_VYUY)
  {
    *yy = (x & 1) ? p[3] : p[1];
    *uu = p[2];
    *vv = p[0];
  }
  else
  {
    *yy = (x & 1) ? p[2] : p[0];
    *uu = p[1];
    *vv = p[3];
  }
  return 0;
}

static int app_swjpg_encode_yuv422_to_jpeg(const uint8_t *yuyv,
                                           int width,
                                           int height,
                                           uint8_t *out,
                                           int out_max,
                                           int quality_level)
{
  app_swjpg_bw_t bw;
  int mcu_w;
  int mcu_h;
  int mcu_x;
  int mcu_y;
  int16_t dc_y = 0;
  int16_t dc_cb = 0;
  int16_t dc_cr = 0;
  uint8_t yblk0[64];
  uint8_t yblk1[64];
  uint8_t cbblk[64];
  uint8_t crblk[64];
  const uint8_t *qt_y;
  const uint8_t *qt_c;
  int x0;
  int y0;
  int yy;
  int xx;
  uint8_t sy;
  uint8_t su;
  uint8_t sv;

  if ((yuyv == NULL) || (out == NULL) || (out_max <= 0))
  {
    return -1;
  }
  if ((width <= 0) || (height <= 0) || ((width & 1) != 0))
  {
    return -1; /* YUV422 requires even width */
  }
  if (quality_level < 0)
  {
    quality_level = 0;
  }
  if (quality_level > 10)
  {
    quality_level = 10;
  }

  app_swjpg_init_tables();
  qt_y = QuantLuminance[quality_level];
  qt_c = QuantChrominance[quality_level];
  mcu_w = (width + 15) / 16;
  mcu_h = (height + 7) / 8;

  memset(&bw, 0, sizeof(bw));
  bw.buf = out;
  bw.max = out_max;

  /* SOI */
  app_swjpg_bw_put_word(&bw, 0xFFD8U);
  /* APP0 JFIF */
  app_swjpg_bw_put_word(&bw, 0xFFE0U);
  app_swjpg_bw_put_word(&bw, 16U);
  app_swjpg_bw_put_byte(&bw, 'J', 0);
  app_swjpg_bw_put_byte(&bw, 'F', 0);
  app_swjpg_bw_put_byte(&bw, 'I', 0);
  app_swjpg_bw_put_byte(&bw, 'F', 0);
  app_swjpg_bw_put_byte(&bw, 0, 0);
  app_swjpg_bw_put_byte(&bw, 1, 0);
  app_swjpg_bw_put_byte(&bw, 1, 0);
  app_swjpg_bw_put_byte(&bw, 0, 0);
  app_swjpg_bw_put_word(&bw, 1U);
  app_swjpg_bw_put_word(&bw, 1U);
  app_swjpg_bw_put_byte(&bw, 0, 0);
  app_swjpg_bw_put_byte(&bw, 0, 0);

  /* DQT (two tables, values in zigzag order) */
  app_swjpg_bw_put_word(&bw, 0xFFDBU);
  app_swjpg_bw_put_word(&bw, (uint16_t)(2 + (1 + 64) * 2));
  app_swjpg_bw_put_byte(&bw, 0x00U, 0);
  for (xx = 0; xx < 64; xx++)
  {
    app_swjpg_bw_put_byte(&bw, qt_y[g_swjpg_zigzag[xx]], 0);
  }
  app_swjpg_bw_put_byte(&bw, 0x01U, 0);
  for (xx = 0; xx < 64; xx++)
  {
    app_swjpg_bw_put_byte(&bw, qt_c[g_swjpg_zigzag[xx]], 0);
  }

  /* SOF0: 4:2:2 sampling (Y 2x1, Cb 1x1, Cr 1x1) */
  app_swjpg_bw_put_word(&bw, 0xFFC0U);
  app_swjpg_bw_put_word(&bw, 17U);
  app_swjpg_bw_put_byte(&bw, 8U, 0);
  app_swjpg_bw_put_word(&bw, (uint16_t)height);
  app_swjpg_bw_put_word(&bw, (uint16_t)width);
  app_swjpg_bw_put_byte(&bw, 3U, 0);
  app_swjpg_bw_put_byte(&bw, 1U, 0);
  app_swjpg_bw_put_byte(&bw, 0x21U, 0);
  app_swjpg_bw_put_byte(&bw, 0U, 0);
  app_swjpg_bw_put_byte(&bw, 2U, 0);
  app_swjpg_bw_put_byte(&bw, 0x11U, 0);
  app_swjpg_bw_put_byte(&bw, 1U, 0);
  app_swjpg_bw_put_byte(&bw, 3U, 0);
  app_swjpg_bw_put_byte(&bw, 0x11U, 0);
  app_swjpg_bw_put_byte(&bw, 1U, 0);

  /* DHT */
  app_swjpg_write_dht(&bw, 0x00U, g_swjpg_bits_dc[0], g_swjpg_vals_dc[0]); /* DC Y */
  app_swjpg_write_dht(&bw, 0x10U, g_swjpg_bits_ac[0], g_swjpg_vals_ac[0]); /* AC Y */
  app_swjpg_write_dht(&bw, 0x01U, g_swjpg_bits_dc[1], g_swjpg_vals_dc[1]); /* DC C */
  app_swjpg_write_dht(&bw, 0x11U, g_swjpg_bits_ac[1], g_swjpg_vals_ac[1]); /* AC C */

  /* SOS */
  app_swjpg_bw_put_word(&bw, 0xFFDAU);
  app_swjpg_bw_put_word(&bw, 12U);
  app_swjpg_bw_put_byte(&bw, 3U, 0);
  app_swjpg_bw_put_byte(&bw, 1U, 0);
  app_swjpg_bw_put_byte(&bw, 0x00U, 0);
  app_swjpg_bw_put_byte(&bw, 2U, 0);
  app_swjpg_bw_put_byte(&bw, 0x11U, 0);
  app_swjpg_bw_put_byte(&bw, 3U, 0);
  app_swjpg_bw_put_byte(&bw, 0x11U, 0);
  app_swjpg_bw_put_byte(&bw, 0U, 0);
  app_swjpg_bw_put_byte(&bw, 63U, 0);
  app_swjpg_bw_put_byte(&bw, 0U, 0);

  for (mcu_y = 0; mcu_y < mcu_h; mcu_y++)
  {
    for (mcu_x = 0; mcu_x < mcu_w; mcu_x++)
    {
      x0 = mcu_x * 16;
      y0 = mcu_y * 8;

      for (yy = 0; yy < 8; yy++)
      {
        for (xx = 0; xx < 8; xx++)
        {
          int sx = x0 + xx;
          int syy = y0 + yy;
          if (sx >= width)
          {
            sx = width - 1;
          }
          if (syy >= height)
          {
            syy = height - 1;
          }
          (void)app_swjpg_get_yuv(yuyv, width, sx, syy, &sy, &su, &sv);
          yblk0[yy * 8 + xx] = sy;

          sx = x0 + 8 + xx;
          if (sx >= width)
          {
            sx = width - 1;
          }
          (void)app_swjpg_get_yuv(yuyv, width, sx, syy, &sy, &su, &sv);
          yblk1[yy * 8 + xx] = sy;

          sx = x0 + (xx * 2);
          if (sx >= width)
          {
            sx = width - 1;
          }
          (void)app_swjpg_get_yuv(yuyv, width, sx, syy, &sy, &su, &sv);
          cbblk[yy * 8 + xx] = su;
          crblk[yy * 8 + xx] = sv;
        }
      }

      if (app_swjpg_encode_block(&bw, yblk0, qt_y, &dc_y, &g_swjpg_dc_huff[0], &g_swjpg_ac_huff[0]) != 0) return -1;
      if (app_swjpg_encode_block(&bw, yblk1, qt_y, &dc_y, &g_swjpg_dc_huff[0], &g_swjpg_ac_huff[0]) != 0) return -1;
      if (app_swjpg_encode_block(&bw, cbblk, qt_c, &dc_cb, &g_swjpg_dc_huff[1], &g_swjpg_ac_huff[1]) != 0) return -1;
      if (app_swjpg_encode_block(&bw, crblk, qt_c, &dc_cr, &g_swjpg_dc_huff[1], &g_swjpg_ac_huff[1]) != 0) return -1;
    }
  }

  app_swjpg_bw_flush_bits(&bw);
  app_swjpg_bw_put_word(&bw, 0xFFD9U);

  if (bw.overflow != 0)
  {
    return -1;
  }
  return bw.pos;
}

static int app_custom_test_encoder_yuyv_to_jpeg(const uint8_t *yuv, int yuv_len, uint8_t *jpeg_out, int jpeg_out_max)
{
  uint32_t ts_ms;
  int expected_len;
  int jpeg_len;

  if ((yuv == NULL) || (jpeg_out == NULL) || (jpeg_out_max <= 0))
  {
    return -1;
  }

  expected_len = APP_UVC_ENC_WIDTH * APP_UVC_ENC_HEIGHT * CAPTURE_BPP;
  if (yuv_len != expected_len)
  {
    return -1;
  }

  /* Match main path behavior: don't encode while previous UVC frame still flying. */
  if (buffer_flying != 0)
  {
    g_uvc_drop_busy_count++;
    g_uvc_last_enc_ret = -2;
    g_uvc_last_enc_ms = 0U;
    return -1;
  }

  ts_ms = HAL_GetTick();
  jpeg_len = app_swjpg_encode_yuv422_to_jpeg(yuv,
                                             APP_UVC_ENC_WIDTH,
                                             APP_UVC_ENC_HEIGHT,
                                             jpeg_out,
                                             jpeg_out_max,
                                             APP_SWJPEG_QUALITY_LEVEL);
  g_uvc_last_enc_ms = HAL_GetTick() - ts_ms;
  g_uvc_last_enc_ret = jpeg_len;

  if (jpeg_len <= 0)
  {
    g_uvc_enc_fail_count++;
    return -1;
  }

  g_uvc_enc_ok_count++;
  app_clean_dcache_for_hw_read(jpeg_out, jpeg_len);
  return jpeg_len;
}
#endif

static void app_enc_invalidate_dcache_after_encode(uint8_t *buf, int len)
{
  app_invalidate_dcache_for_cpu_read(buf, len);
}

static void app_flush_cam_irq_logs(void)
{
  if (g_pipe1_first_vsync_pending_log != 0)
  {
    g_pipe1_first_vsync_pending_log = 0;
    printf("[CAM] pipe1 first VSYNC IRQ\r\n");
  }

  if (g_pipe1_first_frame_pending_log != 0)
  {
    g_pipe1_first_frame_pending_log = 0;
    printf("[CAM] pipe1 first frame IRQ\r\n");
  }

  if (g_pipe2_first_frame_pending_log != 0)
  {
    g_pipe2_first_frame_pending_log = 0;
    printf("[CAM] pipe2 first frame IRQ\r\n");
  }

  if (g_pipe1_first_rearm_ok_pending_log != 0)
  {
    g_pipe1_first_rearm_ok_pending_log = 0;
    printf("[CAM] pipe1 rearm OK next_capt_idx=%d addr=0x%08lX\r\n",
           g_pipe1_first_rearm_ok_next_idx,
           (unsigned long)g_pipe1_first_rearm_ok_addr);
  }

  if (g_pipe1_rearm_fail_pending_log != 0)
  {
    g_pipe1_rearm_fail_pending_log = 0;
    printf("[CAM][ERR] pipe1 rearm failed ret=%d state=%lu disp_idx=%d capt_idx=%d next=%d\r\n",
           g_pipe1_rearm_fail_last_ret,
           (unsigned long)g_pipe1_rearm_fail_last_state,
           g_pipe1_rearm_fail_last_disp_idx,
           g_pipe1_rearm_fail_last_capt_idx,
           g_pipe1_rearm_fail_last_next_idx);
  }

#if APP_DCMIPP_ERR_LOG_ENABLE
  if (g_pipe_error_pending_log != 0)
  {
    uint32_t now_ms = HAL_GetTick();
    if ((g_pipe_error_last_log_ms == 0U) || ((now_ms - g_pipe_error_last_log_ms) >= APP_DCMIPP_ERR_LOG_PERIOD_MS))
    {
      uint32_t p1_ovr_flag = ((g_pipe_error_last_cmsr2 & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U;
      uint32_t p1_ovr_irq_en = ((g_pipe_error_last_cmier & DCMIPP_IT_PIPE1_OVR) != 0U) ? 1U : 0U;
      uint32_t p1_ovr_err = ((g_pipe_error_last_code & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U) ? 1U : 0U;
      g_pipe_error_pending_log = 0;
      g_pipe_error_last_log_ms = now_ms;
      printf("[CAM][ERR] DCMIPP pipe=%lu state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX p1fctcr=0x%08lX ovrf=%lu ovrie=%lu ovrerr=%lu recov=%lu srec=%lu seq=%lu enc_ms=%lu send_ms=%lu len=%d fly=%d tx=%lu/%lu\r\n",
             (unsigned long)g_pipe_error_last_pipe,
             (unsigned long)g_pipe_error_last_state,
             (unsigned long)g_pipe_error_last_code,
             (unsigned long)g_pipe_error_last_cmsr2,
             (unsigned long)g_pipe_error_last_cmier,
             (unsigned long)g_pipe_error_last_p1fctcr,
             (unsigned long)p1_ovr_flag,
             (unsigned long)p1_ovr_irq_en,
             (unsigned long)p1_ovr_err,
             (unsigned long)g_pipe_error_soft_recover_count,
             (unsigned long)g_pipe_error_start_recover_count,
             (unsigned long)g_uvc_last_pipe1_seq,
             (unsigned long)g_uvc_last_enc_ms,
             (unsigned long)g_uvc_last_send_ms,
             g_uvc_last_show_len,
             buffer_flying,
             (unsigned long)g_uvc_tx_submit_id,
             (unsigned long)g_uvc_tx_release_id);
    }
  }
#endif
}

void APP_CAM_DebugOnPipeError(uint32_t pipe)
{
  DCMIPP_HandleTypeDef *hdcmipp = CMW_CAMERA_GetDCMIPPHandle();

  if (pipe < 3U)
  {
    g_pipe_error_irq_count[pipe]++;
  }

  if ((hdcmipp != NULL) && (hdcmipp->Instance != NULL))
  {
    g_pipe_error_last_pipe = pipe;
    g_pipe_error_last_state = HAL_DCMIPP_PIPE_GetState(hdcmipp, pipe);
    g_pipe_error_last_code = hdcmipp->ErrorCode;
    g_pipe_error_last_cmsr2 = READ_REG(hdcmipp->Instance->CMSR2);
    g_pipe_error_last_cmier = READ_REG(hdcmipp->Instance->CMIER);
    g_pipe_error_last_p1fctcr = READ_REG(hdcmipp->Instance->P1FCTCR);

#if APP_DCMIPP_ERR_START_SOFT_RECOVER_ENABLE
    if ((pipe == DCMIPP_PIPE1) && ((hdcmipp->ErrorCode & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U))
    {
      uint32_t now_ms = HAL_GetTick();
      uint32_t since_start_ms = now_ms - g_uvc_stream_start_tick_ms;
      if ((uvc_is_active != 0) &&
          (since_start_ms <= APP_DCMIPP_ERR_START_SOFT_RECOVER_WINDOW_MS) &&
          (g_pipe_error_start_recover_count < APP_DCMIPP_ERR_START_SOFT_RECOVER_MAX))
      {
        __HAL_DCMIPP_CLEAR_FLAG(hdcmipp, DCMIPP_FLAG_PIPE1_OVR);
        __HAL_DCMIPP_ENABLE_IT(hdcmipp, DCMIPP_IT_PIPE1_OVR);
        hdcmipp->ErrorCode &= ~HAL_DCMIPP_ERROR_PIPE1_OVR;
        if (hdcmipp->PipeState[1] == HAL_DCMIPP_PIPE_STATE_ERROR)
        {
          hdcmipp->PipeState[1] = HAL_DCMIPP_PIPE_STATE_BUSY;
        }
        g_pipe_error_soft_recover_count++;
        g_pipe_error_start_recover_count++;
      }
    }
#endif

#if APP_DCMIPP_ERR_SOFT_RECOVER
    if ((pipe == DCMIPP_PIPE1) && ((hdcmipp->ErrorCode & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U))
    {
      /* HAL disables PIPE1 OVR IRQ after first hit; re-enable so we can monitor recurring errors. */
      __HAL_DCMIPP_ENABLE_IT(hdcmipp, DCMIPP_IT_PIPE1_OVR);
      hdcmipp->ErrorCode &= ~HAL_DCMIPP_ERROR_PIPE1_OVR;
      if (hdcmipp->PipeState[1] == HAL_DCMIPP_PIPE_STATE_ERROR)
      {
        hdcmipp->PipeState[1] = HAL_DCMIPP_PIPE_STATE_BUSY;
      }
      g_pipe_error_soft_recover_count++;
    }
#endif
  }

  g_pipe_error_pending_log = 1;
}

/* dma2d */
static SemaphoreHandle_t dma2d_lock;
static StaticSemaphore_t dma2d_lock_buffer;
static SemaphoreHandle_t dma2d_sem;
static StaticSemaphore_t dma2d_sem_buffer;
/* Store current DMA2D_HandleTypeDef instance so we can propagate to irq handler */
static DMA2D_HandleTypeDef *dma2d_current;

/* capture buffers */
static uint8_t capture_buffer[CAPTURE_BUFFER_NB][VENC_MAX_WIDTH * VENC_MAX_HEIGHT * CAPTURE_BPP] ALIGN_32 IN_PSRAM;
static int capture_buffer_disp_idx = 1;
static int capture_buffer_capt_idx = 0;
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC && !APP_UVC_TEST_STATIC_JPEG_MODE
#if APP_UVC_TEST_SRC_IN_AXI && APP_UVC_TEST_SMALL_FRAME_ENABLE && (APP_UVC_TEST_SMALL_WIDTH <= 640) && (APP_UVC_TEST_SMALL_HEIGHT <= 480)
/* Cache-coherency debug path: make synthetic YUV source uncached. */
static uint8_t g_uvc_test_src_buffer[APP_UVC_ENC_FRAME_BYTES] ALIGN_32 UNCACHED;
#elif APP_UVC_TEST_SRC_IN_AXI
static uint8_t g_uvc_test_src_buffer[APP_UVC_ENC_FRAME_BYTES] ALIGN_32;
#else
#if APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE
/* Dedicated test source in PSRAM MMP window (not capture_buffer[]). */
static uint8_t *const g_uvc_test_src_buffer = (uint8_t *)APP_UVC_TEST_FIXED_PSRAM_SRC_ADDR;
#else
static uint8_t g_uvc_test_src_buffer[APP_UVC_ENC_FRAME_BYTES] ALIGN_32 IN_PSRAM;
#endif
#endif

#endif

static uint8_t *app_get_test_src_buffer(void)
{
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC && !APP_UVC_TEST_STATIC_JPEG_MODE
  return g_uvc_test_src_buffer;
#else
  return NULL;
#endif
}

static uint8_t *app_psram_mmp_alias(uint8_t *buf)
{
  /* Keep PSRAM pointer unchanged: 0x9100_0000 is an offset window, not a guaranteed mirror alias. */
  return buf;
}

static uint8_t *app_get_test_src_cpu_buffer(void)
{
  uint8_t *buf = app_get_test_src_buffer();
#if APP_PSRAM_TEST_SRC_USE_MMP_ALIAS && APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_FORCE_DEDICATED_SRC && !APP_UVC_TEST_STATIC_JPEG_MODE && !APP_UVC_TEST_SRC_IN_AXI
  return app_psram_mmp_alias(buf);
#else
  return buf;
#endif
}

/* model */
static uint8_t network_ctx[STAI_NETWORK_CONTEXT_SIZE] ALIGN_32;
 /* nn input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;
 /* nn output buffers */
static uint8_t nn_output_buffers[2][NN_BUFFER_OUT_SIZE_ALIGN] ALIGN_32;
static bqueue_t nn_output_queue;

/* venc */
/* In small-frame test mode, place JPEG output in UNCACHED region to avoid HW/CPU cache mismatch. */
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_SMALL_FRAME_ENABLE
static uint8_t venc_out_buffer[VENC_OUT_BUFFER_SIZE] ALIGN_32 UNCACHED;
#else
static uint8_t venc_out_buffer[VENC_OUT_BUFFER_SIZE] ALIGN_32;
#endif
static uint8_t uvc_in_buffers[VENC_OUT_BUFFER_SIZE] ALIGN_32;

/* uvc */
static struct uvcl_callbacks uvcl_cbs;
static int uvc_is_active;
static volatile int buffer_flying;
static int force_intra;
#if APP_UVC_TEST_PATTERN_MODE
static uint8_t g_uvc_test_pattern_ready;
static uint32_t g_uvc_test_pattern_seq;
#if !APP_UVC_TEST_STATIC_JPEG_MODE
static uint8_t g_uvc_test_encoded_ready;
static int g_uvc_test_encoded_len;
#endif
#endif

 /* threads */
#if !APP_DVP_BRINGUP_PIPE1_ONLY
static StaticTask_t nn_thread;
static StackType_t nn_thread_stack[2 * configMINIMAL_STACK_SIZE];
static StaticTask_t dp_thread;
static StackType_t dp_thread_stack[2 *configMINIMAL_STACK_SIZE];
#endif
static StaticTask_t isp_thread;
static StackType_t isp_thread_stack[2 *configMINIMAL_STACK_SIZE];
static SemaphoreHandle_t isp_sem;
static StaticSemaphore_t isp_sem_buffer;

static int is_cache_enable(void)
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

static void cpuload_init(cpuload_info_t *cpu_load)
{
  memset(cpu_load, 0, sizeof(cpuload_info_t));
}

static void cpuload_update(cpuload_info_t *cpu_load)
{
  int i;

  cpu_load->history[1] = cpu_load->history[0];
  cpu_load->history[0].total = portGET_RUN_TIME_COUNTER_VALUE();
  cpu_load->history[0].thread = cpu_load->history[0].total - ulTaskGetIdleRunTimeCounter();
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
    return ;

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
}

static void cpuload_get_info(cpuload_info_t *cpu_load, float *cpu_load_last, float *cpu_load_last_second,
                             float *cpu_load_last_five_seconds)
{
  if (cpu_load_last)
    *cpu_load_last = 100.0 * (cpu_load->history[0].thread - cpu_load->history[1].thread) /
                     (cpu_load->history[0].total - cpu_load->history[1].total);
  if (cpu_load_last_second)
    *cpu_load_last_second = 100.0 * (cpu_load->history[2].thread - cpu_load->history[3].thread) /
                     (cpu_load->history[2].total - cpu_load->history[3].total);
  if (cpu_load_last_five_seconds)
    *cpu_load_last_five_seconds = 100.0 * (cpu_load->history[2].thread - cpu_load->history[7].thread) /
                     (cpu_load->history[2].total - cpu_load->history[7].total);
}

static void time_stat_update(time_stat_t *p_stat, int value)
{
  int ret;

  ret = xSemaphoreTake(stat_info_lock, portMAX_DELAY);
  assert(ret == pdTRUE);

  p_stat->last = value;
  p_stat->acc += value;
  p_stat->total++;
  p_stat->mean = (float)p_stat->acc / p_stat->total;

  ret = xSemaphoreGive(stat_info_lock);
  assert(ret == pdTRUE);
}

static void stat_info_copy(stat_info_t *copy)
{
  int ret;

  ret = xSemaphoreTake(stat_info_lock, portMAX_DELAY);
  assert(ret == pdTRUE);

  *copy = stat_info;

  ret = xSemaphoreGive(stat_info_lock);
  assert(ret == pdTRUE);
}

static int bqueue_init(bqueue_t *bq, int buffer_nb, uint8_t **buffers)
{
  int i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
    return -1;

  bq->free = xSemaphoreCreateCountingStatic(buffer_nb, buffer_nb, &bq->free_buffer);
  if (!bq->free)
    goto free_sem_error;
  bq->ready = xSemaphoreCreateCountingStatic(buffer_nb, 0, &bq->ready_buffer);
  if (!bq->ready)
    goto ready_sem_error;

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++) {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  vSemaphoreDelete(bq->free);
free_sem_error:
  return -1;
}

static uint8_t *bqueue_get_free(bqueue_t *bq, int is_blocking)
{
  uint8_t *res;
  int ret;

  ret = xSemaphoreTake(bq->free, is_blocking ? portMAX_DELAY : 0);
  if (ret == pdFALSE)
    return NULL;

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_free(bqueue_t *bq)
{
  int ret;

  ret = xSemaphoreGive(bq->free);
  assert(ret == pdTRUE);
}

static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int ret;

  ret = xSemaphoreTake(bq->ready, portMAX_DELAY);
  assert(ret == pdTRUE);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_ready(bqueue_t *bq)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  if (xPortIsInsideInterrupt()) {
    ret = xSemaphoreGiveFromISR(bq->ready, &xHigherPriorityTaskWoken);
    assert(ret == pdTRUE);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    ret = xSemaphoreGive(bq->ready);
    assert(ret == pdTRUE);
  }
}

static int app_main_pipe_frame_event(void)
{
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_USE_DCMIPP_SOURCE
  uint8_t *dedicated_src = app_get_test_src_cpu_buffer();
  int ret;

  if (dedicated_src == NULL)
  {
    return HAL_ERROR;
  }

  /* Direct path: DCMIPP PIPE1 always writes to dedicated source buffer. */
  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t)dedicated_src);
  if (ret != HAL_OK)
  {
    g_pipe1_rearm_fail_count++;
    g_pipe1_rearm_fail_last_ret = ret;
    g_pipe1_rearm_fail_last_state = HAL_DCMIPP_PIPE_GetState(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1);
    g_pipe1_rearm_fail_last_disp_idx = 0;
    g_pipe1_rearm_fail_last_capt_idx = 0;
    g_pipe1_rearm_fail_last_next_idx = 0;
    g_pipe1_rearm_fail_pending_log = 1;
    return ret;
  }

  g_pipe1_rearm_ok_count++;
  g_pipe1_last_completed_idx = 0;
  g_pipe1_completed_seq++;
  return HAL_OK;
#else
  int completed_idx = capture_buffer_capt_idx;
  int next_disp_idx = (capture_buffer_disp_idx + 1) % CAPTURE_BUFFER_NB;
  int next_capt_idx = (capture_buffer_capt_idx + 1) % CAPTURE_BUFFER_NB;
  int ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t) capture_buffer[next_capt_idx]);
  if (ret != HAL_OK)
  {
    g_pipe1_rearm_fail_count++;
    g_pipe1_rearm_fail_last_ret = ret;
    g_pipe1_rearm_fail_last_state = HAL_DCMIPP_PIPE_GetState(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1);
    g_pipe1_rearm_fail_last_disp_idx = capture_buffer_disp_idx;
    g_pipe1_rearm_fail_last_capt_idx = capture_buffer_capt_idx;
    g_pipe1_rearm_fail_last_next_idx = next_capt_idx;
    g_pipe1_rearm_fail_pending_log = 1;
    return ret;
  }

  capture_buffer_disp_idx = next_disp_idx;
  capture_buffer_capt_idx = next_capt_idx;
  g_pipe1_rearm_ok_count++;
  g_pipe1_last_completed_idx = completed_idx;
  g_pipe1_completed_seq++;
#if APP_PIPE1_FRAME_DUMP_ENABLE
  g_pipe1_dump_idx = completed_idx;
  g_pipe1_dump_pending = 1;
#endif

  if (g_pipe1_rearm_ok_count == 1U)
  {
    g_pipe1_first_rearm_ok_next_idx = next_capt_idx;
    g_pipe1_first_rearm_ok_addr = (uint32_t)capture_buffer[next_capt_idx];
    g_pipe1_first_rearm_ok_pending_log = 1;
  }

  return HAL_OK;
#endif
}

static void app_ancillary_pipe_frame_event()
{
  uint8_t *next_buffer;
  int ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer) {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer);
    assert(ret == HAL_OK);
    bqueue_put_ready(&nn_input_queue);
  }
}

static void app_main_pipe_vsync_event()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  ret = xSemaphoreGiveFromISR(isp_sem, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if !APP_DVP_BRINGUP_PIPE1_ONLY
static void nn_thread_fct(void *arg)
{
  stai_network_info info;
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  uint32_t total_ts;
  uint32_t ts;
  int ret;

  (void) nn_period_ms;

  /* initialize runtime */
  ret = stai_runtime_init();
  assert(ret == STAI_SUCCESS);
  /* init model instance */
  ret = stai_network_init(network_ctx);
  assert(ret == STAI_SUCCESS);

  /* setup inout buffers and size */
  ret = stai_network_get_info(network_ctx, &info);
  assert(ret == STAI_SUCCESS);
  assert(info.n_inputs == 1);
  assert(info.n_outputs == 1);
  //printf("nn_out_len = %d\n", nn_out_len);
  assert(info.outputs[0].size_bytes == NN_BUFFER_OUT_SIZE);

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();

  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
  while (1)
  {
    uint8_t *capture_buffer;
    uint8_t *output_buffer;
    stai_ptr outputs[1];
    stai_ptr inputs[1];

    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];

    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);
    output_buffer = bqueue_get_free(&nn_output_queue, 1);
    assert(output_buffer);

    total_ts = HAL_GetTick();
    /* run ATON inference */
    ts = HAL_GetTick();
     /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
    inputs[0] = capture_buffer;
    ret = stai_network_set_inputs(network_ctx, inputs, ARRAY_NB(inputs));
    assert(ret == STAI_SUCCESS);
     /* Invalidate output buffer before Hw access it */
    CACHE_OP(SCB_InvalidateDCache_by_Addr(output_buffer, sizeof(nn_output_buffers[0])));
    outputs[0] = output_buffer;
    ret = stai_network_set_outputs(network_ctx, outputs, ARRAY_NB(outputs));
    assert(ret == STAI_SUCCESS);
    Run_Inference(network_ctx);
    time_stat_update(&stat_info.nn_inference_time, HAL_GetTick() - ts);

    /* release buffers */
    bqueue_put_free(&nn_input_queue);
    bqueue_put_ready(&nn_output_queue);

    time_stat_update(&stat_info.nn_total_time, HAL_GetTick() - total_ts);
  }
}
#endif

static size_t encode_display(int is_intra_force, uint8_t *p_buffer)
{
  size_t res;
  int enc_ret;
  uint32_t ts_ms;
  uint32_t sum_src;
  uint32_t sum_dst;
#if APP_ENC_OUT_SENTINEL_DEBUG
  uint8_t out_head_before[4] = {0xA5U, 0xA5U, 0xA5U, 0xA5U};
  int head_unchanged;
#endif
#if APP_ENC_RUNTIME_VALIDATE
  app_jpeg_meta_t src_meta;
  app_jpeg_meta_t dst_meta;
  int src_valid;
  int dst_valid;
  uint32_t now_ms;
#endif
#if APP_UVC_TEST_PATTERN_MODE
  uint8_t pre[8] = {0};
  uint8_t post[8] = {0};
  int n;
  int i;
#endif
  const uint8_t *audit_jpeg_bs;

  /* USB has not released previous frame yet: skip encode to avoid extra PSRAM/AXI pressure. */
  if (buffer_flying)
  {
    g_uvc_drop_busy_count++;
    g_uvc_last_enc_ret = -2;
    g_uvc_last_enc_ms = 0U;
    force_intra = 1;
    return (size_t)-1;
  }

#if APP_ENC_OUT_SENTINEL_DEBUG
  /* Mark output head before encode to verify HW really writes this buffer. */
  venc_out_buffer[0] = 0xA5U;
  venc_out_buffer[1] = 0xA5U;
  venc_out_buffer[2] = 0xA5U;
  venc_out_buffer[3] = 0xA5U;
  app_clean_dcache_for_hw_read(venc_out_buffer, 32);
#endif

  ts_ms = HAL_GetTick();
  res = ENC_EncodeFrame(p_buffer, venc_out_buffer, VENC_OUT_BUFFER_SIZE, is_intra_force);
  g_uvc_last_enc_ms = HAL_GetTick() - ts_ms;
  enc_ret = (int)res;

  /* encoder failed certainly due to output buffer too small */
  if (enc_ret <= 0)
  {
    g_uvc_enc_fail_count++;
    g_uvc_last_enc_ret = enc_ret;
    return res;
  }
#if APP_UVC_TEST_PATTERN_MODE
  n = (enc_ret > 8) ? 8 : enc_ret;
  for (i = 0; i < n; i++)
  {
    pre[i] = venc_out_buffer[i];
  }
#endif
  app_enc_invalidate_dcache_after_encode(venc_out_buffer, enc_ret);
#if APP_ENC_OUT_SENTINEL_DEBUG
  head_unchanged = (venc_out_buffer[0] == out_head_before[0]) &&
                   (venc_out_buffer[1] == out_head_before[1]) &&
                   (venc_out_buffer[2] == out_head_before[2]) &&
                   (venc_out_buffer[3] == out_head_before[3]);
  if (head_unchanged)
  {
    g_enc_out_sentinel_bad_count++;
    printf("[ENC][OUT] head unchanged after encode ptr=0x%08lX len=%d cnt=%lu b0..b3=%02X %02X %02X %02X\r\n",
           (unsigned long)venc_out_buffer,
           enc_ret,
           (unsigned long)g_enc_out_sentinel_bad_count,
           venc_out_buffer[0], venc_out_buffer[1], venc_out_buffer[2], venc_out_buffer[3]);
  }
#endif
#if APP_UVC_TEST_PATTERN_MODE
  for (i = 0; i < n; i++)
  {
    post[i] = venc_out_buffer[i];
  }
  if ((pre[0] != post[0]) || (pre[1] != post[1]) || (pre[2] != post[2]) || (pre[3] != post[3]))
  {
    printf("[ENC][CACHE] pre=%02X %02X %02X %02X %02X %02X %02X %02X post=%02X %02X %02X %02X %02X %02X %02X %02X len=%d\r\n",
           pre[0], pre[1], pre[2], pre[3], pre[4], pre[5], pre[6], pre[7],
           post[0], post[1], post[2], post[3], post[4], post[5], post[6], post[7],
           enc_ret);
  }
#endif
  app_log_encoded_data_1s(venc_out_buffer, enc_ret);
  sum_src = app_sum_prefix_u32(venc_out_buffer, enc_ret, APP_ENC_DUMP_SUM_BYTES);

#if APP_ENC_RUNTIME_VALIDATE
  app_jpeg_parse_meta(venc_out_buffer, enc_ret, &src_meta);
  src_valid = app_jpeg_is_valid(&src_meta);
  if (!src_valid ||
      (src_meta.sof_w != APP_UVC_ENC_WIDTH) ||
      (src_meta.sof_h != APP_UVC_ENC_HEIGHT))
  {
    g_enc_runtime_invalid_count++;
#if !APP_ENC_FORCE_SEND_AFTER_ENCODE
    g_uvc_enc_fail_count++;
    g_uvc_last_enc_ret = -3;
#endif
    now_ms = HAL_GetTick();
    if ((now_ms - g_enc_runtime_last_log_ms) >= APP_ENC_VALIDATE_LOG_PERIOD_MS)
    {
      printf("[ENC][CHK] src-invalid size=%d soi=%d sof=%d wh=%dx%d exp=%dx%d sos=%d eoi=%d bad=%lu force_send=%d\r\n",
             enc_ret,
             src_meta.has_soi,
             src_meta.sof_off,
             src_meta.sof_w,
             src_meta.sof_h,
             APP_UVC_ENC_WIDTH,
             APP_UVC_ENC_HEIGHT,
             src_meta.sos_off,
             src_meta.has_eoi,
             (unsigned long)g_enc_runtime_invalid_count,
             APP_ENC_FORCE_SEND_AFTER_ENCODE);
      g_enc_runtime_last_log_ms = now_ms;
    }
#if !APP_ENC_FORCE_SEND_AFTER_ENCODE
    return (size_t)-1;
#endif
  }
#endif

  memcpy(uvc_in_buffers, venc_out_buffer, res);
  sum_dst = app_sum_prefix_u32(uvc_in_buffers, enc_ret, APP_ENC_DUMP_SUM_BYTES);
  audit_jpeg_bs = uvc_in_buffers;

#if APP_ENC_RUNTIME_VALIDATE
  app_jpeg_parse_meta(uvc_in_buffers, enc_ret, &dst_meta);
  dst_valid = app_jpeg_is_valid(&dst_meta);
  if (!dst_valid ||
      (sum_src != sum_dst) ||
      (src_meta.sof_w != dst_meta.sof_w) ||
      (src_meta.sof_h != dst_meta.sof_h))
  {
    g_enc_runtime_copy_mismatch_count++;
#if !APP_ENC_FORCE_SEND_AFTER_ENCODE
    g_uvc_enc_fail_count++;
    g_uvc_last_enc_ret = -4;
#endif
    now_ms = HAL_GetTick();
    if ((now_ms - g_enc_runtime_last_log_ms) >= APP_ENC_VALIDATE_LOG_PERIOD_MS)
    {
      printf("[ENC][CHK] copy-mismatch size=%d sum=%lu/%lu src_wh=%dx%d dst_wh=%dx%d dst_ok=%d mis=%lu force_send=%d\r\n",
             enc_ret,
             (unsigned long)sum_src,
             (unsigned long)sum_dst,
             src_meta.sof_w, src_meta.sof_h,
             dst_meta.sof_w, dst_meta.sof_h,
             dst_valid,
             (unsigned long)g_enc_runtime_copy_mismatch_count,
             APP_ENC_FORCE_SEND_AFTER_ENCODE);
      g_enc_runtime_last_log_ms = now_ms;
    }
#if !APP_ENC_FORCE_SEND_AFTER_ENCODE
    return (size_t)-1;
#endif
  }
  g_enc_runtime_valid_count++;
#endif

  g_uvc_enc_ok_count++;
  g_uvc_last_enc_ret = enc_ret;
  app_uvc_log_color_audit_1s(p_buffer, APP_UVC_ENC_FRAME_BYTES, audit_jpeg_bs, enc_ret);
  app_uvc_verify_jpeg_buffers_1s(p_buffer, APP_UVC_ENC_FRAME_BYTES, venc_out_buffer, uvc_in_buffers, enc_ret);

  return res;
}

static int send_display(uint8_t *p_frame, int len)
{
  int ret;
  int presend_ret;
  uint32_t sig;
  uint32_t ts_ms;
#if APP_UVC_TX_DEBUG_ENABLE
  uint32_t now_ms;
#endif

  g_uvc_show_req_count++;
  g_uvc_tx_submit_id++;
  g_uvc_tx_last_submit_len = (uint32_t)len;
  sig = app_sum_prefix_u32(p_frame, len, APP_UVC_TX_SIG_BYTES);
  g_uvc_tx_last_submit_sig = sig;
  presend_ret = app_uvc_presend_check_jpeg(p_frame, len);
#if APP_UVC_PRESEND_BLOCK_ON_INVALID
  if (presend_ret != 0)
  {
    g_uvc_show_fail_count++;
    g_uvc_last_show_ret = -2;
    g_uvc_last_show_len = len;
    return -1;
  }
#else
  (void)presend_ret;
#endif
  app_uvc_clean_dcache_for_show(p_frame, len);
#if APP_UVC_TX_DEBUG_ENABLE
  now_ms = HAL_GetTick();
  if ((now_ms - g_uvc_tx_last_log_ms) >= APP_UVC_TX_DEBUG_PERIOD_MS)
  {
    printf("[UVC][TX] submit id=%lu len=%d ptr=0x%08lX sig%u=%lu clean=%d\r\n",
           (unsigned long)g_uvc_tx_submit_id,
           len,
           (unsigned long)p_frame,
           APP_UVC_TX_SIG_BYTES,
           (unsigned long)g_uvc_tx_last_submit_sig,
           APP_UVC_CLEAN_DCACHE_BEFORE_SHOW);
    g_uvc_tx_last_log_ms = now_ms;
  }
#endif
  buffer_flying = 1;
  ts_ms = HAL_GetTick();
  ret = UVCL_ShowFrame(p_frame, len);
  g_uvc_last_send_ms = HAL_GetTick() - ts_ms;
  if (ret != 0)
  {
    buffer_flying = 0;
    g_uvc_show_fail_count++;
  }
  else
  {
    g_uvc_show_ok_count++;
  }
  g_uvc_last_show_ret = ret;
  g_uvc_last_show_len = len;

  return ret;
}

static void app_uvc_try_send_pipe1_bringup(void)
{
#if APP_DVP_BRINGUP_PIPE1_ONLY
#if !APP_UVC_TEST_PATTERN_MODE
  static uint32_t last_seq;
#endif
  int use_static_jpeg = 0;
  int idx;
  int len;
  int show_ret = 0;
  uint32_t seq;
  uint8_t *src_buf;

  if (!uvc_is_active)
  {
    return;
  }

#if APP_UVC_TEST_PATTERN_MODE
#if APP_UVC_TEST_STATIC_JPEG_MODE
  seq = ++g_uvc_test_pattern_seq;
  idx = 0;
  src_buf = NULL;
  use_static_jpeg = 1;
#else
#if (CAPTURE_BPP != 2)
  return;
#else
#if APP_UVC_TEST_USE_DCMIPP_SOURCE
  {
    static uint32_t last_pipe1_seq_in_test = 0U;
    uint8_t *dedicated_src;

    seq = g_pipe1_completed_seq;
    if ((seq == 0U) || (seq == last_pipe1_seq_in_test))
    {
      return;
    }

    idx = g_pipe1_last_completed_idx;
    if ((idx < 0) || (idx >= CAPTURE_BUFFER_NB))
    {
      return;
    }

#if APP_UVC_TEST_FORCE_DEDICATED_SRC
    dedicated_src = app_get_test_src_cpu_buffer();
    if (dedicated_src == NULL)
    {
      return;
    }
    /* Direct path: DCMIPP already wrote frame into g_uvc_test_src_buffer. */
    app_invalidate_dcache_for_cpu_read(dedicated_src, APP_UVC_ENC_FRAME_BYTES);
    src_buf = dedicated_src;
#else
    src_buf = capture_buffer[idx];
#endif

    last_pipe1_seq_in_test = seq;
    g_uvc_last_pipe1_seq = seq;
  }
#else
#if !APP_UVC_TEST_FORCE_DEDICATED_SRC
  /* Legacy path only: refresh capture_buffer[0] from header. */
  if (app_testpat_reload_capture0_from_header() != 0)
  {
    return;
  }
#endif
#if APP_UVC_TEST_FORCE_DEDICATED_SRC && APP_UVC_TEST_PREBUILT_YUYV_MODE
  /* Dedicated source is loaded when pattern becomes ready; avoid reloading every frame. */
#endif

  if (g_uvc_test_pattern_ready == 0U)
  {
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
    uint8_t *dedicated_src = app_get_test_src_cpu_buffer();
    if (dedicated_src == NULL)
    {
      return;
    }
#if APP_UVC_TEST_PREBUILT_YUYV_MODE
    if (app_testpat_reload_dedicated_src_from_header() != 0)
    {
      return;
    }
#else
    app_testpat_build_frame_yuyv(dedicated_src, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
    app_clean_dcache_for_hw_read(dedicated_src, APP_UVC_ENC_FRAME_BYTES);
    app_invalidate_dcache_for_cpu_read(dedicated_src, APP_UVC_ENC_FRAME_BYTES);
#endif
#else
#if APP_UVC_TEST_PREBUILT_YUYV_MODE
    app_testpat_build_frame_from_prebuilt_yuyv(capture_buffer[0], APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
#else
    app_testpat_build_frame_yuyv(capture_buffer[0], APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
#endif
    app_clean_dcache_for_hw_read(capture_buffer[0], APP_UVC_ENC_FRAME_BYTES);
#endif
    g_uvc_test_pattern_ready = 1U;
#if APP_UVC_TEST_BOOT_LOG_ENABLE
#if APP_UVC_TEST_PREBUILT_YUYV_MODE
    printf("[UVC][TEST] loaded prebuilt YUYV frame %dx%d len=%lu src=0x%08lX cap0=0x%08lX\r\n",
           APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT,
           (unsigned long)g_app_uvc_test_yuyv_320x240_len,
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
           (unsigned long)app_get_test_src_cpu_buffer(),
#else
           (unsigned long)capture_buffer[0],
#endif
           (unsigned long)capture_buffer[0]);
#else
    printf("[UVC][TEST] generated synthetic frame %dx%d %s (left BLUE / right RED) src=0x%08lX cap0=0x%08lX\r\n",
           APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT,
           APP_UVC_TEST_SRC_FMT_NAME,
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
           (unsigned long)app_get_test_src_cpu_buffer(),
#else
           (unsigned long)capture_buffer[0],
#endif
           (unsigned long)capture_buffer[0]);
#endif
#endif
  }

  seq = ++g_uvc_test_pattern_seq;
  idx = 0;
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
  src_buf = app_get_test_src_cpu_buffer();
#else
  src_buf = capture_buffer[0];
#endif
  if (src_buf == NULL)
  {
    return;
  }
#if APP_UVC_TEST_FORCE_DEDICATED_SRC && APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  if (app_testpat_pull_dedicated_psram_to_axi(APP_UVC_ENC_FRAME_BYTES) != 0)
  {
    return;
  }
  src_buf = g_uvc_test_src_axi_staging;
#endif
  g_uvc_last_pipe1_seq = seq;
#endif /* APP_UVC_TEST_USE_DCMIPP_SOURCE */
#endif
#endif
#else
  seq = g_pipe1_completed_seq;
  if (seq == last_seq)
  {
    return;
  }
  last_seq = seq;
  g_uvc_last_pipe1_seq = seq;

  idx = g_pipe1_last_completed_idx;
  if ((idx < 0) || (idx >= CAPTURE_BUFFER_NB))
  {
    return;
  }
  src_buf = capture_buffer[idx];
#endif

  if (g_uvc_start_warmup_left > 0U)
  {
    g_uvc_start_warmup_left--;
    force_intra = 1;
    app_uvc_log_path_event("warmup", seq, idx, -1, 0);
#if APP_UVC_CB_LOG_ENABLE
    if (g_uvc_start_warmup_left == 0U)
    {
      printf("[UVC][START] warmup done, start encode/send\r\n");
    }
#endif
    return;
  }

#if APP_UVC_START_THROTTLE_ENABLE
  if ((APP_UVC_START_THROTTLE_DIV > 1U) && (g_uvc_start_throttle_left > 0U))
  {
    g_uvc_start_throttle_left--;
    if ((seq % APP_UVC_START_THROTTLE_DIV) != 0U)
    {
      g_uvc_start_throttle_skip_count++;
      force_intra = 1;
      app_uvc_log_path_event("throttle-skip", seq, idx, -1, 0);
      return;
    }

    g_uvc_start_throttle_keep_count++;
    if (g_uvc_start_throttle_left == 0U)
    {
      printf("[UVC][START] throttle done keep=%lu skip=%lu div=%lu win=%lu\r\n",
             (unsigned long)g_uvc_start_throttle_keep_count,
             (unsigned long)g_uvc_start_throttle_skip_count,
             (unsigned long)APP_UVC_START_THROTTLE_DIV,
             (unsigned long)APP_UVC_START_THROTTLE_WINDOW_FRAMES);
    }
  }
#endif

  g_uvc_bringup_submit_count++;
  g_uvc_last_submit_idx = idx;
#if APP_UVC_VERBOSE_SRC_LOG_ENABLE
  app_uvc_log_src_1s(src_buf, APP_UVC_ENC_FRAME_BYTES, seq, idx);
  app_uvc_log_yuv_probe_1s(src_buf, APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT);
#endif
#if APP_UVC_TEST_FORCE_DEDICATED_SRC && APP_UVC_TEST_FIXED_PSRAM_SRC_ENABLE && !APP_UVC_TEST_SRC_IN_AXI
  app_psram_src_guard_verify_1s(app_get_test_src_cpu_buffer(), APP_UVC_ENC_FRAME_BYTES);
#else
  app_psram_src_guard_verify_1s(src_buf, APP_UVC_ENC_FRAME_BYTES);
#endif

#if APP_UVC_TEST_STATIC_JPEG_MODE
  if (use_static_jpeg)
  {
    len = (int)g_app_uvc_test_jpeg_320x240_len;
    if ((len <= 0) || (len > (int)VENC_OUT_BUFFER_SIZE))
    {
      app_uvc_log_path_event("jpeg-len-bad", seq, idx, len, -1);
      return;
    }
    memcpy(uvc_in_buffers, g_app_uvc_test_jpeg_320x240, (size_t)len);
    show_ret = send_display(uvc_in_buffers, len);
    app_uvc_log_path_event("send-jpeg", seq, idx, len, show_ret);
    force_intra = 0;
    return;
  }
#else
  (void)use_static_jpeg;
#endif

#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_INJECT_JPEG_TO_VENC_OUT && !APP_UVC_TEST_VIEW_RAW_YUY2
  len = (int)g_app_uvc_test_jpeg_320x240_len;
  if ((len <= 0) || (len > (int)VENC_OUT_BUFFER_SIZE))
  {
    app_uvc_log_path_event("jpeg-venc-len-bad", seq, idx, len, -1);
    return;
  }
  memcpy(venc_out_buffer, g_app_uvc_test_jpeg_320x240, (size_t)len);
  show_ret = send_display(venc_out_buffer, len);
  app_uvc_log_path_event("send-jpeg-venc", seq, idx, len, show_ret);
  force_intra = 0;
  return;
#endif

#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_VIEW_RAW_YUY2
  len = APP_UVC_ENC_FRAME_BYTES;
  memcpy(uvc_in_buffers, src_buf, (size_t)len);
  show_ret = send_display(uvc_in_buffers, len);
  app_uvc_log_path_event("send-yuy2", seq, idx, len, show_ret);
  force_intra = 0;
  return;
#endif

#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_CUSTOM_ENCODER_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2
  len = app_custom_test_encoder_yuyv_to_jpeg(src_buf, APP_UVC_ENC_FRAME_BYTES,
                                             venc_out_buffer, (int)VENC_OUT_BUFFER_SIZE);
  if (len > 0)
  {
    show_ret = send_display(venc_out_buffer, len);
    app_uvc_log_path_event("send-myenc", seq, idx, len, show_ret);
  }
  else
  {
    app_uvc_log_path_event("myenc-fail", seq, idx, len, -1);
  }
  force_intra = 0;
  return;
#endif

#if APP_UVC_TEST_PATTERN_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2 && APP_UVC_TEST_FREEZE_AFTER_FIRST_ENCODE
  if ((g_uvc_test_encoded_ready != 0U) && (g_uvc_test_encoded_len > 0))
  {
    len = g_uvc_test_encoded_len;
    show_ret = send_display(venc_out_buffer, len);
    app_uvc_log_path_event("send-frozen", seq, idx, len, show_ret);
    force_intra = 0;
    return;
  }
#endif

  len = (int)encode_display(force_intra, src_buf);
  if (len > 0)
  {
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_SEND_DIRECT_ENC_BUF && !APP_UVC_TEST_VIEW_RAW_YUY2
    show_ret = send_display(venc_out_buffer, len);
#else
    show_ret = send_display(uvc_in_buffers, len);
#endif
    app_uvc_log_path_event("send", seq, idx, len, show_ret);
#if APP_UVC_TEST_PATTERN_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2 && APP_UVC_TEST_FREEZE_AFTER_FIRST_ENCODE
    if (g_uvc_test_encoded_ready == 0U)
    {
      g_uvc_test_encoded_ready = 1U;
      g_uvc_test_encoded_len = len;
    }
#endif
    force_intra = 0;
  }
  else
  {
    app_uvc_log_path_event("enc-fail", seq, idx, len, 0);
  }
#endif
}

static int build_display_inference_info(uint8_t *p_buffer, uint32_t inf_time, int line_nb)
{
  const int offset_x = 16;

  DRAW_PrintfArgbHw(&INF_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset_x, line_nb * INF_INFO_FONT.height,
                    " Inference : %4.1f ms ", (double)inf_time);

  return line_nb + 1;
}

static int build_display_cpu_load(uint8_t *p_buffer, int line_nb)
{
  const int offset_x = 16;
  float cpu_load_one_second;

  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);
  DRAW_PrintfArgbHw(&INF_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset_x, line_nb * INF_INFO_FONT.height,
                    " Cpu load  : %4.1f  %% ", cpu_load_one_second);
  line_nb++;

  return line_nb;
}

static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < 0)
    *x = 0;
  if (*y < 0)
    *y = 0;
  if (*x >= VENC_WIDTH)
    *x = VENC_WIDTH - 1;
  if (*y >= VENC_HEIGHT)
    *y = VENC_HEIGHT - 1;

  return (xi != *x) || (yi != *y);
}

static void convert_length(float32_t wi, float32_t hi, int *wo, int *ho)
{
  *wo = (int) (VENC_WIDTH * wi);
  *ho = (int) (VENC_HEIGHT * hi);
}

static void convert_point(float32_t xi, float32_t yi, int *xo, int *yo)
{
  *xo = (int) (VENC_WIDTH * xi);
  *yo = (int) (VENC_HEIGHT * yi);
}

static void cvt_nn_box_to_dp_box(od_pp_outBuffer_t *detect, box_t *box_dp)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;

  convert_point(detect->x_center, detect->y_center, &xc, &yc);
  convert_length(detect->width, detect->height, &w, &h);
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  box_dp->x = x0;
  box_dp->y = y0;
  box_dp->w = x1 - x0;
  box_dp->h = y1 - y0;
  box_dp->conf = detect->conf;
}

static void draw_box(uint8_t *p_buffer, od_pp_outBuffer_t *box_nn)
{
  box_t box_disp;

  cvt_nn_box_to_dp_box(box_nn, &box_disp);
  DRAW_RectArgbHw(p_buffer, VENC_WIDTH, VENC_HEIGHT, box_disp.x, box_disp.y, box_disp.w, box_disp.h, OBJ_RECT_COLOR);
  DRAW_PrintfArgbHw(&CONF_LEVEL_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, box_disp.x, box_disp.y, "%5.1f %%",
                    box_disp.conf * 100);
}

static void time_stat_display(time_stat_t *p_stat, uint8_t *p_buffer, char *label, int line_nb, int indent)
{
  int offset = VENC_WIDTH - 41 * DBG_INFO_FONT.width;

  DRAW_PrintfArgbHw(&DBG_INFO_FONT, p_buffer, VENC_WIDTH, VENC_HEIGHT, offset, line_nb * DBG_INFO_FONT.height,
                    "%*s%s : %3d ms / %5.1f ms ", indent + 1, "", label, p_stat->last, p_stat->mean);
}

static int build_display_nn_dbg(uint8_t *p_buffer, stat_info_t *si, int line_nb)
{
  time_stat_display(&si->nn_total_time, p_buffer,     "NN thread stats  ", line_nb++, 0);
  time_stat_display(&si->nn_inference_time, p_buffer, "inference    ", line_nb++, 4);

  return line_nb;
}

static int build_display_disp_dbg(uint8_t *p_buffer, stat_info_t *si, int line_nb)
{
  time_stat_display(&si->disp_total_time, p_buffer,   "DISP thread stats", line_nb++, 0);
  time_stat_display(&si->nn_pp_time, p_buffer,        "pp           " , line_nb++, 4);
  time_stat_display(&si->disp_display_time, p_buffer, "display      ", line_nb++, 4);
  time_stat_display(&si->disp_enc_time, p_buffer,     "encode       ", line_nb++, 4);

  return line_nb;
}

static int update_and_capture_debug_enabled()
{
  static int prev_button_state = GPIO_PIN_RESET;
  static int display_debug_enabled = 0;
  int cur_button_state;

  cur_button_state = BSP_PB_GetState(BUTTON_USER1);
  if (cur_button_state == GPIO_PIN_SET && prev_button_state == GPIO_PIN_RESET)
    display_debug_enabled = !display_debug_enabled;
  prev_button_state = cur_button_state;

  return display_debug_enabled;
}

static void build_display_stat_info(uint8_t *p_buffer, stat_info_t *si)
{
  int line_nb = 1;

  if (!update_and_capture_debug_enabled())
    return ;

  line_nb = build_display_nn_dbg(p_buffer, si, line_nb);
  line_nb = build_display_disp_dbg(p_buffer, si, line_nb);
}

static void build_display(uint8_t *p_buffer, od_pp_out_t *pp_out)
{
  const uint8_t *fig_array[] = {fig0, fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9};
  int line_nb = VENC_HEIGHT / INF_INFO_FONT.height - 4;
  stat_info_t si_copy;
  int nb;
  int i;

  stat_info_copy(&si_copy);

  for (i = 0; i < pp_out->nb_detect; i++)
    draw_box(p_buffer, &pp_out->pOutBuff[i]);

  line_nb = build_display_inference_info(p_buffer, si_copy.nn_inference_time.last, line_nb);
  line_nb = build_display_cpu_load(p_buffer, line_nb);

  nb = MIN(pp_out->nb_detect, ARRAY_NB(fig_array) - 1);
  DRAW_CopyArgbHW(p_buffer, VENC_WIDTH, VENC_HEIGHT, (uint8_t *) fig_array[nb], 64, 64, 16, 16);

  build_display_stat_info(p_buffer, &si_copy);
}

static void display_frame(int is_intra_force, od_pp_out_t *pp_out)
{
  uint8_t *dp_buffer = capture_buffer[capture_buffer_disp_idx];
  uint32_t ts;
  int len;

  ts = HAL_GetTick();
  build_display(dp_buffer, pp_out);
  time_stat_update(&stat_info.disp_display_time, HAL_GetTick() - ts);

  ts = HAL_GetTick();
  len = encode_display(is_intra_force, dp_buffer);
  time_stat_update(&stat_info.disp_enc_time, HAL_GetTick() - ts);

  if (len > 0)
    send_display(uvc_in_buffers, len);
}

static int display_new_frame(od_pp_out_t *pp_out)
{
  static int uvc_is_active_prev = 0;
  int uvc_is_active_local = uvc_is_active;

  if (uvc_is_active_local) {
    display_frame(!uvc_is_active_prev || force_intra, pp_out);
    force_intra = 0;
  }

  uvc_is_active_prev = uvc_is_active_local;

  return uvc_is_active_local;
}

#if !APP_DVP_BRINGUP_PIPE1_ONLY
static void dp_thread_fct(void *arg)
{
  od_yolov2_pp_static_param_t pp_params;
  stai_network_info info;
  od_pp_out_t pp_output;
  uint32_t total_ts;
  void *pp_input;
  int is_dp_done;
  uint32_t ts;
  int ret;

  /* setup post process */
  ret = stai_network_get_info(network_ctx, &info);
  assert(ret == STAI_SUCCESS);
  app_postprocess_init(&pp_params, &info);
  while (1)
  {
    uint8_t *output_buffer;

    output_buffer = bqueue_get_ready(&nn_output_queue);
    assert(output_buffer);
    total_ts = HAL_GetTick();

    /* Do post process */
    ts = HAL_GetTick();
    pp_input = (void *) output_buffer;
    pp_output.pOutBuff = NULL;
    ret = app_postprocess_run((void * []){pp_input}, 1, &pp_output, &pp_params);
    assert(ret == AI_OD_POSTPROCESS_ERROR_NO);
    time_stat_update(&stat_info.nn_pp_time, HAL_GetTick() - ts);
    cpuload_update(&cpu_load);

    /* compose + encode + send */
    is_dp_done = display_new_frame(&pp_output);

    if (is_dp_done)
      time_stat_update(&stat_info.disp_total_time, HAL_GetTick() - total_ts);

    bqueue_put_free(&nn_output_queue);
  }
}
#endif

static void isp_thread_fct(void *arg)
{
#if !APP_UVC_TEST_PATTERN_MODE
  int ret;
#endif

  while (1) {
#if APP_UVC_TEST_PATTERN_MODE
    vTaskDelay(pdMS_TO_TICKS(1));
#else
    ret = xSemaphoreTake(isp_sem, pdMS_TO_TICKS(1000));
    if (ret == pdTRUE)
    {
      CAM_IspUpdate();
    }
#endif

#if APP_DVP_BRINGUP_PIPE1_ONLY
    app_uvc_try_send_pipe1_bringup();
#endif
    app_flush_cam_irq_logs();
    app_log_cam_health_1s();
    app_log_dcmipp_error_poll_1s();
    app_log_uvc_health_1s();
    app_log_uvc_perf_1s();
    app_dump_pipe1_frame_data_1s();
  }
}

static void app_uvc_streaming_active(struct uvcl_callbacks *cbs, UVCL_StreamConf_t stream)
{
  uint32_t now_ms = HAL_GetTick();
  DCMIPP_HandleTypeDef *hdcmipp = CMW_CAMERA_GetDCMIPPHandle();

  (void)cbs;
  uvc_is_active = 1;
  g_uvc_stream_active_count++;
  g_uvc_stream_session_id++;
  g_uvc_stream_start_tick_ms = now_ms;
  g_uvc_start_warmup_left = APP_UVC_START_WARMUP_FRAMES;
#if APP_UVC_START_THROTTLE_ENABLE
  g_uvc_start_throttle_left = APP_UVC_START_THROTTLE_WINDOW_FRAMES;
#else
  g_uvc_start_throttle_left = 0U;
#endif
  g_uvc_start_throttle_skip_count = 0U;
  g_uvc_start_throttle_keep_count = 0U;
  g_uvc_path_last_log_ms = 0U;
  g_uvc_last_enc_ms = 0U;
  g_uvc_last_send_ms = 0U;
  g_uvc_last_pipe1_seq = g_pipe1_completed_seq;
  g_pipe_error_start_recover_count = 0U;
#if APP_UVC_TEST_PATTERN_MODE
  g_uvc_test_pattern_seq = 0U;
#if !APP_UVC_TEST_STATIC_JPEG_MODE
  g_uvc_test_encoded_ready = 0U;
  g_uvc_test_encoded_len = 0;
#endif
#endif
  force_intra = 1;
  BSP_LED_On(LED_RED);
#if APP_UVC_CB_LOG_ENABLE
  printf("[UVC][START] session=%lu tick_ms=%lu cfg=%dx%d@%d payload=%d on=%lu off=%lu warmup_drop=%lu throttle(win=%lu div=%lu)\r\n",
         (unsigned long)g_uvc_stream_session_id,
         (unsigned long)now_ms,
         stream.width, stream.height, stream.fps, stream.payload_type,
         (unsigned long)g_uvc_stream_active_count,
         (unsigned long)g_uvc_stream_inactive_count,
         (unsigned long)g_uvc_start_warmup_left,
         (unsigned long)g_uvc_start_throttle_left,
         (unsigned long)APP_UVC_START_THROTTLE_DIV);
#elif APP_UVC_LOG_ENABLE
  printf("[UVC] streaming active %dx%d@%d payload=%d\r\n",
         stream.width, stream.height, stream.fps, stream.payload_type);
#else
  (void)stream;
#endif

  if ((hdcmipp != NULL) && (hdcmipp->Instance != NULL))
  {
    uint32_t cmsr2 = READ_REG(hdcmipp->Instance->CMSR2);
    uint32_t cmier = READ_REG(hdcmipp->Instance->CMIER);
    uint32_t err = hdcmipp->ErrorCode;
    uint32_t p1_state = HAL_DCMIPP_PIPE_GetState(hdcmipp, DCMIPP_PIPE1);
    uint32_t p1_ovrf = ((cmsr2 & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U;
    uint32_t p1_ovrie = ((cmier & DCMIPP_IT_PIPE1_OVR) != 0U) ? 1U : 0U;
    uint32_t p1_ovrerr = ((err & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U) ? 1U : 0U;
#if APP_UVC_TEST_BOOT_LOG_ENABLE
    printf("[CAM][ERR][SNAP][UVC-START] p1_state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX p1_ovrf=%lu p1_ovrie=%lu p1_ovrerr=%lu\r\n",
           (unsigned long)p1_state,
           (unsigned long)err,
           (unsigned long)cmsr2,
           (unsigned long)cmier,
           (unsigned long)p1_ovrf,
           (unsigned long)p1_ovrie,
           (unsigned long)p1_ovrerr);
#endif

#if APP_UVC_START_OVR_RECOVER_ENABLE
    if ((p1_state == HAL_DCMIPP_PIPE_STATE_ERROR) || (p1_ovrf != 0U) || (p1_ovrerr != 0U) || (p1_ovrie == 0U))
    {
      uint32_t cmsr2_after;
      uint32_t cmier_after;
      uint32_t err_after;
      uint32_t state_after;

      __HAL_DCMIPP_CLEAR_FLAG(hdcmipp, DCMIPP_FLAG_PIPE1_OVR);
      hdcmipp->ErrorCode &= ~HAL_DCMIPP_ERROR_PIPE1_OVR;
      if (hdcmipp->PipeState[1] == HAL_DCMIPP_PIPE_STATE_ERROR)
      {
        hdcmipp->PipeState[1] = HAL_DCMIPP_PIPE_STATE_BUSY;
      }
      __HAL_DCMIPP_ENABLE_IT(hdcmipp, DCMIPP_IT_PIPE1_OVR);
      __DSB();

      cmsr2_after = READ_REG(hdcmipp->Instance->CMSR2);
      cmier_after = READ_REG(hdcmipp->Instance->CMIER);
      err_after = hdcmipp->ErrorCode;
      state_after = HAL_DCMIPP_PIPE_GetState(hdcmipp, DCMIPP_PIPE1);

#if APP_UVC_TEST_BOOT_LOG_ENABLE
      printf("[CAM][ERR][UVC-START-RECOVER] state=%lu->%lu err=0x%08lX->0x%08lX cmsr2=0x%08lX->0x%08lX cmier=0x%08lX->0x%08lX\r\n",
             (unsigned long)p1_state,
             (unsigned long)state_after,
             (unsigned long)err,
             (unsigned long)err_after,
             (unsigned long)cmsr2,
             (unsigned long)cmsr2_after,
             (unsigned long)cmier,
             (unsigned long)cmier_after);
#endif
    }
#endif
#if !APP_UVC_TEST_BOOT_LOG_ENABLE && (!defined(APP_UVC_START_OVR_RECOVER_ENABLE) || !(APP_UVC_START_OVR_RECOVER_ENABLE))
    (void)cmsr2;
    (void)cmier;
    (void)err;
    (void)p1_state;
    (void)p1_ovrf;
    (void)p1_ovrie;
    (void)p1_ovrerr;
#endif
  }
}

static void app_uvc_streaming_inactive(struct uvcl_callbacks *cbs)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t dur_ms = now_ms - g_uvc_stream_start_tick_ms;

  (void)cbs;
  uvc_is_active = 0;
  g_uvc_start_warmup_left = 0U;
  g_uvc_start_throttle_left = 0U;
  g_uvc_stream_inactive_count++;
  BSP_LED_Off(LED_RED);
#if APP_UVC_CB_LOG_ENABLE
  printf("[UVC][STOP] session=%lu tick_ms=%lu dur_ms=%lu flying=%d on=%lu off=%lu show_ok=%lu rel=%lu\r\n",
         (unsigned long)g_uvc_stream_session_id,
         (unsigned long)now_ms,
         (unsigned long)dur_ms,
         buffer_flying,
         (unsigned long)g_uvc_stream_active_count,
         (unsigned long)g_uvc_stream_inactive_count,
         (unsigned long)g_uvc_show_ok_count,
         (unsigned long)g_uvc_frame_release_count);
#elif APP_UVC_LOG_ENABLE
  printf("[UVC] streaming inactive flying=%d show_ok=%lu rel=%lu\r\n",
         buffer_flying,
         (unsigned long)g_uvc_show_ok_count,
         (unsigned long)g_uvc_frame_release_count);
#else
  (void)dur_ms;
#endif
}

static void app_uvc_frame_release(struct uvcl_callbacks *cbs, void *frame)
{
  uint32_t sig;

  if (!buffer_flying)
  {
#if APP_UVC_LOG_ENABLE
    printf("[UVC][WARN] frame_release while no frame flying frame=0x%08lX\r\n", (unsigned long)frame);
#else
    (void)cbs;
    (void)frame;
#endif
    return;
  }

  g_uvc_tx_release_id++;
  g_uvc_tx_last_release_ptr = (uintptr_t)frame;
  sig = app_sum_prefix_u32(uvc_in_buffers, (int)g_uvc_tx_last_submit_len, APP_UVC_TX_SIG_BYTES);
  g_uvc_tx_last_release_sig = sig;
  if ((uintptr_t)frame != (uintptr_t)uvc_in_buffers)
  {
    g_uvc_tx_ptr_mismatch_count++;
#if APP_UVC_LOG_ENABLE
    printf("[UVC][TX][WARN] release ptr mismatch got=0x%08lX exp=0x%08lX cnt=%lu\r\n",
           (unsigned long)frame,
           (unsigned long)uvc_in_buffers,
           (unsigned long)g_uvc_tx_ptr_mismatch_count);
#endif
  }
  if (g_uvc_tx_last_release_sig != g_uvc_tx_last_submit_sig)
  {
    g_uvc_tx_sig_mismatch_count++;
#if APP_UVC_LOG_ENABLE
    printf("[UVC][TX][WARN] release sig mismatch sub=%lu rel=%lu len=%lu cnt=%lu\r\n",
           (unsigned long)g_uvc_tx_last_submit_sig,
           (unsigned long)g_uvc_tx_last_release_sig,
           (unsigned long)g_uvc_tx_last_submit_len,
           (unsigned long)g_uvc_tx_sig_mismatch_count);
#endif
  }
  g_uvc_frame_release_count++;
  buffer_flying = 0;
}

#if APP_UVC_TEST_BOOT_LOG_ENABLE
static void app_display_info_header()
{
  printf("========================================\n\r");
  printf("x-cube-n6-ai-mjpeg-usb-uvc v2.2.0--- hinoeng app 09:13 (%s)\n\r", APP_VERSION_STRING);
  printf("Build date & time: %s %s\n\r", __DATE__, __TIME__);
#if defined(__GNUC__)
  printf("Compiler: GCC %d.%d.%d\n\r", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(__ICCARM__)
  printf("Compiler: IAR EWARM %d.%d.%d\r\n", __VER__ / 1000000, (__VER__ / 1000) % 1000 ,__VER__ % 1000);
#else
  printf("Compiler: Unknown\n\r");
#endif
  printf("HAL: %lu.%lu.%lu\n\r", __STM32N6xx_HAL_VERSION_MAIN, __STM32N6xx_HAL_VERSION_SUB1, __STM32N6xx_HAL_VERSION_SUB2);
  printf("STEdgeAI Tools: %d.%d.%d\n\r", STAI_TOOLS_VERSION_MAJOR, STAI_TOOLS_VERSION_MINOR, STAI_TOOLS_VERSION_MICRO);
  printf("NN model: %s\n\r", STAI_NETWORK_ORIGIN_MODEL_NAME);
  printf("========================================\n\r");
}
#endif

void app_run()
{
  UBaseType_t isp_priority = FREERTOS_PRIORITY(2);
#if !APP_DVP_BRINGUP_PIPE1_ONLY
  UBaseType_t dp_priority = FREERTOS_PRIORITY(-2);
  UBaseType_t nn_priority = FREERTOS_PRIORITY(1);
#endif
  UVCL_Conf_t uvcl_conf = { 0 };
#if !(APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_CUSTOM_ENCODER_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2)
  ENC_Conf_t enc_conf = { 0 };
#endif
  TaskHandle_t hdl;
  int ret;

#if APP_UVC_TEST_BOOT_LOG_ENABLE
  app_display_info_header();
  printf("[APP] cfg sensor=%dx%d venc=%dx%d pipe1_only=%d\r\n",
         SENSOR_DVP_WIDTH,
         SENSOR_DVP_HEIGHT,
         VENC_DVP_WIDTH,
         VENC_DVP_HEIGHT,
         APP_DVP_BRINGUP_PIPE1_ONLY);
  printf("[PSRAM][SUM] init=%d readid=%d id=%02X %02X %02X %02X %02X %02X write=%d read=%d mismatch=%d mmp=%d rd8=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
         g_psram_init_ret,
         g_psram_readid_ret,
         g_psram_id[0], g_psram_id[1], g_psram_id[2], g_psram_id[3], g_psram_id[4], g_psram_id[5],
         g_psram_write_ret,
         g_psram_read_ret,
         g_psram_verify_mismatch,
         g_psram_mmp_ret,
         g_psram_probe_rd8[0], g_psram_probe_rd8[1], g_psram_probe_rd8[2], g_psram_probe_rd8[3],
         g_psram_probe_rd8[4], g_psram_probe_rd8[5], g_psram_probe_rd8[6], g_psram_probe_rd8[7]);
  printf("[APP] buf capture0=0x%08lX captureN=%d nn_in0=0x%08lX nn_out0=0x%08lX\r\n",
         (unsigned long)capture_buffer[0],
         CAPTURE_BUFFER_NB,
         (unsigned long)nn_input_buffers[0],
         (unsigned long)nn_output_buffers[0]);
#endif
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  ret = BSP_PB_Init(BUTTON_USER1, BUTTON_MODE_GPIO);
  assert(ret == BSP_ERROR_NONE);

  cpuload_init(&cpu_load);

  /* create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);
  ret = bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
  assert(ret == 0);

  /* setup fonts */
  ret = DRAW_FontSetup(&Font12, &font_12);
  assert(ret == 0);
  ret = DRAW_FontSetup(&Font16, &font_16);
  assert(ret == 0);

  /* Enable venc */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
   LL_VENC_Init();

  /*** Camera Init ************************************************************/  
  CAM_Init();

  /* Decide runtime YUV byte order for software JPEG path. */
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_USE_DCMIPP_SOURCE
#if (CAPTURE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1_UYVY)
  g_uvc_runtime_src_fmt = APP_UVC_TEST_SRC_FMT_UYVY;
#else
  g_uvc_runtime_src_fmt = APP_UVC_TEST_SRC_FMT_YUYV;
#endif
#if (APP_UVC_LIVE_SRC_FMT_OVERRIDE != 0)
  g_uvc_runtime_src_fmt = APP_UVC_LIVE_SRC_FMT_OVERRIDE;
  printf("==== UVC FIX: LIVE SRC_FMT OVERRIDE=%s ====\r\n",
         app_uvc_src_fmt_name(g_uvc_runtime_src_fmt));
#endif
  printf("==== UVC FIX: LIVE DCMIPP SRC_FMT=%s (from CAPTURE_FORMAT) ====\r\n",
         app_uvc_src_fmt_name(g_uvc_runtime_src_fmt));
  printf("==== UVC FIX: DISABLE FORCE APP_UVC_TEST_SRC_FMT IN LIVE MODE ====\r\n");
#else
  g_uvc_runtime_src_fmt = APP_UVC_TEST_SRC_FMT;
  printf("==== UVC FIX: PATTERN SRC_FMT=%s (from APP_UVC_TEST_SRC_FMT) ====\r\n",
         app_uvc_src_fmt_name(g_uvc_runtime_src_fmt));
#endif

  /* Encoder init */
#if !(APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_CUSTOM_ENCODER_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2)
  enc_conf.width = APP_UVC_ENC_WIDTH;
  enc_conf.height = APP_UVC_ENC_HEIGHT;
#if APP_UVC_TEST_PATTERN_MODE
  enc_conf.fps = APP_UVC_TEST_STREAM_FPS;
#else
  enc_conf.fps = CAMERA_FPS;
#endif
#if (CAPTURE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
  enc_conf.input_type = ENC_INPUT_YUV422_YUYV;
#elif (CAPTURE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1_UYVY)
  enc_conf.input_type = ENC_INPUT_YUV422_UYVY;
#else
  enc_conf.input_type = ENC_INPUT_RGB888;
#endif
#if APP_UVC_TEST_PATTERN_MODE
#if APP_UVC_TEST_USE_DCMIPP_SOURCE
  switch (g_uvc_runtime_src_fmt)
  {
    case APP_UVC_TEST_SRC_FMT_UYVY:
    case APP_UVC_TEST_SRC_FMT_VYUY:
      enc_conf.input_type = ENC_INPUT_YUV422_UYVY;
      break;
    case APP_UVC_TEST_SRC_FMT_YVYU:
    case APP_UVC_TEST_SRC_FMT_YUYV:
    default:
      enc_conf.input_type = ENC_INPUT_YUV422_YUYV;
      break;
  }
  printf("==== UVC FIX: ENC INPUT FOLLOWS LIVE SRC_FMT=%s ====\r\n",
         app_uvc_src_fmt_name(g_uvc_runtime_src_fmt));
#else
#if (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_YUYV)
  enc_conf.input_type = ENC_INPUT_YUV422_YUYV;
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[UVC][TEST] quick-color-test: force encoder input YUYV (match synthetic source)\r\n");
#endif
#elif (APP_UVC_TEST_SRC_FMT == APP_UVC_TEST_SRC_FMT_UYVY)
  enc_conf.input_type = ENC_INPUT_YUV422_UYVY;
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[UVC][TEST] quick-color-test: force encoder input UYVY (match synthetic source)\r\n");
#endif
#endif
#endif
#endif
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[APP] enc cfg %dx%d@%d input_type=%d cap_fmt=%d bpp=%d\r\n",
         enc_conf.width,
         enc_conf.height,
         enc_conf.fps,
         (int)enc_conf.input_type,
         (int)CAPTURE_FORMAT,
         CAPTURE_BPP);
#endif
  ENC_Init(&enc_conf);
#endif
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_CUSTOM_ENCODER_MODE && !APP_UVC_TEST_VIEW_RAW_YUY2
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[ENC][SW] custom software JPEG encoder enabled: skip ENC_Init/ENC_EncodeFrame path\r\n");
#endif
#endif

  /* Uvc init */
  uvcl_conf.streams[0].width = APP_UVC_ENC_WIDTH;
  uvcl_conf.streams[0].height = APP_UVC_ENC_HEIGHT;
#if APP_UVC_TEST_PATTERN_MODE
  uvcl_conf.streams[0].fps = APP_UVC_TEST_STREAM_FPS;
#else
  uvcl_conf.streams[0].fps = CAMERA_FPS;
#endif
#if APP_UVC_TEST_PATTERN_MODE && APP_UVC_TEST_VIEW_RAW_YUY2
  uvcl_conf.streams[0].payload_type = UVCL_PAYLOAD_UNCOMPRESSED_YUY2;
  uvcl_conf.streams[0].dwMaxVideoFrameSize = APP_UVC_ENC_FRAME_BYTES;
#else
  uvcl_conf.streams[0].payload_type = UVCL_PAYLOAD_JPEG;
  uvcl_conf.streams[0].dwMaxVideoFrameSize = VENC_OUT_BUFFER_SIZE;
#endif
  uvcl_conf.streams_nb = 1;
  uvcl_conf.is_immediate_mode = 1;
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[UVC][CFG] stream=%dx%d@%d payload=%d max_frame=%lu venc_out=0x%08lX test_mode=%d\r\n",
         uvcl_conf.streams[0].width,
         uvcl_conf.streams[0].height,
         uvcl_conf.streams[0].fps,
         uvcl_conf.streams[0].payload_type,
         (unsigned long)uvcl_conf.streams[0].dwMaxVideoFrameSize,
         (unsigned long)venc_out_buffer,
         APP_UVC_TEST_PATTERN_MODE);
#endif
  uvcl_cbs.streaming_active = app_uvc_streaming_active;
  uvcl_cbs.streaming_inactive = app_uvc_streaming_inactive;
  uvcl_cbs.frame_release = app_uvc_frame_release;
  ret = UVCL_Init(USB1_OTG_HS, &uvcl_conf, &uvcl_cbs);
#if APP_UVC_LOG_ENABLE
  if (ret == 0)
  {
    printf("[UVC] UVCL_Init OK (USB1_OTG_HS)\r\n");
  }
  else
  {
    printf("[UVC][ERR] UVCL_Init failed ret=%d\r\n", ret);
  }
#endif

  /* sems + mutex init */
  isp_sem = xSemaphoreCreateCountingStatic(1, 0, &isp_sem_buffer);
  assert(isp_sem);
  dma2d_sem = xSemaphoreCreateCountingStatic(1, 0, &dma2d_sem_buffer);
  assert(dma2d_sem);
  dma2d_lock = xSemaphoreCreateMutexStatic(&dma2d_lock_buffer);
  assert(dma2d_lock);
  stat_info_lock = xSemaphoreCreateMutexStatic(&stat_info_lock_buffer);
  assert(stat_info_lock);

  /* Start LCD Display camera pipe stream */
#if APP_UVC_TEST_PATTERN_MODE
#if APP_UVC_TEST_USE_DCMIPP_SOURCE
  {
    uint8_t *dedicated_src = app_get_test_src_cpu_buffer();
    if (dedicated_src == NULL)
    {
      dedicated_src = capture_buffer[0];
    }
    CAM_DisplayPipe_Start(dedicated_src, CMW_MODE_CONTINUOUS);
  }
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[CAM] test-pattern mode + DCMIPP source: pipe1 camera start enabled\r\n");
  printf("[UVC][TEST] source=live DCMIPP pipe1 -> g_uvc_test_src_buffer (direct)\r\n");
#endif
#else
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[CAM] test-pattern mode enabled: pipe1 camera start skipped\r\n");
#if APP_UVC_TEST_STATIC_JPEG_MODE
  printf("[UVC][TEST] source=static JPEG 320x240 left BLUE / right RED (encoder bypass)\r\n");
  printf("[UVC][TEST] static_jpeg_len=%lu max_frame=%u\r\n",
         (unsigned long)g_app_uvc_test_jpeg_320x240_len,
         (unsigned int)VENC_OUT_BUFFER_SIZE);
#else
#if APP_UVC_TEST_PREBUILT_YUYV_MODE
  printf("[UVC][TEST] source=prebuilt YUYV 320x240 (generated from static JPEG)\r\n");
  printf("[UVC][TEST] prebuilt_yuyv_len=%lu fmt=%s\r\n",
         (unsigned long)g_app_uvc_test_yuyv_320x240_len,
         APP_UVC_TEST_SRC_FMT_NAME);
#else
  printf("[UVC][TEST] source=synthesized %s frame: left BLUE / right RED\r\n",
         APP_UVC_TEST_SRC_FMT_NAME);
#endif
#if APP_UVC_TEST_VIEW_RAW_YUY2
  printf("[UVC][TEST] view_mode=raw YUY2 over UVC (encoder bypass for color diagnosis)\r\n");
#else
  printf("[UVC][TEST] view_mode=JPEG via encoder (YUV->JPEG path)\r\n");
#if APP_UVC_SEND_DIRECT_ENC_BUF
  printf("[UVC][TEST] tx_source=direct venc_out_buffer (skip memcpy to uvc_in_buffers)\r\n");
#else
  printf("[UVC][TEST] tx_source=uvc_in_buffers (with memcpy from encoder output)\r\n");
#endif
  printf("[ENC][BUF] yuv_src=%s jpeg_after_encode=venc_out_buffer@0x%08lX jpeg_before_uvc=uvc_in_buffers@0x%08lX out_size=%u\r\n",
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
         "g_uvc_test_src_buffer",
#else
         "capture_buffer[0]",
#endif
         (unsigned long)venc_out_buffer,
         (unsigned long)uvc_in_buffers,
         (unsigned int)VENC_OUT_BUFFER_SIZE);
#endif
#if APP_UVC_TEST_SMALL_FRAME_ENABLE
  printf("[UVC][TEST] small-frame mode enabled: %dx%d (non-PSRAM src=%d)\r\n",
         APP_UVC_ENC_WIDTH, APP_UVC_ENC_HEIGHT, APP_UVC_TEST_SRC_IN_AXI);
#endif
#if APP_UVC_TEST_FORCE_DEDICATED_SRC
  printf("[UVC][TEST] src_mode=dedicated_buffer (not capture_buffer[0])\r\n");
#else
  printf("[UVC][TEST] src_mode=capture_buffer[0]\r\n");
#endif
#endif
#endif
#endif
#else
  CAM_DisplayPipe_Start(capture_buffer[0], CMW_MODE_CONTINUOUS);
#endif

  /* threads init */
#if APP_DVP_BRINGUP_PIPE1_ONLY
#if APP_UVC_TEST_BOOT_LOG_ENABLE
  printf("[CAM] bring-up mode enabled: NN/PIPE2 and DP threads are skipped\r\n");
#endif
#if APP_UVC_LOG_ENABLE
  printf("[UVC] bring-up mode: direct pipe1->ENC->UVCL path enabled in ISP thread\r\n");
#endif
#else
  hdl = xTaskCreateStatic(nn_thread_fct, "nn", configMINIMAL_STACK_SIZE * 2, NULL, nn_priority, nn_thread_stack,
                          &nn_thread);
  assert(hdl != NULL);
  hdl = xTaskCreateStatic(dp_thread_fct, "dp", configMINIMAL_STACK_SIZE * 2, NULL, dp_priority, dp_thread_stack,
                          &dp_thread);
  assert(hdl != NULL);
#endif
  hdl = xTaskCreateStatic(isp_thread_fct, "isp", configMINIMAL_STACK_SIZE * 2, NULL, isp_priority, isp_thread_stack,
                          &isp_thread);
  assert(hdl != NULL);

  BSP_LED_On(LED_GREEN);
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
#if APP_UVC_TEST_PATTERN_MODE && !APP_UVC_TEST_USE_DCMIPP_SOURCE
  (void)pipe;
  return HAL_OK;
#else
  /* Keep ISR callback minimal to avoid capture instability during bring-up. */
  if (pipe == DCMIPP_PIPE1)
  {
    g_pipe1_frame_irq_count++;
    (void)app_main_pipe_frame_event();
  }
  else if (pipe == DCMIPP_PIPE2)
  {
    g_pipe2_frame_irq_count++;
    app_ancillary_pipe_frame_event();
  }

  return HAL_OK;
#endif
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if ((pipe == DCMIPP_PIPE1) && (g_pipe1_first_vsync_logged == 0))
  {
    g_pipe1_first_vsync_logged = 1;
    g_pipe1_first_vsync_pending_log = 1;
  }

  if (pipe == DCMIPP_PIPE1)
  {
    g_pipe1_vsync_irq_count++;
  }

  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}

void DRAW_HwLock(void *dma2d_handle)
{
  int ret;

  ret = xSemaphoreTake(dma2d_lock, portMAX_DELAY);
  assert(ret == pdTRUE);

  dma2d_current = dma2d_handle;
}

void DRAW_HwUnlock()
{
  int ret;

  ret = xSemaphoreGive(dma2d_lock);
  assert(ret == pdTRUE);
}

void DRAW_Wfe()
{
  int ret;

  ret = xSemaphoreTake(dma2d_sem, portMAX_DELAY);
  assert(ret == pdTRUE);
}

void DRAW_Signal()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ret;

  ret = xSemaphoreGiveFromISR(dma2d_sem, &xHigherPriorityTaskWoken);
  assert(ret == pdTRUE);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(dma2d_current);
}
