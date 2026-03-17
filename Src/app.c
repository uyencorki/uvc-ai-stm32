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

#include "app_cam.h"
#include "app_config.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "cmw_camera.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6570_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app_enc.h"
#include "utils.h"
#include "uvcl.h"
#include "draw.h"
#include "stai.h"
#include "stai_network.h"

#include "figs.h"

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
#define APP_PIPE1_FRAME_DUMP_BYTES 16
#define APP_PIPE1_FRAME_DUMP_PERIOD_MS 5000U
#define APP_CAM_HEALTH_LOG_ENABLE 1
#define APP_DCMIPP_ERR_LOG_ENABLE 1
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

/* Encoded bitstream debug: log at low rate to avoid flooding UART/CPU. */
#define APP_ENC_DUMP_ENABLE 0
#define APP_ENC_DUMP_PERIOD_MS 1000U
#define APP_ENC_DUMP_BYTES 16
#define APP_ENC_DUMP_SUM_BYTES 64

/* Master switch for any UVC-prefixed log print. */
#define APP_UVC_LOG_ENABLE 0
/* Keep callback-level stream logs even when APP_UVC_LOG_ENABLE is off. */
#define APP_UVC_CB_LOG_ENABLE 1

/* UVC TX-path debug (after encode, before/after USB submit). */
#define APP_UVC_TX_DEBUG_ENABLE 0
#define APP_UVC_TX_DEBUG_PERIOD_MS 1000U
#define APP_UVC_TX_SIG_BYTES 64
#define APP_UVC_CLEAN_DCACHE_BEFORE_SHOW 1

static int is_cache_enable(void);

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
#define VENC_MAX_WIDTH 1280
#define VENC_MAX_HEIGHT 720
#define VENC_OUT_BUFFER_SIZE (255 * 1024)

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
static uint32_t g_pipe1_dump_last_ms;
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
static uint32_t g_uvc_tx_last_log_ms;
static uint32_t g_uvc_tx_submit_id;
static uint32_t g_uvc_tx_release_id;
static uint32_t g_uvc_tx_last_submit_len;
static uint32_t g_uvc_tx_last_submit_sig;
static uint32_t g_uvc_tx_last_release_sig;
static uint32_t g_uvc_tx_ptr_mismatch_count;
static uint32_t g_uvc_tx_sig_mismatch_count;
static uintptr_t g_uvc_tx_last_release_ptr;
static uint32_t g_uvc_stream_session_id;
static uint32_t g_uvc_stream_start_tick_ms;
static int uvc_is_active;
static volatile int buffer_flying;

/* Forward declarations: used by early debug dump helper before full definitions below. */
static uint8_t capture_buffer[CAPTURE_BUFFER_NB][VENC_MAX_WIDTH * VENC_MAX_HEIGHT * CAPTURE_BPP];
static int capture_buffer_disp_idx;

static void app_dump_pipe1_frame_data_1s(void)
{
#if APP_PIPE1_FRAME_DUMP_ENABLE
  uint8_t *src;
  uintptr_t src_addr;
  uintptr_t inval_addr;
  uint32_t inval_len;
  uint32_t now_ms;
  uint32_t sum;
  int idx;
  int i;
  uint8_t sample[APP_PIPE1_FRAME_DUMP_BYTES];

  if (g_pipe1_dump_pending == 0)
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - g_pipe1_dump_last_ms) < APP_PIPE1_FRAME_DUMP_PERIOD_MS)
  {
    return;
  }

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
  }

  printf("[CAM][DATA] p1 idx=%d addr=0x%08lX b0..b7=%02X %02X %02X %02X %02X %02X %02X %02X sum%u=%lu\r\n",
         idx,
         (unsigned long)capture_buffer[idx],
         sample[0], sample[1], sample[2], sample[3],
         sample[4], sample[5], sample[6], sample[7],
         APP_PIPE1_FRAME_DUMP_BYTES,
         (unsigned long)sum);

  g_pipe1_dump_last_ms = now_ms;
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

static int app_h264_find_start_code(const uint8_t *bs, int len, int *sc_off, int *sc_len)
{
  int i;
  int lim;

  if ((bs == NULL) || (len < 4))
  {
    return 0;
  }

  lim = (len > 64) ? 64 : len;
  for (i = 0; i <= (lim - 4); i++)
  {
    if ((bs[i] == 0x00U) && (bs[i + 1] == 0x00U))
    {
      if (bs[i + 2] == 0x01U)
      {
        *sc_off = i;
        *sc_len = 3;
        return 1;
      }
      if ((bs[i + 2] == 0x00U) && (bs[i + 3] == 0x01U))
      {
        *sc_off = i;
        *sc_len = 4;
        return 1;
      }
    }
  }

  return 0;
}

static void app_log_encoded_data_1s(const uint8_t *bs, int len)
{
#if APP_ENC_DUMP_ENABLE
  uint32_t now_ms;
  uint32_t sum = 0U;
  uint8_t sample[8] = {0};
  int sc_off = -1;
  int sc_len = 0;
  int nal_type = -1;
  int sum_bytes;
  int i;
  int valid = 0;

  if ((bs != NULL) && (len > 0) && (len <= (int)VENC_OUT_BUFFER_SIZE))
  {
    if (app_h264_find_start_code(bs, len, &sc_off, &sc_len))
    {
      int nal_idx = sc_off + sc_len;
      if (nal_idx < len)
      {
        nal_type = (int)(bs[nal_idx] & 0x1FU);
        /* Valid single-byte NAL header type for H.264 stream payload */
        valid = (nal_type >= 1) && (nal_type <= 12);
      }
    }
  }

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

  printf("[ENC][H264] size=%d sc=%d/%d nal=%d b0..b7=%02X %02X %02X %02X %02X %02X %02X %02X sum%u=%lu ok=+%lu bad=+%lu tot=(%lu,%lu)\r\n",
         len,
         sc_off,
         sc_len,
         nal_type,
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
      printf("[CAM][ERR] DCMIPP pipe=%lu state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX p1fctcr=0x%08lX ovrf=%lu ovrie=%lu ovrerr=%lu recov=%lu\r\n",
             (unsigned long)g_pipe_error_last_pipe,
             (unsigned long)g_pipe_error_last_state,
             (unsigned long)g_pipe_error_last_code,
             (unsigned long)g_pipe_error_last_cmsr2,
             (unsigned long)g_pipe_error_last_cmier,
             (unsigned long)g_pipe_error_last_p1fctcr,
             (unsigned long)p1_ovr_flag,
             (unsigned long)p1_ovr_irq_en,
             (unsigned long)p1_ovr_err,
             (unsigned long)g_pipe_error_soft_recover_count);
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

/* model */
static uint8_t network_ctx[STAI_NETWORK_CONTEXT_SIZE] ALIGN_32;
 /* nn input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;
 /* nn output buffers */
static uint8_t nn_output_buffers[2][NN_BUFFER_OUT_SIZE_ALIGN] ALIGN_32;
static bqueue_t nn_output_queue;

/* venc */
static uint8_t venc_out_buffer[VENC_OUT_BUFFER_SIZE] ALIGN_32 UNCACHED;
static uint8_t uvc_in_buffers[VENC_OUT_BUFFER_SIZE] ALIGN_32;

/* uvc */
static struct uvcl_callbacks uvcl_cbs;
static int uvc_is_active;
static volatile int buffer_flying;
static int force_intra;

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

  /* USB has not released previous frame yet: skip encode to avoid extra PSRAM/AXI pressure. */
  if (buffer_flying)
  {
    g_uvc_drop_busy_count++;
    g_uvc_last_enc_ret = -2;
    force_intra = 1;
    return (size_t)-1;
  }

  res = ENC_EncodeFrame(p_buffer, venc_out_buffer, VENC_OUT_BUFFER_SIZE, is_intra_force);
  enc_ret = (int)res;

  /* encoder failed certainly due to output buffer too small */
  if (enc_ret <= 0)
  {
    g_uvc_enc_fail_count++;
    g_uvc_last_enc_ret = enc_ret;
    return res;
  }

  g_uvc_enc_ok_count++;
  g_uvc_last_enc_ret = enc_ret;
  app_log_encoded_data_1s(venc_out_buffer, enc_ret);

  memcpy(&uvc_in_buffers, venc_out_buffer, res);

  return res;
}

static int send_display(int len)
{
  int ret;
  uint32_t sig;
#if APP_UVC_TX_DEBUG_ENABLE
  uint32_t now_ms;
#endif

  g_uvc_show_req_count++;
  g_uvc_tx_submit_id++;
  g_uvc_tx_last_submit_len = (uint32_t)len;
  sig = app_sum_prefix_u32(uvc_in_buffers, len, APP_UVC_TX_SIG_BYTES);
  g_uvc_tx_last_submit_sig = sig;
  app_uvc_clean_dcache_for_show(uvc_in_buffers, len);
#if APP_UVC_TX_DEBUG_ENABLE
  now_ms = HAL_GetTick();
  if ((now_ms - g_uvc_tx_last_log_ms) >= APP_UVC_TX_DEBUG_PERIOD_MS)
  {
    printf("[UVC][TX] submit id=%lu len=%d ptr=0x%08lX sig%u=%lu clean=%d\r\n",
           (unsigned long)g_uvc_tx_submit_id,
           len,
           (unsigned long)uvc_in_buffers,
           APP_UVC_TX_SIG_BYTES,
           (unsigned long)g_uvc_tx_last_submit_sig,
           APP_UVC_CLEAN_DCACHE_BEFORE_SHOW);
    g_uvc_tx_last_log_ms = now_ms;
  }
#endif
  buffer_flying = 1;
  ret = UVCL_ShowFrame(uvc_in_buffers, len);
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
  static uint32_t last_seq;
  int idx;
  int len;
  uint32_t seq;

  if (!uvc_is_active)
  {
    return;
  }

  seq = g_pipe1_completed_seq;
  if (seq == last_seq)
  {
    return;
  }
  last_seq = seq;

  idx = g_pipe1_last_completed_idx;
  if ((idx < 0) || (idx >= CAPTURE_BUFFER_NB))
  {
    return;
  }

  g_uvc_bringup_submit_count++;
  g_uvc_last_submit_idx = idx;

  len = (int)encode_display(force_intra, capture_buffer[idx]);
  if (len > 0)
  {
    (void)send_display(len);
    force_intra = 0;
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
    send_display(len);
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
  int ret;

  while (1) {
    ret = xSemaphoreTake(isp_sem, pdMS_TO_TICKS(1000));
    if (ret == pdTRUE)
    {
      CAM_IspUpdate();
    }

#if APP_DVP_BRINGUP_PIPE1_ONLY
    app_uvc_try_send_pipe1_bringup();
#endif
    app_flush_cam_irq_logs();
    app_log_cam_health_1s();
    app_log_dcmipp_error_poll_1s();
    app_log_uvc_health_1s();
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
  BSP_LED_On(LED_RED);
#if APP_UVC_CB_LOG_ENABLE
  printf("[UVC][START] session=%lu tick_ms=%lu cfg=%dx%d@%d payload=%d on=%lu off=%lu\r\n",
         (unsigned long)g_uvc_stream_session_id,
         (unsigned long)now_ms,
         stream.width, stream.height, stream.fps, stream.payload_type,
         (unsigned long)g_uvc_stream_active_count,
         (unsigned long)g_uvc_stream_inactive_count);
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
    printf("[CAM][ERR][SNAP][UVC-START] p1_state=%lu err=0x%08lX cmsr2=0x%08lX cmier=0x%08lX p1_ovrf=%lu p1_ovrie=%lu p1_ovrerr=%lu\r\n",
           (unsigned long)p1_state,
           (unsigned long)err,
           (unsigned long)cmsr2,
           (unsigned long)cmier,
           (unsigned long)(((cmsr2 & DCMIPP_FLAG_PIPE1_OVR) != 0U) ? 1U : 0U),
           (unsigned long)(((cmier & DCMIPP_IT_PIPE1_OVR) != 0U) ? 1U : 0U),
           (unsigned long)(((err & HAL_DCMIPP_ERROR_PIPE1_OVR) != 0U) ? 1U : 0U));
  }
}

static void app_uvc_streaming_inactive(struct uvcl_callbacks *cbs)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t dur_ms = now_ms - g_uvc_stream_start_tick_ms;

  (void)cbs;
  uvc_is_active = 0;
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

static void app_display_info_header()
{
  printf("========================================\n\r");
  printf("x-cube-n6-ai-h264-usb-uvc v2.2.0--- hinoeng app 09:13 (%s)\n\r", APP_VERSION_STRING);
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

void app_run()
{
  UBaseType_t isp_priority = FREERTOS_PRIORITY(2);
#if !APP_DVP_BRINGUP_PIPE1_ONLY
  UBaseType_t dp_priority = FREERTOS_PRIORITY(-2);
  UBaseType_t nn_priority = FREERTOS_PRIORITY(1);
#endif
  UVCL_Conf_t uvcl_conf = { 0 };
  ENC_Conf_t enc_conf = { 0 };
  TaskHandle_t hdl;
  int ret;

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

  /* Encoder init */
  enc_conf.width = VENC_WIDTH;
  enc_conf.height = VENC_HEIGHT;
  enc_conf.fps = CAMERA_FPS;
#if (CAPTURE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
  enc_conf.input_type = ENC_INPUT_YUV422_YUYV;
#elif (CAPTURE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1_UYVY)
  enc_conf.input_type = ENC_INPUT_YUV422_UYVY;
#else
  enc_conf.input_type = ENC_INPUT_RGB888;
#endif
  printf("[APP] enc cfg %dx%d@%d input_type=%d cap_fmt=%d bpp=%d\r\n",
         enc_conf.width,
         enc_conf.height,
         enc_conf.fps,
         (int)enc_conf.input_type,
         (int)CAPTURE_FORMAT,
         CAPTURE_BPP);
  ENC_Init(&enc_conf);

  /* Uvc init */
  uvcl_conf.streams[0].width = VENC_WIDTH;
  uvcl_conf.streams[0].height = VENC_HEIGHT;
  uvcl_conf.streams[0].fps = CAMERA_FPS;
  uvcl_conf.streams[0].payload_type = UVCL_PAYLOAD_FB_H264;
  uvcl_conf.streams_nb = 1;
  uvcl_conf.is_immediate_mode = 1;
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
  CAM_DisplayPipe_Start(capture_buffer[0], CMW_MODE_CONTINUOUS);

  /* threads init */
#if APP_DVP_BRINGUP_PIPE1_ONLY
  printf("[CAM] bring-up mode enabled: NN/PIPE2 and DP threads are skipped\r\n");
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
