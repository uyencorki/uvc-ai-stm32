 /**
 ******************************************************************************
 * @file    main.c
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

#include "app.h"
#include "app_config.h"
#include "app_fuseprogramming.h"
#include "main.h"
#include "npu_cache.h"
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_bus.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_xspi.h"
#include <stdio.h>
#include <stdint.h>
#include "stm32n6xx_hal_rif.h"
#include "FreeRTOS.h"
#include "task.h"

extern int __uncached_bss_start__;
extern int __uncached_bss_end__;

UART_HandleTypeDef huart1;

int g_psram_init_ret = BSP_ERROR_NO_INIT;
int g_psram_readid_ret = BSP_ERROR_NO_INIT;
int g_psram_write_ret = BSP_ERROR_NO_INIT;
int g_psram_read_ret = BSP_ERROR_NO_INIT;
int g_psram_verify_mismatch = -1;
int g_psram_mmp_ret = BSP_ERROR_NO_INIT;
uint8_t g_psram_id[6];
uint8_t g_psram_probe_rd8[8];
uint32_t g_psram_fullscan_bytes = 0U;
uint32_t g_psram_fullscan_fail_chunks = 0U;
uint32_t g_psram_fullscan_mismatch_bytes = 0U;

static StaticTask_t main_thread;
static StackType_t main_thread_stack[configMINIMAL_STACK_SIZE];

#ifndef PSRAM_FULL_SCAN_ENABLE
#define PSRAM_FULL_SCAN_ENABLE 0U
#endif

#ifndef PSRAM_FULL_SCAN_CHUNK_BYTES
#define PSRAM_FULL_SCAN_CHUNK_BYTES 4096U
#endif

#ifndef PSRAM_FULL_SCAN_PROGRESS_STEP
#define PSRAM_FULL_SCAN_PROGRESS_STEP (4U * 1024U * 1024U)
#endif

#ifndef PSRAM_WINDOW_ALIAS_TEST_ENABLE
#define PSRAM_WINDOW_ALIAS_TEST_ENABLE 0U
#endif

#ifndef PSRAM_MMP_READ_SCAN_ENABLE
#define PSRAM_MMP_READ_SCAN_ENABLE 0U
#endif

#ifndef PSRAM_QUICK_PROBE_ENABLE
/* 0: hide small probe logs, 1: keep quick INC/ALT/WALK probe logs */
#define PSRAM_QUICK_PROBE_ENABLE 0U
#endif

#ifndef PSRAM_SCAN_INFO_LOG_ENABLE
/* 0: only scan errors are printed, 1: print start/progress/summary */
#define PSRAM_SCAN_INFO_LOG_ENABLE 0U
#endif

#ifndef PSRAM_W958_YUV_TEST_ENABLE
/* 1: run YUV422 write/read verify right after MMP enable */
#define PSRAM_W958_YUV_TEST_ENABLE 1U
#endif

static uint8_t g_psram_full_wr[PSRAM_FULL_SCAN_CHUNK_BYTES];
static uint8_t g_psram_full_rd[PSRAM_FULL_SCAN_CHUNK_BYTES];

static void SystemClock_Config(void);
static void NPURam_enable();
static void NPUCache_config();
static void Security_Config();
static void IAC_Config();
static void CONSOLE_Config(void);
static void Setup_Mpu(void);
static void PSRAM_DebugProbe(void);
static void PSRAM_MMP_DebugProbe(void);
#if (PSRAM_WINDOW_ALIAS_TEST_ENABLE == 1U)
static void PSRAM_AliasDebugProbe(void);
#endif
static void PSRAM_FullScanDebugProbe(void);
static void PSRAM_MMP_ReadScanDebugProbe(uint32_t expected_pattern_id);
static void PSRAM_FillPattern(uint8_t *dst, uint32_t len, uint32_t pattern_id);
static int PSRAM_CompareAndLog(const char *tag, const uint8_t *wr, const uint8_t *rd, uint32_t len);
static int main_freertos(void);
static void main_thread_fct(void *arg);

/* This is defined in port.c */
void vPortSetupTimerInterrupt(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  CONSOLE_Config();

  printf ("========================================== \r\n");
  printf ("------ run app --------- [%s %s]\r\n", __DATE__, __TIME__);

  /* Power on ICACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_ICACTIVE_Msk;

  /* Set back system and CPU clock source to HSI */
  __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

  HAL_Init();

  CONSOLE_Config();


  BOARD_Pins_Init_DCMIPP();

  Setup_Mpu();

  SCB_EnableICache();

#if defined(USE_DCACHE)
  /* Power on DCACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
  SCB_EnableDCache();
#endif

  return main_freertos();
}

static void NPURam_enable()
{
  __HAL_RCC_NPU_CLK_ENABLE();
  __HAL_RCC_NPU_FORCE_RESET();
  __HAL_RCC_NPU_RELEASE_RESET();

  /* Enable NPU RAMs (4x448KB) */
  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
  __HAL_RCC_RAMCFG_CLK_ENABLE();
  RAMCFG_HandleTypeDef hramcfg = {0};
  hramcfg.Instance =  RAMCFG_SRAM3_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM4_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM5_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM6_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
}

static void Setup_Mpu()
{
  MPU_Attributes_InitTypeDef attr;
  MPU_Region_InitTypeDef region;

  attr.Number = MPU_ATTRIBUTES_NUMBER0;
  attr.Attributes = MPU_NOT_CACHEABLE;
  HAL_MPU_ConfigMemoryAttributes(&attr);

  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER0;
  region.BaseAddress = (uint32_t)&__uncached_bss_start__;
  region.LimitAddress = (uint32_t)&__uncached_bss_end__ - 1;
  region.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  region.AccessPermission = MPU_REGION_ALL_RW;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  region.DisablePrivExec = MPU_PRIV_INSTRUCTION_ACCESS_ENABLE;
  region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  HAL_MPU_ConfigRegion(&region);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  memset(&__uncached_bss_start__, 0, &__uncached_bss_end__ - &__uncached_bss_start__);
}

static void NPUCache_config()
{
  npu_cache_enable();
}

static void Security_Config()
{
  __HAL_RCC_RIFSC_CLK_ENABLE();
  RIMC_MasterConfig_t RIMC_master = {0};
  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1 , &RIMC_master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_OTG1HS , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

static void IAC_Config(void)
{
/* Configure IAC to trap illegal access events */
  __HAL_RCC_IAC_CLK_ENABLE();
  __HAL_RCC_IAC_FORCE_RESET();
  __HAL_RCC_IAC_RELEASE_RESET();
}

void IAC_IRQHandler(void)
{
  while (1)
  {
  }
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};

  BSP_SMPS_Init(SMPS_VOLTAGE_OVERDRIVE);
  HAL_Delay(1); /* Assuming Voltage Ramp Speed of 1mV/us --> 100mV increase takes 100us */

  // Oscillator config already done in bootrom
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;

  /* PLL1 = 64 x 25 / 2 = 800MHz */
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 2;
  RCC_OscInitStruct.PLL1.PLLN = 25;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;

  /* PLL2 = 64 x 125 / 8 = 1000MHz */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL2.PLLM = 8;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLN = 125;
  RCC_OscInitStruct.PLL2.PLLP1 = 1;
  RCC_OscInitStruct.PLL2.PLLP2 = 1;

  /* PLL3 = (64 x 225 / 8) / (1 * 2) = 900MHz */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 8;
  RCC_OscInitStruct.PLL3.PLLN = 225;
  RCC_OscInitStruct.PLL3.PLLFractional = 0;
  RCC_OscInitStruct.PLL3.PLLP1 = 1;
  RCC_OscInitStruct.PLL3.PLLP2 = 2;

  /* PLL4 in bypass mode.
   * Keep PLL4 output active to allow IC17 and IC18 clock source switch from PLL4 (reset value) to PLL2 and PLL1 (see
   * RM0486 Clock switches and gating).
   */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_BYPASS;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4 |
                                 RCC_CLOCKTYPE_PCLK5);

  /* CPU CLock (sysa_ck) = ic1_ck = PLL1 output/ic1_divider = 800 MHz */
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 1;

  /* AXI Clock (sysb_ck) = ic2_ck = PLL1 output/ic2_divider = 400 MHz */
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 2;

  /* NPU Clock (sysc_ck) = ic6_ck = PLL2 output/ic6_divider = 1000 MHz */
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL2;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 1;

  /* AXISRAM3/4/5/6 Clock (sysd_ck) = ic11_ck = PLL3 output/ic11_divider = 900 MHz */
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 1;

  /* HCLK = sysb_ck / HCLK divider = 200 MHz */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

  /* PCLKx = HCLK / PCLKx divider = 200 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    while(1);
  }

  RCC_PeriphCLKInitStruct.PeriphClockSelection = 0;

  /* XSPI1 kernel clock (ck_ker_xspi1) = HCLK = 200MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI1;
  RCC_PeriphCLKInitStruct.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_HCLK;

  /* XSPI2 kernel clock (ck_ker_xspi1) = HCLK =  200MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI2;
  RCC_PeriphCLKInitStruct.Xspi2ClockSelection = RCC_XSPI2CLKSOURCE_HCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    while (1);
  }
}

static void CONSOLE_Config()
{
  GPIO_InitTypeDef gpio_init;

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

 /* DISCO & NUCLEO USART1 (PE5/PE6) */
  gpio_init.Mode      = GPIO_MODE_AF_PP;
  gpio_init.Pull      = GPIO_PULLUP;
  gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOE, &gpio_init);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    while (1);
  }
}

static void DMA2D_Config()
{
  __HAL_RCC_DMA2D_CLK_ENABLE();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();
}

static void PSRAM_FillPattern(uint8_t *dst, uint32_t len, uint32_t pattern_id)
{
  uint32_t i;

  if (dst == NULL)
  {
    return;
  }

  for (i = 0; i < len; i++)
  {
    switch (pattern_id)
    {
      case 0U: /* Incremental byte pattern: A0 A1 ... */
        dst[i] = (uint8_t)(0xA0U + (uint8_t)i);
        break;
      case 1U: /* Alternating checkerboard bytes */
        dst[i] = ((i & 1U) == 0U) ? 0x55U : 0xAAU;
        break;
      default: /* Walking bit */
        dst[i] = (uint8_t)(1U << (i & 7U));
        break;
    }
  }
}

static int PSRAM_CompareAndLog(const char *tag, const uint8_t *wr, const uint8_t *rd, uint32_t len)
{
  uint32_t i;
  int mismatch = 0;
  int first_idx = -1;
  uint8_t first_wr = 0U;
  uint8_t first_rd = 0U;

  if ((tag == NULL) || (wr == NULL) || (rd == NULL))
  {
    return (int)len;
  }

  for (i = 0; i < len; i++)
  {
    if (wr[i] != rd[i])
    {
      if (first_idx < 0)
      {
        first_idx = (int)i;
        first_wr = wr[i];
        first_rd = rd[i];
      }
      mismatch++;
    }
  }

  printf("[PSRAM][%s] wr=%02X %02X %02X %02X %02X %02X %02X %02X rd=%02X %02X %02X %02X %02X %02X %02X %02X mismatch=%d\r\n",
         tag,
         wr[0], wr[1], wr[2], wr[3], wr[4], wr[5], wr[6], wr[7],
         rd[0], rd[1], rd[2], rd[3], rd[4], rd[5], rd[6], rd[7],
         mismatch);

  if (first_idx >= 0)
  {
    printf("[PSRAM][%s] first_mismatch i=%d wr=%02X rd=%02X xor=%02X\r\n",
           tag, first_idx, first_wr, first_rd, (uint8_t)(first_wr ^ first_rd));
  }

  return mismatch;
}

static void PSRAM_FillPatternWithOffset(uint8_t *dst, uint32_t len, uint32_t pattern_id, uint32_t base_off)
{
  uint32_t i;

  for (i = 0; i < len; i++)
  {
    uint32_t x = base_off + i;
    switch (pattern_id)
    {
      case 0U:
        dst[i] = (uint8_t)(x & 0xFFU);
        break;
      case 1U:
        dst[i] = ((x & 1U) == 0U) ? 0x55U : 0xAAU;
        break;
      default:
        dst[i] = (uint8_t)(1U << (x & 7U));
        break;
    }
  }
}

static int PSRAM_CompareChunk(const uint8_t *wr, const uint8_t *rd, uint32_t len,
                              uint32_t *first_idx, uint8_t *exp, uint8_t *got)
{
  uint32_t i;
  int mismatch = 0;
  uint32_t idx = 0U;
  uint8_t e = 0U;
  uint8_t g = 0U;
  int captured = 0;

  for (i = 0; i < len; i++)
  {
    if (wr[i] != rd[i])
    {
      mismatch++;
      if (captured == 0)
      {
        idx = i;
        e = wr[i];
        g = rd[i];
        captured = 1;
      }
    }
  }

  if (first_idx != NULL)
  {
    *first_idx = idx;
  }
  if (exp != NULL)
  {
    *exp = e;
  }
  if (got != NULL)
  {
    *got = g;
  }

  return mismatch;
}

static void PSRAM_FullScanDebugProbe(void)
{
  static const char *pattern_name[3] = {"FULL-INC", "FULL-ALT55AA", "FULL-WALK1"};
  static const uint32_t max_fail_logs = 16U;
  uint32_t p;
  uint32_t off;
  uint32_t fail_logs = 0U;
  uint32_t fail_chunks = 0U;
  uint32_t mismatch_bytes = 0U;

#if (PSRAM_SCAN_INFO_LOG_ENABLE == 1U)
  printf("[PSRAM][FULL] begin size=%lu chunk=%lu patterns=3 base=0x%08lX\r\n",
         (unsigned long)BSP_XSPI_RAM_SIZE_BYTES,
         (unsigned long)PSRAM_FULL_SCAN_CHUNK_BYTES,
         (unsigned long)BSP_XSPI_RAM_MMP_BASE);
#endif

  for (p = 0U; p < 3U; p++)
  {
    uint32_t progress_mark = 0U;
#if (PSRAM_SCAN_INFO_LOG_ENABLE == 1U)
    printf("[PSRAM][FULL] pattern=%s start\r\n", pattern_name[p]);
#endif

    for (off = 0U; off < BSP_XSPI_RAM_SIZE_BYTES; off += PSRAM_FULL_SCAN_CHUNK_BYTES)
    {
      uint32_t chunk = BSP_XSPI_RAM_SIZE_BYTES - off;
      int32_t ret;
      int mismatch;
      uint32_t first_idx = 0U;
      uint8_t exp = 0U;
      uint8_t got = 0U;

      if (chunk > PSRAM_FULL_SCAN_CHUNK_BYTES)
      {
        chunk = PSRAM_FULL_SCAN_CHUNK_BYTES;
      }

      PSRAM_FillPatternWithOffset(g_psram_full_wr, chunk, p, off);
      memset(g_psram_full_rd, 0, chunk);

      ret = BSP_XSPI_RAM_Write(0U, g_psram_full_wr, off, chunk);
      if (ret != BSP_ERROR_NONE)
      {
        fail_chunks++;
        mismatch_bytes += chunk;
        if (fail_logs < max_fail_logs)
        {
          printf("[PSRAM][FULL][ERR] write off=0x%08lX len=%lu ret=%ld\r\n",
                 (unsigned long)off, (unsigned long)chunk, (long)ret);
          fail_logs++;
        }
        continue;
      }

      ret = BSP_XSPI_RAM_Read(0U, g_psram_full_rd, off, chunk);
      if (ret != BSP_ERROR_NONE)
      {
        fail_chunks++;
        mismatch_bytes += chunk;
        if (fail_logs < max_fail_logs)
        {
          printf("[PSRAM][FULL][ERR] read off=0x%08lX len=%lu ret=%ld\r\n",
                 (unsigned long)off, (unsigned long)chunk, (long)ret);
          fail_logs++;
        }
        continue;
      }

      mismatch = PSRAM_CompareChunk(g_psram_full_wr, g_psram_full_rd, chunk, &first_idx, &exp, &got);
      if (mismatch != 0)
      {
        fail_chunks++;
        mismatch_bytes += (uint32_t)mismatch;
        if (fail_logs < max_fail_logs)
        {
          printf("[PSRAM][FULL][ERR] cmp off=0x%08lX len=%lu mismatch=%d first_i=%lu exp=%02X got=%02X\r\n",
                 (unsigned long)off,
                 (unsigned long)chunk,
                 mismatch,
                 (unsigned long)first_idx,
                 exp,
                 got);
          fail_logs++;
        }
      }

      if ((PSRAM_SCAN_INFO_LOG_ENABLE == 1U) && ((off - progress_mark) >= PSRAM_FULL_SCAN_PROGRESS_STEP))
      {
        progress_mark = off;
        printf("[PSRAM][FULL] pattern=%s progress=%lu/%luMB\r\n",
               pattern_name[p],
               (unsigned long)(off / (1024U * 1024U)),
               (unsigned long)(BSP_XSPI_RAM_SIZE_BYTES / (1024U * 1024U)));
      }
    }

#if (PSRAM_SCAN_INFO_LOG_ENABLE == 1U)
    printf("[PSRAM][FULL] pattern=%s done fail_chunks=%lu mismatch_bytes=%lu\r\n",
           pattern_name[p],
           (unsigned long)fail_chunks,
           (unsigned long)mismatch_bytes);
#endif
  }

  g_psram_fullscan_bytes = BSP_XSPI_RAM_SIZE_BYTES;
  g_psram_fullscan_fail_chunks = fail_chunks;
  g_psram_fullscan_mismatch_bytes = mismatch_bytes;

  if ((g_psram_fullscan_fail_chunks != 0U) || (g_psram_fullscan_mismatch_bytes != 0U) ||
      (PSRAM_SCAN_INFO_LOG_ENABLE == 1U))
  {
    printf("[PSRAM][FULL] summary size=%lu fail_chunks=%lu mismatch_bytes=%lu\r\n",
           (unsigned long)g_psram_fullscan_bytes,
           (unsigned long)g_psram_fullscan_fail_chunks,
           (unsigned long)g_psram_fullscan_mismatch_bytes);
  }
}

static void PSRAM_MMP_ReadScanDebugProbe(uint32_t expected_pattern_id)
{
  static const uint32_t max_fail_logs = 16U;
  uint32_t off;
  uint32_t fail_chunks = 0U;
  uint32_t mismatch_bytes = 0U;
  uint32_t fail_logs = 0U;

#if (PSRAM_SCAN_INFO_LOG_ENABLE == 1U)
  printf("[PSRAM][MMPFULL] begin size=%lu chunk=%lu expected_pattern=%lu base=0x%08lX\r\n",
         (unsigned long)BSP_XSPI_RAM_SIZE_BYTES,
         (unsigned long)PSRAM_FULL_SCAN_CHUNK_BYTES,
         (unsigned long)expected_pattern_id,
         (unsigned long)BSP_XSPI_RAM_MMP_BASE);
#endif

  for (off = 0U; off < BSP_XSPI_RAM_SIZE_BYTES; off += PSRAM_FULL_SCAN_CHUNK_BYTES)
  {
    uint32_t chunk = BSP_XSPI_RAM_SIZE_BYTES - off;
    volatile uint8_t *mmp = (volatile uint8_t *)(BSP_XSPI_RAM_MMP_BASE + off);
    uintptr_t cache_addr;
    int32_t cache_len;
    int mismatch;
    uint32_t first_idx = 0U;
    uint8_t exp = 0U;
    uint8_t got = 0U;
    uint32_t i;

    if (chunk > PSRAM_FULL_SCAN_CHUNK_BYTES)
    {
      chunk = PSRAM_FULL_SCAN_CHUNK_BYTES;
    }

    cache_addr = ((uintptr_t)mmp) & ~(uintptr_t)31U;
    cache_len = (int32_t)(((((uintptr_t)mmp - cache_addr) + chunk) + 31U) & ~(uintptr_t)31U);

#if defined(USE_DCACHE)
    SCB_InvalidateDCache_by_Addr((uint32_t *)cache_addr, cache_len);
    __DSB();
    __ISB();
#endif

    for (i = 0U; i < chunk; i++)
    {
      g_psram_full_rd[i] = mmp[i];
    }

    PSRAM_FillPatternWithOffset(g_psram_full_wr, chunk, expected_pattern_id, off);
    mismatch = PSRAM_CompareChunk(g_psram_full_wr, g_psram_full_rd, chunk, &first_idx, &exp, &got);

    if (mismatch != 0)
    {
      fail_chunks++;
      mismatch_bytes += (uint32_t)mismatch;
      if (fail_logs < max_fail_logs)
      {
        printf("[PSRAM][MMPFULL][ERR] off=0x%08lX len=%lu mismatch=%d first_i=%lu exp=%02X got=%02X\r\n",
               (unsigned long)off,
               (unsigned long)chunk,
               mismatch,
               (unsigned long)first_idx,
               exp,
               got);
        fail_logs++;
      }
    }
  }

  if ((fail_chunks != 0U) || (mismatch_bytes != 0U) || (PSRAM_SCAN_INFO_LOG_ENABLE == 1U))
  {
    printf("[PSRAM][MMPFULL] summary fail_chunks=%lu mismatch_bytes=%lu\r\n",
           (unsigned long)fail_chunks,
           (unsigned long)mismatch_bytes);
  }
}

static void PSRAM_DebugProbe(void)
{
  static const char *pattern_name[3] = {"INC", "ALT55AA", "WALK1"};
  static const uint32_t probe_addr = 0x00100000U;
  uint8_t wr[16];
  uint8_t rd[16] = {0};
  int32_t ret;
  int mismatch_total = 0;
  int mismatch;
  uint32_t p;
  int i;

  ret = BSP_XSPI_RAM_ReadID(0, g_psram_id);
  g_psram_readid_ret = (int)ret;
  printf("[PSRAM] ReadID ret=%ld id=%02X %02X %02X %02X %02X %02X\r\n",
         (long)ret, g_psram_id[0], g_psram_id[1], g_psram_id[2], g_psram_id[3], g_psram_id[4], g_psram_id[5]);

  for (p = 0U; p < 3U; p++)
  {
    PSRAM_FillPattern(wr, sizeof(wr), p);
    memset(rd, 0, sizeof(rd));

    ret = BSP_XSPI_RAM_Write(0, wr, probe_addr, sizeof(wr));
    if (p == 0U)
    {
      g_psram_write_ret = (int)ret;
    }
    printf("[PSRAM][%s] Write ret=%ld addr=0x%08lX len=%u\r\n",
           pattern_name[p], (long)ret, (unsigned long)probe_addr, (unsigned)sizeof(wr));
    if (ret != BSP_ERROR_NONE)
    {
      mismatch_total += (int)sizeof(wr);
      continue;
    }

    ret = BSP_XSPI_RAM_Read(0, rd, probe_addr, sizeof(rd));
    if (p == 0U)
    {
      g_psram_read_ret = (int)ret;
    }
    printf("[PSRAM][%s] Read ret=%ld addr=0x%08lX len=%u\r\n",
           pattern_name[p], (long)ret, (unsigned long)probe_addr, (unsigned)sizeof(rd));
    if (ret != BSP_ERROR_NONE)
    {
      mismatch_total += (int)sizeof(rd);
      continue;
    }

    if (p == 0U)
    {
      for (i = 0; i < 8; i++)
      {
        g_psram_probe_rd8[i] = rd[i];
      }
    }

    mismatch = PSRAM_CompareAndLog(pattern_name[p], wr, rd, sizeof(wr));
    mismatch_total += mismatch;
  }

  g_psram_verify_mismatch = mismatch_total;
  printf("[PSRAM] verify mismatch(total)=%d\r\n", mismatch_total);
}

static void PSRAM_MMP_DebugProbe(void)
{
  static const char *pattern_name[3] = {"MMP-INC", "MMP-ALT55AA", "MMP-WALK1"};
  static const uint32_t probe_addr = 0x00100000U;
  volatile uint8_t *mmp = (volatile uint8_t *)(XSPI1_BASE + probe_addr);
  uint8_t wr[16];
  uint8_t rd[16];
  uint8_t warm[8];
  char retry_tag[24];
  uintptr_t cache_addr;
  int32_t cache_len;
  int mismatch_total = 0;
  int mismatch;
  uint32_t p;
  int i;

  cache_addr = ((uintptr_t)mmp) & ~(uintptr_t)31U;
  cache_len = 32;

  printf("[PSRAM][MMP] base=0x%08lX probe=0x%08lX ptr=0x%08lX\r\n",
         (unsigned long)XSPI1_BASE,
         (unsigned long)probe_addr,
         (unsigned long)(XSPI1_BASE + probe_addr));

#if defined(USE_DCACHE)
  SCB_InvalidateDCache_by_Addr((uint32_t *)cache_addr, cache_len);
  __DSB();
  __ISB();
#endif
  for (i = 0; i < (int)sizeof(warm); i++)
  {
    warm[i] = mmp[i];
  }
  printf("[PSRAM][MMP] warmup rd=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
         warm[0], warm[1], warm[2], warm[3], warm[4], warm[5], warm[6], warm[7]);

  for (p = 0U; p < 3U; p++)
  {
    PSRAM_FillPattern(wr, sizeof(wr), p);

    for (i = 0; i < (int)sizeof(wr); i++)
    {
      mmp[i] = wr[i];
    }

#if defined(USE_DCACHE)
    /* Ensure writeback reaches XSPI memory and next read fetches fresh data. */
    SCB_CleanDCache_by_Addr((uint32_t *)cache_addr, cache_len);
    __DSB();
    SCB_InvalidateDCache_by_Addr((uint32_t *)cache_addr, cache_len);
    __DSB();
    __ISB();
#endif

    for (i = 0; i < (int)sizeof(rd); i++)
    {
      rd[i] = mmp[i];
    }

    mismatch = PSRAM_CompareAndLog(pattern_name[p], wr, rd, sizeof(wr));
    if (mismatch != 0)
    {
#if defined(USE_DCACHE)
      SCB_InvalidateDCache_by_Addr((uint32_t *)cache_addr, cache_len);
      __DSB();
      __ISB();
#endif
      for (i = 0; i < (int)sizeof(rd); i++)
      {
        rd[i] = mmp[i];
      }
      (void)snprintf(retry_tag, sizeof(retry_tag), "%s-R", pattern_name[p]);
      mismatch = PSRAM_CompareAndLog(retry_tag, wr, rd, sizeof(wr));
    }
    mismatch_total += mismatch;
  }

  printf("[PSRAM][MMP] verify mismatch(total)=%d\r\n", mismatch_total);
}

#if (PSRAM_WINDOW_ALIAS_TEST_ENABLE == 1U)
static void PSRAM_AliasDebugProbe(void)
{
  static const char *pattern_name90[3] = {"WIN90-INC", "WIN90-ALT55AA", "WIN90-WALK1"};
  static const char *pattern_name91[3] = {"WIN91-INC", "WIN91-ALT55AA", "WIN91-WALK1"};
  static const uint32_t probe_addrs[2] = {0x00100000U, 0x00049800U};
  uint8_t wr[256];
  uint8_t rd90[256];
  uint8_t rd91[256];
  uint32_t aidx;
  int mismatch_total = 0;
  int independent_count = 0;
  int failed_count = 0;

  printf("[PSRAM][WIN] begin base90=0x%08lX base91=0x91000000 len=%lu\r\n",
         (unsigned long)XSPI1_BASE,
         (unsigned long)sizeof(wr));

  for (aidx = 0U; aidx < 2U; aidx++)
  {
    uint32_t p;
    uint32_t off = probe_addrs[aidx];
    volatile uint8_t *m90 = (volatile uint8_t *)(XSPI1_BASE + off);
    volatile uint8_t *m91 = (volatile uint8_t *)(0x91000000UL + off);
    uintptr_t c90 = ((uintptr_t)m90) & ~(uintptr_t)31U;
    uintptr_t c91 = ((uintptr_t)m91) & ~(uintptr_t)31U;
    int32_t clen = 288; /* cover 256-byte sample with line alignment margin */

    printf("[PSRAM][WIN] off=0x%08lX p90=0x%08lX p91=0x%08lX\r\n",
           (unsigned long)off,
           (unsigned long)(XSPI1_BASE + off),
           (unsigned long)(0x91000000UL + off));

    for (p = 0U; p < 3U; p++)
    {
      uint32_t i;
      int mismatch90;
      int mismatch91;
      int independent;

      /* Keep same pattern on both windows; validate each window independently first. */
      PSRAM_FillPattern(wr, sizeof(wr), p);

      for (i = 0U; i < sizeof(wr); i++)
      {
        m90[i] = wr[i];
      }

#if defined(USE_DCACHE)
      SCB_CleanDCache_by_Addr((uint32_t *)c90, clen);
      __DSB();
      SCB_InvalidateDCache_by_Addr((uint32_t *)c90, clen);
      __DSB();
      __ISB();
#endif

      for (i = 0U; i < sizeof(wr); i++)
      {
        rd90[i] = m90[i];
      }

      /* Write/read 0x91 window in a separate pass to avoid cross-side effects while diagnosing. */
      for (i = 0U; i < sizeof(wr); i++)
      {
        m91[i] = wr[i];
      }

#if defined(USE_DCACHE)
      SCB_CleanDCache_by_Addr((uint32_t *)c91, clen);
      __DSB();
      SCB_InvalidateDCache_by_Addr((uint32_t *)c91, clen);
      __DSB();
      __ISB();
#endif

      for (i = 0U; i < sizeof(wr); i++)
      {
        rd91[i] = m91[i];
      }

      mismatch90 = PSRAM_CompareAndLog(pattern_name90[p], wr, rd90, sizeof(wr));
      mismatch91 = PSRAM_CompareAndLog(pattern_name91[p], wr, rd91, sizeof(wr));
      mismatch_total += mismatch90;
      mismatch_total += mismatch91;

      independent = ((mismatch90 == 0) && (mismatch91 == 0)) ? 1 : 0;

      if (independent != 0)
      {
        independent_count++;
      }
      else
      {
        failed_count++;
      }

      printf("[PSRAM][WIN] mode p=%lu independent=%d\r\n",
             (unsigned long)p,
             independent);
    }
  }

  printf("[PSRAM][WIN] summary mismatch(total)=%d independent=%d failed=%d\r\n",
         mismatch_total,
         independent_count,
         failed_count);
}
#endif

static int main_freertos()
{
  TaskHandle_t hdl;

  hdl = xTaskCreateStatic(main_thread_fct, "main", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,
                          main_thread_stack, &main_thread);
  assert(hdl != NULL);

  vTaskStartScheduler();
  assert(0);

  return -1;
}

static void main_thread_fct(void *arg)
{
  uint32_t preemptPriority;
  uint32_t subPriority;
  IRQn_Type i;
  int ret;

  /* Copy SysTick_IRQn priority set by RTOS and use it as default priorities for IRQs. We are now sure that all irqs
   * have default priority below or equal to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
   */
  HAL_NVIC_GetPriority(SysTick_IRQn, HAL_NVIC_GetPriorityGrouping(), &preemptPriority, &subPriority);
  for (i = PVD_PVM_IRQn; i <= LTDC_UP_ERR_IRQn; i++)
    HAL_NVIC_SetPriority(i, preemptPriority, subPriority);

  /* Call SystemClock_Config() after vTaskStartScheduler() since it call HAL_Delay() which call vTaskDelay(). Drawback
   * is that we must call vPortSetupTimerInterrupt() since SystemCoreClock value has been modified by SystemClock_Config()
   */
  SystemClock_Config();
  vPortSetupTimerInterrupt();

  CONSOLE_Config();



  NPURam_enable();
  Fuse_Programming();

  DMA2D_Config();

  NPUCache_config();

  /*** External RAM and NOR Flash *********************************************/
  ret = BSP_XSPI_RAM_Init(0);
  g_psram_init_ret = ret;
  if ((ret != BSP_ERROR_NONE) || (PSRAM_SCAN_INFO_LOG_ENABLE == 1U))
  {
    printf("[PSRAM] Init ret=%d\r\n", ret);
  }
  if (ret != BSP_ERROR_NONE)
  {
    printf("[PSRAM] STOP: init failed, abort boot.\r\n");
    while (1)
    {
      HAL_Delay(1000);
    }
  }
  if (ret == BSP_ERROR_NONE)
  {
#if (PSRAM_QUICK_PROBE_ENABLE == 1U)
    PSRAM_DebugProbe();
#endif
#if (PSRAM_FULL_SCAN_ENABLE == 1U)
    PSRAM_FullScanDebugProbe();
#endif
    ret = BSP_XSPI_RAM_EnableMemoryMappedMode(0);
    g_psram_mmp_ret = ret;
    if ((ret != BSP_ERROR_NONE) || (PSRAM_SCAN_INFO_LOG_ENABLE == 1U))
    {
      printf("[PSRAM] MMP enable ret=%d\r\n", ret);
    }
    if (ret == BSP_ERROR_NONE)
    {
#if ((PSRAM_W958_YUV_TEST_ENABLE == 1U) && (BSP_XSPI_RAM_USE_W958D6NBKX == 1U))
      {
        int32_t yuv_ret = BSP_XSPI_RAM_W958_TestYuv422RgbBuffer(0U);
        if (yuv_ret != BSP_ERROR_NONE)
        {
          printf("[PSRAM][YUVTEST] FAIL ret=%ld\r\n", (long)yuv_ret);
        }
        else
        {
          printf("[PSRAM][YUVTEST] PASS\r\n");
        }
      }
#endif

#if (PSRAM_QUICK_PROBE_ENABLE == 1U)
      PSRAM_MMP_DebugProbe();
#endif
#if (PSRAM_MMP_READ_SCAN_ENABLE == 1U)
      /* Full MMP read-back expects data left by FULL scan last pattern (WALK1, id=2). */
      PSRAM_MMP_ReadScanDebugProbe(2U);
#endif
#if (PSRAM_WINDOW_ALIAS_TEST_ENABLE == 1U)
      PSRAM_AliasDebugProbe();
#else
      if (PSRAM_SCAN_INFO_LOG_ENABLE == 1U)
      {
        printf("[PSRAM][WIN] skipped (enable PSRAM_WINDOW_ALIAS_TEST_ENABLE=1 to run)\r\n");
      }
#endif
    }
    else
    {
      printf("[PSRAM] STOP: MMP enable failed, abort boot.\r\n");
      while (1)
      {
        HAL_Delay(1000);
      }
    }
  }

  BSP_XSPI_NOR_Init_t NOR_Init;
  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
  BSP_XSPI_NOR_Init(0, &NOR_Init);
  BSP_XSPI_NOR_EnableMemoryMappedMode(0);

  ret = BSP_LED_Init(LED_GREEN);
  assert(ret == BSP_ERROR_NONE);

  ret = BSP_LED_Init(LED_RED);
  assert(ret == BSP_ERROR_NONE);

  /* Set all required IPs as secure privileged */
  Security_Config();

  IAC_Config();

  /* Keep all IP's enabled during WFE so they can wake up CPU. Fine tune
   * this if you want to save maximum power
   */
  LL_BUS_EnableClockLowPower(~0);
  LL_MEM_EnableClockLowPower(~0);
  LL_AHB1_GRP1_EnableClockLowPower(~0);
  LL_AHB2_GRP1_EnableClockLowPower(~0);
  LL_AHB3_GRP1_EnableClockLowPower(~0);
  LL_AHB4_GRP1_EnableClockLowPower(~0);
  LL_AHB5_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP2_EnableClockLowPower(~0);
  LL_APB2_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP2_EnableClockLowPower(~0);
  LL_APB5_GRP1_EnableClockLowPower(~0);
  LL_MISC_EnableClockLowPower(~0);

  app_run();

  vTaskDelete(NULL);
}

HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};
  HAL_StatusTypeDef ret;

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP;
  RCC_PeriphCLKInitStruct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
  /* Align DCMIPP clock setup with DCMIPP_ContinuousMode reference (IC17 <- PLL1/4). */
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockDivider = 4;
  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  if (ret)
    return ret;

  return HAL_OK;
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  assert(hpcd->Instance == USB1_OTG_HS);

  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable the VDD33USB independent USB 33 voltage monitor */
  HAL_PWREx_EnableVddUSBVMEN();

  /* Wait until VDD33USB is ready */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_USB33RDY) == 0U);

  /* Enable VDDUSB supply */
  HAL_PWREx_EnableVddUSB();

  /* Enable USB1 OTG clock */
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

  /* Set FSEL to 24 Mhz */
  USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 0x4U);
  USB1_HS_PHYC->USBPHYC_CR |= (0x2U << 0x4U);

  /* Enable USB1 OTG PHY clock */
  __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

  /* Enable USB OTG interrupt */
  HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
}

void npu_cache_enable_clocks_and_reset()
{
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
  __HAL_RCC_CACHEAXI_RELEASE_RESET();
}

void npu_cache_disable_clocks_and_reset()
{
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  UNUSED(file);
  UNUSED(line);
  __BKPT(0);
  while (1)
  {
  }
}
#endif

/* Allow to debug with cache enable */
__attribute__ ((section (".keep_me"))) void app_clean_invalidate_dbg()
{
  SCB_CleanInvalidateDCache();
}
