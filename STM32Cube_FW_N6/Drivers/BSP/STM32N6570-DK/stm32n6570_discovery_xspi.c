/**
  ******************************************************************************
  * @file    stm32n6570_discovery_xspi.c
  * @author  MCD Application Team
  * @brief   This file includes a standard driver for the MX66UW1G45G and the APS256XX
  *          XSPI memories mounted on the STM32N6570-DK board.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) This driver is used to drive the MX66UW1G45G Octal NOR and the APS256XX Octal PSRAM
       external memories mounted on STM32N6570-DK board.

   (#) This driver need specific component driver (MX66UW1G45G and APS256XX) to be included with.

   (#) MX66UW1G45G Initialization steps:
       (++) Initialize the XSPI external memory using the BSP_XSPI_NOR_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            XSPI interface with the external memory.

   (#) MX66UW1G45G Octal NOR memory operations
       (++) XSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_XSPI_NOR_Read()/BSP_XSPI_NOR_Write().
       (++) The function BSP_XSPI_NOR_GetInfo() returns the configuration of the XSPI memory.
            (see the XSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_XSPI_NOR_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole
            chip by calling the function BSP_XSPI_NOR_Erase_Chip().
       (++) The function BSP_XSPI_NOR_GetStatus() returns the current status of the XSPI memory.
            (see the XSPI memory data sheet)
       (++) The memory access can be configured in memory-mapped mode with the call of
            function BSP_XSPI_NOR_EnableMemoryMapped(). To go back in indirect mode, the
            function BSP_XSPI_NOR_DisableMemoryMapped() should be used.
       (++) The erase operation can be suspend and resume with using functions
            BSP_XSPI_NOR_SuspendErase() and BSP_XSPI_NOR_ResumeErase()
       (++) It is possible to put the memory in deep power-down mode to reduce its consumption.
            For this, the function BSP_XSPI_NOR_EnterDeepPowerDown() should be called. To leave
            the deep power-down mode, the function BSP_XSPI_NOR_LeaveDeepPowerDown() should be called.
       (++) The function BSP_XSPI_NOR_ReadID() returns the identifier of the memory
            (see the XSPI memory data sheet)
       (++) The configuration of the interface between peripheral and memory is done by
            the function BSP_XSPI_NOR_ConfigFlash(), three modes are possible :
            - SPI : instruction, address and data on one line
            - STR OPI : instruction, address and data on eight lines with sampling on one edge of clock
            - DTR OPI : instruction, address and data on eight lines with sampling on both edgaes of clock

   (#) APS256XX Octal PSRAM memory Initialization steps:
       (++) Initialize the Octal PSRAM external memory using the BSP_XSPI_RAM_Init() function. This
            function includes the MSP layer hardware resources initialization and the
            XSPI interface with the external memory.

   (#) APS256XXL Octal PSRAM memory operations
       (++) Octal PSRAM memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_XSPI_RAM_Read()/BSP_XSPI_RAM_Write().
       (++) The memory access can be configured in memory-mapped mode with the call of
            function BSP_XSPI_RAM_EnableMemoryMapped(). To go back in indirect mode, the
            function BSP_XSPI_RAM_DisableMemoryMapped() should be used.
       (++) The function BSP_XSPI_RAM_ReadID() returns the identifier of the memory
            (see the XSPI memory data sheet)

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32n6570_discovery_xspi.h"
#include <stdio.h>
#include <string.h>

#ifndef BSP_XSPI_RAM_DEBUG_LOG_ENABLE
/* 0: only error logs, 1: include info logs */
#define BSP_XSPI_RAM_DEBUG_LOG_ENABLE 0U
#endif

#ifndef BSP_XSPI_RAM_CMD_LOG_ENABLE
/* 0: hide [CMD] logs, 1: show [CMD] logs */
#define BSP_XSPI_RAM_CMD_LOG_ENABLE 0U
#endif

#ifndef BSP_XSPI_RAM_W958_INIT_LOG_ENABLE
/* 0: disable init summary logs, 1: enable init summary logs */
#define BSP_XSPI_RAM_W958_INIT_LOG_ENABLE 1U
#endif

#ifndef BSP_XSPI_RAM_W958_REGTRACE_LOG_ENABLE
/* 0: disable low-level reg access trace, 1: enable */
#define BSP_XSPI_RAM_W958_REGTRACE_LOG_ENABLE 0U
#endif

#define XSPI_RAM_LOG_ERR(fmt, ...) printf("[XSPI-RAM] " fmt "\r\n", ##__VA_ARGS__)

#if (BSP_XSPI_RAM_DEBUG_LOG_ENABLE == 1U)
#define XSPI_RAM_LOG_INFO(fmt, ...) printf("[XSPI-RAM] " fmt "\r\n", ##__VA_ARGS__)
#else
#define XSPI_RAM_LOG_INFO(fmt, ...)
#endif
#define XSPI_RAM_LOG XSPI_RAM_LOG_INFO

#if (BSP_XSPI_RAM_W958_INIT_LOG_ENABLE == 1U)
#define XSPI_RAM_W958_INIT_LOG(fmt, ...) printf("[XSPI-RAM] [XSPI_RAM_W958_init] " fmt "\r\n", ##__VA_ARGS__)
#else
#define XSPI_RAM_W958_INIT_LOG(fmt, ...)
#endif

#if (BSP_XSPI_RAM_W958_REGTRACE_LOG_ENABLE == 1U)
#define XSPI_RAM_W958_REGTRACE_LOG(fmt, ...) XSPI_RAM_LOG_ERR("[W958][TRACE] " fmt, ##__VA_ARGS__)
#else
#define XSPI_RAM_W958_REGTRACE_LOG(fmt, ...)
#endif

#define XSPI_RAM_LOG_HAL_ERR(inst, tag) \
  XSPI_RAM_LOG_ERR("%s HAL state=%lu err=0x%08lX SR=0x%08lX", \
                   (tag), \
                   (unsigned long)HAL_XSPI_GetState(&hxspi_ram[(inst)]), \
                   (unsigned long)HAL_XSPI_GetError(&hxspi_ram[(inst)]), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->SR))

#define XSPI_RAM_LOG_REGS(inst, tag) \
  XSPI_RAM_LOG_ERR("%s CR=0x%08lX DCR1=0x%08lX HLCR=0x%08lX CCR=0x%08lX WCCR=0x%08lX AR=0x%08lX DLR=%lu", \
                   (tag), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->CR), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->DCR1), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->HLCR), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->CCR), \
                   (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->WCCR), \
                    (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->AR), \
                    (unsigned long)(READ_REG(hxspi_ram[(inst)].Instance->DLR) + 1U))

#define XSPI_RAM_LOG_SR_DECODE(inst, tag) \
  do { \
    uint32_t _sr = (uint32_t)READ_REG(hxspi_ram[(inst)].Instance->SR); \
    (void)_sr; \
    XSPI_RAM_W958_REGTRACE_LOG("%s SR=0x%08lX TE=%lu TC=%lu FT=%lu SM=%lu TO=%lu BUSY=%lu FLEVEL=%lu", \
                               (tag), \
                               (unsigned long)_sr, \
                               (unsigned long)((_sr & XSPI_SR_TEF) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_TCF) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_FTF) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_SMF) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_TOF) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_BUSY) ? 1U : 0U), \
                               (unsigned long)((_sr & XSPI_SR_FLEVEL) >> XSPI_SR_FLEVEL_Pos)); \
  } while (0)

#define XSPI_RAM_LOG_CMD(tag, aspace, addr, aw, len, dqs, dmode) \
  do { \
    if (BSP_XSPI_RAM_CMD_LOG_ENABLE == 1U) \
    { \
      XSPI_RAM_LOG_INFO("%s aspace=%lu addr=0x%08lX aw=%lu len=%lu dqs=%lu dmode=%lu", \
                        (tag), \
                        (unsigned long)(aspace), \
                        (unsigned long)(addr), \
                        (unsigned long)(aw), \
                        (unsigned long)(len), \
                        (unsigned long)(dqs), \
                        (unsigned long)(dmode)); \
    } \
  } while (0)

#define XSPI_RAM_TRY_ABORT(inst, tag) \
  do { \
    CLEAR_BIT(hxspi_ram[(inst)].Instance->CR, XSPI_CR_FMODE); \
    hxspi_ram[(inst)].State = HAL_XSPI_STATE_READY; \
    XSPI_RAM_LOG_ERR("%s soft-recover to READY", (tag)); \
  } while (0)

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32N6570_DK
  * @{
  */

/** @defgroup STM32N6570_DK_XSPI XSPI
  * @{
  */

/* Exported variables --------------------------------------------------------*/
#if (USE_NOR_MEMORY_MX66UW1G45G == 1)
/** @addtogroup STM32N6570_DK_XSPI_NOR_Exported_Variables
  * @{
  */
extern XSPI_NOR_Ctx_t XSPI_Nor_Ctx[XSPI_NOR_INSTANCES_NUMBER];

XSPI_HandleTypeDef hxspi_nor[XSPI_NOR_INSTANCES_NUMBER] = {0};

XSPI_NOR_Ctx_t XSPI_Nor_Ctx[XSPI_NOR_INSTANCES_NUMBER]  = {{
    XSPI_ACCESS_NONE,
    MX66UW1G45G_SPI_MODE,
    MX66UW1G45G_STR_TRANSFER
  }
};
/**
  * @}
  */
#endif /* (USE_NOR_MEMORY_MX66UW1G45G == 1) */

/* Exported variables --------------------------------------------------------*/
#if (USE_RAM_MEMORY_APS256XX == 1)
/** @addtogroup STM32N6570_DK_XSPI_RAM_Exported_Variables
  * @{
  */
extern XSPI_RAM_Ctx_t XSPI_Ram_Ctx[XSPI_RAM_INSTANCES_NUMBER];

XSPI_HandleTypeDef hxspi_ram[XSPI_RAM_INSTANCES_NUMBER] = {0};

XSPI_RAM_Ctx_t XSPI_Ram_Ctx[XSPI_RAM_INSTANCES_NUMBER] = {{
    XSPI_ACCESS_NONE,
    BSP_XSPI_RAM_VARIABLE_LATENCY,
    BSP_XSPI_RAM_HYBRID_BURST,
    APS256XX_BURST_16_BYTES
  }
};

/**
  * @}
  */
#endif /* (USE_RAM_MEMORY_APS256XX == 1) */

/* Private constants --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32N6570_DK_XSPI_NOR_Private_Variables XSPI_NOR Private Variables
  * @{
  */
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
static uint32_t XSPINor_IsMspCbValid[XSPI_NOR_INSTANCES_NUMBER] = {0};
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup STM32N6570_DK_XSPI_RAM_Private_Variables XSPI_RAM Private Variables
  * @{
  */
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
static uint32_t XSPIRam_IsMspCbValid[XSPI_RAM_INSTANCES_NUMBER] = {0};
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */
/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/

#if (USE_NOR_MEMORY_MX66UW1G45G == 1)
/** @defgroup STM32N6570_DK_XSPI_NOR_Private_Functions XSPI_NOR Private Functions
  * @{
  */
static void    XSPI_NOR_MspInit(const XSPI_HandleTypeDef *hxspi);
static void    XSPI_NOR_MspDeInit(const XSPI_HandleTypeDef *hxspi);
static int32_t XSPI_NOR_ResetMemory(uint32_t Instance);
static int32_t XSPI_NOR_EnterDOPIMode(uint32_t Instance);
static int32_t XSPI_NOR_EnterSOPIMode(uint32_t Instance);
static int32_t XSPI_NOR_ExitOPIMode(uint32_t Instance);
/**
  * @}
  */
#endif /* (USE_NOR_MEMORY_MX66UW1G45G == 1) */

#if (USE_RAM_MEMORY_APS256XX == 1)
/** @defgroup STM32N6570_DK_XSPI_RAM_Private_Functions XSPI_RAM Private Functions
  * @{
  */
static void XSPI_RAM_MspInit(const XSPI_HandleTypeDef *hxspi);
static void XSPI_RAM_MspDeInit(const XSPI_HandleTypeDef *hxspi);
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
/* ------- driver W958D6NBKX : private prototypes begin ------- */
static int32_t XSPI_RAM_W958_WriteReg(uint32_t Instance, uint32_t RegIndex, uint16_t Value);
static int32_t XSPI_RAM_W958_ReadReg(uint32_t Instance, uint32_t RegIndex, uint16_t *Value);
static int32_t XSPI_RAM_W958_ReadMem(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
static int32_t XSPI_RAM_W958_WriteMem(uint32_t Instance, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
static int32_t XSPI_RAM_W958_HardRecover(uint32_t Instance, const char *reason_tag);
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */
static int32_t XSPI_RAM_W958_AddressRwTest(uint32_t Instance);
static void XSPI_RAM_W958_LogCR0(const char *tag, uint16_t cr0);
static void XSPI_RAM_W958_LogCR1(const char *tag, uint16_t cr1);
static void XSPI_RAM_W958_LogID(const char *tag, uint16_t id0, uint16_t id1);
static void XSPI_RAM_W958_DebugCR0Mismatch(uint32_t Instance, uint16_t cr0_exp, uint16_t cr0_rb,
                                           uint16_t cr1_exp, uint16_t cr1_rb);
static void XSPI_RAM_W958_DCacheCleanRange(const void *buf, uint32_t len);
static void XSPI_RAM_W958_DCacheInvalidateRange(const void *buf, uint32_t len);
static void XSPI_RAM_W958_HexHead(const uint8_t *buf, uint32_t len, char *out, uint32_t out_size);
static void XSPI_RAM_W958_MmpWrite(volatile uint8_t *dst, const uint8_t *src, uint32_t len);
static void XSPI_RAM_W958_MmpRead(const volatile uint8_t *src, uint8_t *dst, uint32_t len);
/* ------- driver W958D6NBKX : private prototypes end ------- */
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
/**
  * @}
  */
#endif /* (USE_RAM_MEMORY_APS256XX == 1) */

/* Exported functions ---------------------------------------------------------*/

#if (USE_NOR_MEMORY_MX66UW1G45G == 1)
/** @addtogroup STM32N6570_DK_XSPI_NOR_Exported_Functions
  * @{
  */

/**
  * @brief  Initializes the XSPI interface.
  * @param  Instance   XSPI Instance
  * @param  Init       XSPI Init structure
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_Init(uint32_t Instance, BSP_XSPI_NOR_Init_t *Init)
{
  int32_t ret;
  MX_XSPI_InitTypeDef xspi_init;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (XSPI_Nor_Ctx[Instance].IsInitialized == XSPI_ACCESS_NONE)
    {
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      /* Msp XSPI initialization */
      XSPI_NOR_MspInit(&hxspi_nor[Instance]);
#else
      /* Register the XSPI MSP Callbacks */
      if (XSPINor_IsMspCbValid[Instance] == 0UL)
      {
        if (BSP_XSPI_NOR_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

      /* Fill config structure */
      xspi_init.ClockPrescaler = 0x03; /* XSPI clock = 200MHz / ClockPrescaler = 50MHz, then switch to 200MHz*/
      xspi_init.MemorySize     = (uint32_t)POSITION_VAL((uint32_t)BSP_XSPI_NOR_FLASH_SIZE_BYTES) - 1U;
      xspi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
      xspi_init.TransferRate   = (uint32_t)Init->TransferRate;

      /* STM32 XSPI interface initialization */
      if (MX_XSPI_NOR_Init(&hxspi_nor[Instance], &xspi_init) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      /* XSPI memory reset */
      else if (XSPI_NOR_ResetMemory(Instance) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      /* Check if memory is ready */
      else if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                                XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      /* Configure the memory */
      else if (BSP_XSPI_NOR_ConfigFlash(Instance, Init->InterfaceMode, Init->TransferRate) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  if ((ret == BSP_ERROR_NONE) &&
      (Init->InterfaceMode == BSP_XSPI_NOR_OPI_MODE) &&
      (Init->TransferRate == BSP_XSPI_NOR_DTR_TRANSFER))
  {
    (void) (HAL_XSPI_SetClockPrescaler(&hxspi_nor[Instance], 0));
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  De-Initializes the XSPI interface.
  * @param  Instance   XSPI Instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (XSPI_Nor_Ctx[Instance].IsInitialized != XSPI_ACCESS_NONE)
    {
      /* Disable Memory mapped mode */
      if (XSPI_Nor_Ctx[Instance].IsInitialized == XSPI_ACCESS_MMP)
      {
        if (BSP_XSPI_NOR_DisableMemoryMappedMode(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_COMPONENT_FAILURE;
        }
      }

      /* Set default XSPI_Nor_Ctx values */
      XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_NONE;
      XSPI_Nor_Ctx[Instance].InterfaceMode = BSP_XSPI_NOR_SPI_MODE;
      XSPI_Nor_Ctx[Instance].TransferRate  = BSP_XSPI_NOR_STR_TRANSFER;

      /* Call the DeInit function to reset the driver */
      if (HAL_XSPI_DeInit(&hxspi_nor[Instance]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }

#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      XSPI_NOR_MspDeInit(&hxspi_nor[Instance]);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 0) */
    }
  }

  /* Return BSP status */
  return ret;
}
/**
  * @}
  */

/** @addtogroup STM32N6570_DK_XSPI_Exported_Init_Functions
  * @{
  */
/**
  * @brief  Initializes the XSPI interface.
  * @param  hxspi          XSPI handle
  * @param  Init           XSPI config structure
  * @retval BSP status
  */
__weak HAL_StatusTypeDef MX_XSPI_NOR_Init(XSPI_HandleTypeDef *hxspi, MX_XSPI_InitTypeDef *Init)
{
  /* XSPI initialization */
  hxspi->Instance = XSPI2;

  hxspi->Init.FifoThresholdByte       = 1;
  hxspi->Init.MemorySize              = Init->MemorySize; /* 1 GBits */
  hxspi->Init.ChipSelectHighTimeCycle = 2;
  hxspi->Init.FreeRunningClock        = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi->Init.ClockMode               = HAL_XSPI_CLOCK_MODE_0;
  hxspi->Init.DelayHoldQuarterCycle   = HAL_XSPI_DHQC_DISABLE;
  hxspi->Init.ClockPrescaler          = Init->ClockPrescaler;
  hxspi->Init.SampleShifting          = Init->SampleShifting;
  hxspi->Init.ChipSelectBoundary      = HAL_XSPI_BONDARYOF_NONE;
  hxspi->Init.MemoryMode              = HAL_XSPI_SINGLE_MEM;
  hxspi->Init.WrapSize                = HAL_XSPI_WRAP_NOT_SUPPORTED;

  if (Init->TransferRate == (uint32_t) BSP_XSPI_NOR_DTR_TRANSFER)
  {
    hxspi->Init.MemoryType            = HAL_XSPI_MEMTYPE_MACRONIX;
    hxspi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  }
  else
  {
    hxspi->Init.MemoryType            = HAL_XSPI_MEMTYPE_MACRONIX;
    hxspi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  }
  return HAL_XSPI_Init(hxspi);
}
/**
  * @}
  */

/** @addtogroup STM32N6570_DK_XSPI_NOR_Exported_Functions
  * @{
  */
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief Default BSP XSPI Msp Callbacks
  * @param Instance      XSPI Instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hxspi_nor[Instance], HAL_XSPI_MSP_INIT_CB_ID, ((pXSPI_CallbackTypeDef) XSPI_NOR_MspInit)) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hxspi_nor[Instance], HAL_XSPI_MSP_DEINIT_CB_ID, ((pXSPI_CallbackTypeDef) XSPI_NOR_MspDeInit)) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      XSPINor_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief BSP XSPI Msp Callback registering
  * @param Instance     XSPI Instance
  * @param CallBacks    pointer to MspInit/MspDeInit callbacks functions
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_RegisterMspCallbacks(uint32_t Instance, BSP_XSPI_Cb_t *CallBacks)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hxspi_nor[Instance], HAL_XSPI_MSP_INIT_CB_ID, CallBacks->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hxspi_nor[Instance],
                                       HAL_XSPI_MSP_DEINIT_CB_ID, CallBacks->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      XSPINor_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Reads an amount of data from the XSPI memory.
  * @param  Instance  XSPI instance
  * @param  pData     Pointer to data to be read
  * @param  ReadAddr  Read start address
  * @param  Size      Size of data to read
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_Read(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_STR_TRANSFER)
    {
      if (MX66UW1G45G_ReadSTR(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                               MX66UW1G45G_4BYTES_SIZE, pData, ReadAddr, Size) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {

      /* Bypass the Pre-scaler */
      (void) (HAL_XSPI_SetClockPrescaler(&hxspi_nor[Instance], 0));

      if (MX66UW1G45G_ReadDTR(&hxspi_nor[Instance], pData, ReadAddr, Size) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Writes an amount of data to the XSPI memory.
  * @param  Instance  XSPI instance
  * @param  pData     Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size      Size of data to write
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_Write(uint32_t Instance, const uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t end_addr;
  uint32_t current_size;
  uint32_t current_addr;
  uint32_t data_addr;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Calculation of the size between the write address and the end of the page */
    current_size = MX66UW1G45G_PAGE_SIZE - (WriteAddr % MX66UW1G45G_PAGE_SIZE);

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size)
    {
      current_size = Size;
    }

    /* Initialize the address variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;
    data_addr = (uint32_t)pData;

    /* Perform the write page by page */
    do
    {

      /* Check if Flash busy ? */
      if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                           XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }/* Enable write operations */
      else if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                        XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        if (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_STR_TRANSFER)
        {
          /* Issue page program command */
          if (MX66UW1G45G_PageProgram(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                       MX66UW1G45G_4BYTES_SIZE, (uint8_t *)data_addr, current_addr,
                                       current_size) != MX66UW1G45G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
        }
        else
        {
          /* Issue page program command */
          if (MX66UW1G45G_PageProgramDTR(&hxspi_nor[Instance], (uint8_t *)data_addr, current_addr,
                                          current_size) != MX66UW1G45G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
        }

        if (ret == BSP_ERROR_NONE)
        {
          /* Configure automatic polling mode to wait for end of program */
          if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                               XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
          else
          {
            /* Update the address and size variables for next page programming */
            current_addr += current_size;
            data_addr += current_size;
            current_size = ((current_addr + MX66UW1G45G_PAGE_SIZE) > end_addr)
                           ? (end_addr - current_addr)
                           : MX66UW1G45G_PAGE_SIZE;
          }
        }
      }
    } while ((current_addr < end_addr) && (ret == BSP_ERROR_NONE));
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Erases the specified block of the XSPI memory.
  * @param  Instance     XSPI instance
  * @param  BlockAddress Block address to erase
  * @param  BlockSize    Erase Block size
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_Erase_Block(uint32_t Instance, uint32_t BlockAddress, BSP_XSPI_NOR_Erase_t BlockSize)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check Flash busy ? */
    if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                         XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Enable write operations */
    else if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                      XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Issue Block Erase command */
    else if (MX66UW1G45G_BlockErase(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                     XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_4BYTES_SIZE,
                                     BlockAddress, BlockSize) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Erases the entire XSPI memory.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_Erase_Chip(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check Flash busy ? */
    if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                         XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Enable write operations */
    else if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                      XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Issue Chip erase command */
    else if (MX66UW1G45G_ChipErase(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                     XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Reads current status of the XSPI memory.
  * @param  Instance  XSPI instance
  * @retval XSPI memory status: whether busy or not
  */
int32_t BSP_XSPI_NOR_GetStatus(uint32_t Instance)
{
  static uint8_t reg[2];
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (MX66UW1G45G_ReadSecurityRegister(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, reg) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Check the value of the register */
    else if ((reg[0] & (MX66UW1G45G_SECR_P_FAIL | MX66UW1G45G_SECR_E_FAIL)) != 0U)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if ((reg[0] & (MX66UW1G45G_SECR_PSB | MX66UW1G45G_SECR_ESB)) != 0U)
    {
      ret = BSP_ERROR_XSPI_SUSPENDED;
    }
    else if (MX66UW1G45G_ReadStatusRegister(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                             XSPI_Nor_Ctx[Instance].TransferRate, reg) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }/* Check the value of the register */
    else if ((reg[0] & MX66UW1G45G_SR_WIP) != 0U)
    {
      ret = BSP_ERROR_BUSY;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Return the configuration of the XSPI memory.
  * @param  Instance  XSPI instance
  * @param  pInfo     pointer on the configuration structure
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_GetInfo(uint32_t Instance, BSP_XSPI_NOR_Info_t *pInfo)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (MX66UW1G45G_GetFlashInfo(pInfo) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Override DK default (128MB) with custom-board NOR size. */
      pInfo->FlashSize           = BSP_XSPI_NOR_FLASH_SIZE_BYTES;
      pInfo->EraseSectorSize     = MX66UW1G45G_BLOCK_64K;
      pInfo->EraseSectorsNumber  = (BSP_XSPI_NOR_FLASH_SIZE_BYTES / MX66UW1G45G_BLOCK_64K);
      pInfo->EraseSubSectorSize  = MX66UW1G45G_BLOCK_4K;
      pInfo->EraseSubSectorNumber = (BSP_XSPI_NOR_FLASH_SIZE_BYTES / MX66UW1G45G_BLOCK_4K);
      pInfo->EraseSubSector1Size = MX66UW1G45G_BLOCK_4K;
      pInfo->EraseSubSector1Number = (BSP_XSPI_NOR_FLASH_SIZE_BYTES / MX66UW1G45G_BLOCK_4K);
      pInfo->ProgPageSize        = MX66UW1G45G_PAGE_SIZE;
      pInfo->ProgPagesNumber     = (BSP_XSPI_NOR_FLASH_SIZE_BYTES / MX66UW1G45G_PAGE_SIZE);
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Configure the XSPI in memory-mapped mode
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((XSPI_Nor_Ctx[Instance].InterfaceMode == BSP_XSPI_NOR_OPI_MODE) &&
        (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_DTR_TRANSFER))
    {
      /* Bypass the Pre-scaler only for high-speed OPI/DTR mode. */
      (void) (HAL_XSPI_SetClockPrescaler(&hxspi_nor[Instance], 0));
    }

    if (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_STR_TRANSFER)
    {
      if (MX66UW1G45G_EnableMemoryMappedModeSTR(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                                 MX66UW1G45G_4BYTES_SIZE) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else /* Update XSPI context if all operations are well done */
      {
        XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_MMP;
      }
    }
    else
    {
      if (MX66UW1G45G_EnableMemoryMappedModeDTR(&hxspi_nor[Instance],
                                                 XSPI_Nor_Ctx[Instance].InterfaceMode) != MX66UW1G45G_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else /* Update XSPI context if all operations are well done */
      {

       XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_MMP;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Exit form memory-mapped mode
  *         Only 1 Instance can running MMP mode. And it will lock system at this mode.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_DisableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (XSPI_Nor_Ctx[Instance].IsInitialized != XSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_XSPI_MMP_UNLOCK_FAILURE;
    }/* Abort MMP back to indirect mode */
    else if (HAL_XSPI_Abort(&hxspi_nor[Instance]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else /* Update XSPI NOR context if all operations are well done */
    {
      XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Get flash ID 3 Bytes:
  *         Manufacturer ID, Memory type, Memory density
  * @param  Instance  XSPI instance
  * @param  Id Pointer to flash ID bytes
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX66UW1G45G_ReadID(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                               XSPI_Nor_Ctx[Instance].TransferRate, Id) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Set Flash to desired Interface mode. And this instance becomes current instance.
  *         If current instance running at MMP mode then this function doesn't work.
  *         Indirect -> Indirect
  * @param  Instance  XSPI instance
  * @param  Mode      XSPI mode
  * @param  Rate      XSPI transfer rate
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_ConfigFlash(uint32_t Instance, BSP_XSPI_NOR_Interface_t Mode, BSP_XSPI_NOR_Transfer_t Rate)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if MMP mode locked ************************************************/
    if (XSPI_Nor_Ctx[Instance].IsInitialized == XSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_XSPI_MMP_LOCK_FAILURE;
    }
    else
    {
      /* Setup Flash interface ***************************************************/
      switch (XSPI_Nor_Ctx[Instance].InterfaceMode)
      {
        case BSP_XSPI_NOR_OPI_MODE :  /* 8-8-8 commands */
          if ((Mode != BSP_XSPI_NOR_OPI_MODE) || (Rate != XSPI_Nor_Ctx[Instance].TransferRate))
          {
            /* Exit OPI mode */
            ret = XSPI_NOR_ExitOPIMode(Instance);

            if ((ret == BSP_ERROR_NONE) && (Mode == BSP_XSPI_NOR_OPI_MODE))
            {

              if (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_STR_TRANSFER)
              {
                /* Enter DTR OPI mode */
                ret = XSPI_NOR_EnterDOPIMode(Instance);
              }
              else
              {
                /* Enter STR OPI mode */
                ret = XSPI_NOR_EnterSOPIMode(Instance);
              }
            }
          }
          break;

        case BSP_XSPI_NOR_SPI_MODE :  /* 1-1-1 commands, Power on H/W default setting */
        default :
          if (Mode == BSP_XSPI_NOR_OPI_MODE)
          {
            if (Rate == BSP_XSPI_NOR_STR_TRANSFER)
            {
              /* Enter STR OPI mode */
              ret = XSPI_NOR_EnterSOPIMode(Instance);
            }
            else
            {
              /* Enter DTR OPI mode */
              ret = XSPI_NOR_EnterDOPIMode(Instance);
            }
          }
          break;
      }

      /* Update XSPI context if all operations are well done */
      if (ret == BSP_ERROR_NONE)
      {
        /* Update current status parameter *****************************************/
        XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;
        XSPI_Nor_Ctx[Instance].InterfaceMode = Mode;
        XSPI_Nor_Ctx[Instance].TransferRate  = Rate;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_SuspendErase(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  /* Check whether the device is busy (erase operation is in progress). */
  else if (BSP_XSPI_NOR_GetStatus(Instance) != BSP_ERROR_BUSY)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX66UW1G45G_Suspend(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (BSP_XSPI_NOR_GetStatus(Instance) != BSP_ERROR_XSPI_SUSPENDED)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function resumes a paused erase command.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_ResumeErase(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  /* Check whether the device is busy (erase operation is in progress). */
  else if (BSP_XSPI_NOR_GetStatus(Instance) != BSP_ERROR_XSPI_SUSPENDED)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX66UW1G45G_Resume(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                               XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /*
  When this command is executed, the status register write in progress bit is set to 1, and
  the flag status register program erase controller bit is set to 0. This command is ignored
  if the device is not in a suspended state.
  */
  else if (BSP_XSPI_NOR_GetStatus(Instance) != BSP_ERROR_BUSY)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enter the XSPI memory in deep power down mode.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_EnterDeepPowerDown(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX66UW1G45G_EnterPowerDown(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                       XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* ---          Memory takes 10us max to enter deep power down          --- */

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function leave the XSPI memory from deep power down mode.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_NOR_LeaveDeepPowerDown(uint32_t Instance)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= XSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MX66UW1G45G_NoOperation(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* --- A NOP command is sent to the memory, as the nCS should be low for at least 20 ns --- */
  /* ---                  Memory takes 30us min to leave deep power down                  --- */

  /* Return BSP status */
  return ret;
}
/**
  * @}
  */
#endif /* (USE_NOR_MEMORY_MX66UW1G45G == 1) */

#if (USE_RAM_MEMORY_APS256XX == 1)
/** @addtogroup STM32N6570_DK_XSPI_RAM_Exported_Functions
  * @{
  */

#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
/* ------- driver W958D6NBKX : helper functions begin ------- */
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
static int32_t XSPI_RAM_W958_HardRecover(uint32_t Instance, const char *reason_tag)
{
  MX_XSPI_InitTypeDef xspi_init = {0};
  XSPI_HyperbusCfgTypeDef hb_cfg = {0};
  uint32_t sr_before;

  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  sr_before = (uint32_t)READ_REG(hxspi_ram[Instance].Instance->SR);
  XSPI_RAM_LOG_ERR("[W958][RECOVER] begin reason=%s SR=0x%08lX", reason_tag, (unsigned long)sr_before);

  XSPI_RAM_FORCE_RESET();
  XSPI_RAM_RELEASE_RESET();

  xspi_init.ClockPrescaler = BSP_XSPI_RAM_W958_BOOT_PRESCALER;
  xspi_init.MemorySize     = BSP_XSPI_RAM_W958_XSPI_MEMORY_SIZE;
  xspi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;

  if (MX_XSPI_RAM_Init(&hxspi_ram[Instance], &xspi_init) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][RECOVER][ERR] MX_XSPI_RAM_Init failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][RECOVER][ERR] regs after MX init fail");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  hb_cfg.RWRecoveryTimeCycle = BSP_XSPI_RAM_W958_RW_RECOVERY_CYCLES;
  hb_cfg.AccessTimeCycle = BSP_XSPI_RAM_W958_ACCESS_CYCLES;
  hb_cfg.WriteZeroLatency = HAL_XSPI_LATENCY_ON_WRITE;
  hb_cfg.LatencyMode = HAL_XSPI_FIXED_LATENCY;

  if (HAL_XSPI_HyperbusCfg(&hxspi_ram[Instance], &hb_cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][RECOVER][ERR] HyperbusCfg failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][RECOVER][ERR] regs after HyperbusCfg fail");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  HAL_Delay(1U);
  XSPI_RAM_LOG_SR_DECODE(Instance, "W958][RECOVER/end");
  XSPI_RAM_LOG_ERR("[W958][RECOVER] done reason=%s", reason_tag);
  return BSP_ERROR_NONE;
}
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */

static int32_t XSPI_RAM_W958_WriteReg(uint32_t Instance, uint32_t RegIndex, uint16_t Value)
{
  XSPI_HyperbusCmdTypeDef sCommand = {0};
  uint8_t wr[2];
  wr[0] = (uint8_t)(Value & 0xFFU);
  wr[1] = (uint8_t)((Value >> 8) & 0xFFU);

  sCommand.AddressSpace = HAL_XSPI_REGISTER_ADDRESS_SPACE;
  sCommand.Address = RegIndex;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.DataLength = 2U;
  /* Register transaction must not use RWDS. */
  sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_REG_DATA_MODE;

  XSPI_RAM_W958_REGTRACE_LOG("WriteReg begin inst=%lu reg=0x%08lX val=0x%04X dmode=%lu dqs=%lu",
                             (unsigned long)Instance,
                             (unsigned long)RegIndex,
                             (unsigned int)Value,
                             (unsigned long)sCommand.DataMode,
                             (unsigned long)sCommand.DQSMode);
  XSPI_RAM_LOG_SR_DECODE(Instance, "WriteReg/begin");
  XSPI_RAM_LOG_CMD("[W958][CMD] WriteReg", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_SR_DECODE(Instance, "WriteReg/cmd_fail");
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteReg HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }
  XSPI_RAM_LOG_SR_DECODE(Instance, "WriteReg/cmd_ok");
  XSPI_RAM_W958_REGTRACE_LOG("WriteReg tx bytes=%02X %02X", (unsigned int)wr[0], (unsigned int)wr[1]);
  if (HAL_XSPI_Transmit(&hxspi_ram[Instance], wr, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_SR_DECODE(Instance, "WriteReg/tx_fail");
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteReg TX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  XSPI_RAM_LOG_SR_DECODE(Instance, "WriteReg/tx_ok");
  XSPI_RAM_W958_REGTRACE_LOG("WriteReg done inst=%lu reg=0x%08lX val=0x%04X",
                             (unsigned long)Instance,
                             (unsigned long)RegIndex,
                             (unsigned int)Value);
  XSPI_RAM_LOG("[W958] WriteReg addr=0x%08lX val=0x%04X", (unsigned long)RegIndex, Value);
  return BSP_ERROR_NONE;
}

static int32_t XSPI_RAM_W958_ReadReg(uint32_t Instance, uint32_t RegIndex, uint16_t *Value)
{
  XSPI_HyperbusCmdTypeDef sCommand = {0};
  uint8_t rd[2] = {0};

  if (Value == NULL)
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  sCommand.AddressSpace = HAL_XSPI_REGISTER_ADDRESS_SPACE;
  sCommand.Address = RegIndex;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.DataLength = 2U;
  /* Roll back to known-good register read mode used in earlier successful ID reads. */
  sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_REG_DATA_MODE;

  XSPI_RAM_W958_REGTRACE_LOG("ReadReg begin inst=%lu reg=0x%08lX dmode=%lu dqs=%lu",
                             (unsigned long)Instance,
                             (unsigned long)RegIndex,
                             (unsigned long)sCommand.DataMode,
                             (unsigned long)sCommand.DQSMode);
  XSPI_RAM_LOG_SR_DECODE(Instance, "ReadReg/begin");
  XSPI_RAM_LOG_CMD("[W958][CMD] ReadReg", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_SR_DECODE(Instance, "ReadReg/cmd_fail");
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadReg HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }
  XSPI_RAM_LOG_SR_DECODE(Instance, "ReadReg/cmd_ok");
  if (HAL_XSPI_Receive(&hxspi_ram[Instance], rd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_SR_DECODE(Instance, "ReadReg/rx_fail");
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadReg RX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  XSPI_RAM_LOG_SR_DECODE(Instance, "ReadReg/rx_ok");
  *Value = ((uint16_t)rd[1] << 8) | rd[0];
  XSPI_RAM_W958_REGTRACE_LOG("ReadReg done inst=%lu reg=0x%08lX rd=%02X %02X val=0x%04X",
                             (unsigned long)Instance,
                             (unsigned long)RegIndex,
                             (unsigned int)rd[0],
                             (unsigned int)rd[1],
                             (unsigned int)*Value);
  XSPI_RAM_LOG("[W958] ReadReg addr=0x%08lX val=0x%04X", (unsigned long)RegIndex, (unsigned int)*Value);
  return BSP_ERROR_NONE;
}

static int32_t XSPI_RAM_W958_ReadMem(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  XSPI_HyperbusCmdTypeDef sCommand = {0};

  if ((pData == NULL) || (Size == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  sCommand.AddressSpace = HAL_XSPI_MEMORY_ADDRESS_SPACE;
  sCommand.Address = ReadAddr;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.DataLength = Size;
  sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_MEM_DATA_MODE;

  XSPI_RAM_LOG_CMD("[W958][CMD] ReadMem", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadMem HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadMem");
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
    (void)XSPI_RAM_W958_HardRecover(Instance, "ReadMem/cmd_fail");
#else
    XSPI_RAM_LOG_ERR("[W958][ERR] ReadMem stop: recover disabled");
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Receive(&hxspi_ram[Instance], pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadMem RX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadMem");
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
    (void)XSPI_RAM_W958_HardRecover(Instance, "ReadMem/rx_fail");
#else
    XSPI_RAM_LOG_ERR("[W958][ERR] ReadMem stop: recover disabled");
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static int32_t XSPI_RAM_W958_WriteMem(uint32_t Instance, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  XSPI_HyperbusCmdTypeDef sCommand = {0};

  if ((pData == NULL) || (Size == 0U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  sCommand.AddressSpace = HAL_XSPI_MEMORY_ADDRESS_SPACE;
  sCommand.Address = WriteAddr;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.DataLength = Size;
  /* RWDS/DQS mode is configurable to match board wiring and byte-mask behavior. */
  sCommand.DQSMode = BSP_XSPI_RAM_W958_WRITE_DQS_MODE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_MEM_DATA_MODE;

  XSPI_RAM_LOG_CMD("[W958][CMD] WriteMem", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteMem HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteMem");
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
    (void)XSPI_RAM_W958_HardRecover(Instance, "WriteMem/cmd_fail");
#else
    XSPI_RAM_LOG_ERR("[W958][ERR] WriteMem stop: recover disabled");
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Transmit(&hxspi_ram[Instance], pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteMem TX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteMem");
#if (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U)
    (void)XSPI_RAM_W958_HardRecover(Instance, "WriteMem/tx_fail");
#else
    XSPI_RAM_LOG_ERR("[W958][ERR] WriteMem stop: recover disabled");
#endif /* (BSP_XSPI_RAM_W958_ENABLE_RECOVER == 1U) */
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}

static int32_t XSPI_RAM_W958_AddressRwTest(uint32_t Instance)
{
#if (BSP_XSPI_RAM_W958_ADDR_TEST_ENABLE == 1U)
  uint8_t wr[BSP_XSPI_RAM_W958_ADDR_TEST_BYTES];
  uint8_t rd[BSP_XSPI_RAM_W958_ADDR_TEST_BYTES];
  uint32_t off;
  const uint32_t test_len = BSP_XSPI_RAM_W958_ADDR_TEST_BYTES;
  uint32_t j;
  uint16_t id0 = 0U;
  uint16_t id1 = 0U;
  uint16_t cr0 = 0U;
  uint16_t cr1 = 0U;
  char wr_hex[(3U * BSP_XSPI_RAM_W958_ADDR_TEST_BYTES) + 1U];
  char rd_hex[(3U * BSP_XSPI_RAM_W958_ADDR_TEST_BYTES) + 1U];

  if ((test_len == 0U) || (test_len > BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES))
  {
    XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest invalid len=%lu (chunk=%lu)",
                     (unsigned long)test_len,
                     (unsigned long)BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES);
    return BSP_ERROR_WRONG_PARAM;
  }

  if (test_len > BSP_XSPI_RAM_SIZE_BYTES)
  {
    XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest invalid len=%lu size=%lu",
                     (unsigned long)test_len,
                     (unsigned long)BSP_XSPI_RAM_SIZE_BYTES);
    return BSP_ERROR_WRONG_PARAM;
  }

  /* Register sanity check: only log when error. */
  if ((XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID0_ADDR, &id0) != BSP_ERROR_NONE) ||
      (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID1_ADDR, &id1) != BSP_ERROR_NONE) ||
      (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0) != BSP_ERROR_NONE) ||
      (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, &cr1) != BSP_ERROR_NONE))
  {
    XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest reg read fail");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  if ((cr0 != BSP_XSPI_RAM_W958_CR0_INIT) || (cr1 != BSP_XSPI_RAM_W958_CR1_INIT))
  {
    XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest reg mismatch ID0=0x%04X ID1=0x%04X exp_CR0=0x%04X got_CR0=0x%04X exp_CR1=0x%04X got_CR1=0x%04X",
                     (unsigned int)id0,
                     (unsigned int)id1,
                     (unsigned int)BSP_XSPI_RAM_W958_CR0_INIT,
                     (unsigned int)cr0,
                     (unsigned int)BSP_XSPI_RAM_W958_CR1_INIT,
                     (unsigned int)cr1);
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Full RAM scan with fixed 32-byte transfers; log only on failure. */
  for (off = 0U; (off + test_len) <= BSP_XSPI_RAM_SIZE_BYTES; off += test_len)
  {
    for (j = 0U; j < test_len; j++)
    {
      wr[j] = (uint8_t)(((off >> 8) + (j * 13U) + 0x5AU) & 0xFFU);
      rd[j] = 0xAAU;
    }

    if (XSPI_RAM_W958_WriteMem(Instance, wr, off, test_len) != BSP_ERROR_NONE)
    {
      XSPI_RAM_W958_HexHead(wr, test_len, wr_hex, (uint32_t)sizeof(wr_hex));
      XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest FULL write fail off=0x%08lX len=%lu wr=%s",
                       (unsigned long)off,
                       (unsigned long)test_len,
                       wr_hex);
      return BSP_ERROR_PERIPH_FAILURE;
    }

    if (XSPI_RAM_W958_ReadMem(Instance, rd, off, test_len) != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest FULL read fail off=0x%08lX len=%lu",
                       (unsigned long)off,
                       (unsigned long)test_len);
      return BSP_ERROR_PERIPH_FAILURE;
    }

    for (j = 0U; j < test_len; j++)
    {
      if (rd[j] != wr[j])
      {
        XSPI_RAM_W958_HexHead(wr, test_len, wr_hex, (uint32_t)sizeof(wr_hex));
        XSPI_RAM_W958_HexHead(rd, test_len, rd_hex, (uint32_t)sizeof(rd_hex));
        XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest FULL mismatch off=0x%08lX i=%lu exp=%02X got=%02X",
                         (unsigned long)off,
                         (unsigned long)j,
                         (unsigned int)wr[j],
                         (unsigned int)rd[j]);
        XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest FULL WR=%s", wr_hex);
        XSPI_RAM_LOG_ERR("[W958][ERR] AddrTest FULL RD=%s", rd_hex);
        return BSP_ERROR_PERIPH_FAILURE;
      }
    }
  }

  return BSP_ERROR_NONE;
#else
  (void)Instance;
  return BSP_ERROR_NONE;
#endif /* (BSP_XSPI_RAM_W958_ADDR_TEST_ENABLE == 1U) */
}

static void XSPI_RAM_W958_DCacheCleanRange(const void *buf, uint32_t len)
{
#if defined(USE_DCACHE)
  uintptr_t addr;
  uintptr_t clean_addr;
  uint32_t clean_len;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  addr = (uintptr_t)buf;
  clean_addr = addr & ~((uintptr_t)31U);
  clean_len = (uint32_t)(((addr - clean_addr) + (uintptr_t)len + 31U) & ~((uintptr_t)31U));
  SCB_CleanDCache_by_Addr((uint8_t *)clean_addr, clean_len);
#else
  (void)buf;
  (void)len;
#endif
}

static void XSPI_RAM_W958_DCacheInvalidateRange(const void *buf, uint32_t len)
{
#if defined(USE_DCACHE)
  uintptr_t addr;
  uintptr_t inv_addr;
  uint32_t inv_len;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  addr = (uintptr_t)buf;
  inv_addr = addr & ~((uintptr_t)31U);
  inv_len = (uint32_t)(((addr - inv_addr) + (uintptr_t)len + 31U) & ~((uintptr_t)31U));
  SCB_InvalidateDCache_by_Addr((uint8_t *)inv_addr, inv_len);
#else
  (void)buf;
  (void)len;
#endif
}

static void XSPI_RAM_W958_HexHead(const uint8_t *buf, uint32_t len, char *out, uint32_t out_size)
{
  uint32_t i;
  uint32_t p = 0U;

  if ((out == NULL) || (out_size == 0U))
  {
    return;
  }

  if (buf == NULL)
  {
    out[0] = '\0';
    return;
  }

  for (i = 0U; i < len; i++)
  {
    int n = snprintf(&out[p], (size_t)(out_size - p), "%02X%s",
                     (unsigned int)buf[i], (i + 1U < len) ? " " : "");
    if (n < 0)
    {
      break;
    }
    p += (uint32_t)n;
    if (p >= out_size)
    {
      break;
    }
  }
  out[out_size - 1U] = '\0';
}

static void XSPI_RAM_W958_MmpWrite(volatile uint8_t *dst, const uint8_t *src, uint32_t len)
{
  uint32_t i = 0U;
  volatile uint8_t *d8 = (volatile uint8_t *)dst;

  if ((dst == NULL) || (src == NULL) || (len == 0U))
  {
    return;
  }

  if ((((uintptr_t)dst | (uintptr_t)src) & 0x3U) == 0U)
  {
    volatile uint32_t *d32 = (volatile uint32_t *)dst;
    const uint32_t *s32 = (const uint32_t *)src;
    uint32_t words = len / 4U;
    for (i = 0U; i < words; i++)
    {
      d32[i] = s32[i];
    }
    i = words * 4U;
  }

  for (; i < len; i++)
  {
    d8[i] = src[i];
  }
}

static void XSPI_RAM_W958_MmpRead(const volatile uint8_t *src, uint8_t *dst, uint32_t len)
{
  uint32_t i = 0U;
  const volatile uint8_t *s8 = src;

  if ((src == NULL) || (dst == NULL) || (len == 0U))
  {
    return;
  }

  if ((((uintptr_t)src | (uintptr_t)dst) & 0x3U) == 0U)
  {
    const volatile uint32_t *s32 = (const volatile uint32_t *)src;
    uint32_t *d32 = (uint32_t *)dst;
    uint32_t words = len / 4U;
    for (i = 0U; i < words; i++)
    {
      d32[i] = s32[i];
    }
    i = words * 4U;
  }

  for (; i < len; i++)
  {
    dst[i] = s8[i];
  }
}

static void XSPI_RAM_W958_LogCR0(const char *tag, uint16_t cr0)
{
  uint32_t dpd = ((uint32_t)cr0 >> 15) & 0x1U;
  uint32_t ds = ((uint32_t)cr0 >> 12) & 0x7U;
  uint32_t rsv = ((uint32_t)cr0 >> 8) & 0xFU;
  uint32_t il = ((uint32_t)cr0 >> 4) & 0xFU;
  uint32_t fixed = ((uint32_t)cr0 >> 3) & 0x1U;
  uint32_t hybrid = ((uint32_t)cr0 >> 2) & 0x1U;
  uint32_t burst = (uint32_t)cr0 & 0x3U;
  uint32_t burst_len = (burst == 0U) ? 128U : ((burst == 1U) ? 64U : ((burst == 2U) ? 16U : 32U));
  XSPI_RAM_LOG("[%s] CR0=0x%04X dpd=%lu ds=%lu il=%lu fixed=%lu hybrid=%lu burst_sel=%lu burst_len=%lu rsv=0x%lX",
               tag,
               (unsigned int)cr0,
               (unsigned long)dpd,
               (unsigned long)ds,
               (unsigned long)il,
               (unsigned long)fixed,
               (unsigned long)hybrid,
               (unsigned long)burst,
               (unsigned long)burst_len,
               (unsigned long)rsv);
}

static void XSPI_RAM_W958_LogCR1(const char *tag, uint16_t cr1)
{
  uint32_t clk_type = ((uint32_t)cr1 >> 6) & 0x1U;
  uint32_t hs = ((uint32_t)cr1 >> 5) & 0x1U;
  uint32_t par = ((uint32_t)cr1 >> 2) & 0x7U;
  uint32_t dri = (uint32_t)cr1 & 0x3U;
  const char *par_desc;

  switch (par)
  {
    case 0U: par_desc = "full"; break;
    case 1U: par_desc = "bottom_1_2"; break;
    case 2U: par_desc = "bottom_1_4"; break;
    case 3U: par_desc = "bottom_1_8"; break;
    case 5U: par_desc = "top_1_2"; break;
    case 6U: par_desc = "top_1_4"; break;
    case 7U: par_desc = "top_1_8"; break;
    default: par_desc = "reserved"; break;
  }

  XSPI_RAM_LOG("[%s] CR1=0x%04X clk=%s hs=%lu par=%lu(%s) dri=%lu",
               tag,
               (unsigned int)cr1,
               (clk_type == 1U) ? "single_ended" : "differential",
               (unsigned long)hs,
               (unsigned long)par,
               par_desc,
               (unsigned long)dri);
}

static void XSPI_RAM_W958_LogID(const char *tag, uint16_t id0, uint16_t id1)
{
  uint32_t row_bits = ((uint32_t)id0 >> 8) & 0x1FU;
  uint32_t col_bits = ((uint32_t)id0 >> 4) & 0x0FU;
  uint32_t mfg = (uint32_t)id0 & 0x0FU;
  uint32_t dev = (uint32_t)id1 & 0x0FU;
  const char *mfg_name = (mfg == 0x6U) ? "Winbond" : "Unknown";
  const char *dev_name = (dev == 0x9U) ? "HyperRAM3.0" : "Unknown";

  XSPI_RAM_LOG("[%s] ID0=0x%04X ID1=0x%04X row_bits=%lu col_bits=%lu mfg=0x%lX(%s) dev=0x%lX(%s)",
               tag,
               (unsigned int)id0,
               (unsigned int)id1,
               (unsigned long)row_bits,
               (unsigned long)col_bits,
               (unsigned long)mfg,
               mfg_name,
               (unsigned long)dev,
               dev_name);
}

#if defined(__GNUC__)
__attribute__((unused))
#endif
static void XSPI_RAM_W958_DebugCR0Mismatch(uint32_t Instance, uint16_t cr0_exp, uint16_t cr0_rb,
                                           uint16_t cr1_exp, uint16_t cr1_rb)
{
  uint16_t cr0_now = 0U;
  uint16_t diff = (uint16_t)(cr0_exp ^ cr0_rb);
  uint16_t test_val[4];
  const char *test_name[4] = { "exp", "set_b2", "clr_b2", "toggle_burst01" };
  uint32_t i;

  XSPI_RAM_LOG("[W958][DBG] CR mismatch debug begin inst=%lu diff=0x%04X",
               (unsigned long)Instance, (unsigned int)diff);
  XSPI_RAM_W958_LogCR0("W958][DBG][CR0_EXP", cr0_exp);
  XSPI_RAM_W958_LogCR0("W958][DBG][CR0_RB", cr0_rb);
  XSPI_RAM_W958_LogCR1("W958][DBG][CR1_EXP", cr1_exp);
  XSPI_RAM_W958_LogCR1("W958][DBG][CR1_RB", cr1_rb);

  for (i = 0U; i < 3U; i++)
  {
    if (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0_now) == BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][DBG] CR0 sample[%lu]=0x%04X", (unsigned long)i, (unsigned int)cr0_now);
    }
    else
    {
      XSPI_RAM_LOG("[W958][DBG][ERR] CR0 sample[%lu] read fail", (unsigned long)i);
    }
  }

  test_val[0] = cr0_exp;
  test_val[1] = (uint16_t)(cr0_exp | 0x0004U); /* set bit2 */
  test_val[2] = (uint16_t)(cr0_exp & (uint16_t)~0x0004U); /* clear bit2 */
  test_val[3] = (uint16_t)((cr0_exp & (uint16_t)~0x0003U) | ((cr0_exp + 1U) & 0x0003U)); /* toggle burst bits */

  for (i = 0U; i < 4U; i++)
  {
    uint16_t wr = test_val[i];
    uint16_t rb = 0U;

    XSPI_RAM_LOG("[W958][DBG] CR0 test[%lu]=%s wr=0x%04X", (unsigned long)i, test_name[i], (unsigned int)wr);
    if (XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, wr) != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][DBG][ERR] CR0 test[%lu] write fail", (unsigned long)i);
      continue;
    }
    if (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &rb) != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][DBG][ERR] CR0 test[%lu] readback fail", (unsigned long)i);
      continue;
    }
    XSPI_RAM_LOG("[W958][DBG] CR0 test[%lu] rb=0x%04X xor=0x%04X",
                 (unsigned long)i, (unsigned int)rb, (unsigned int)(wr ^ rb));
  }

  /* Restore expected value (best effort). */
  if (XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, cr0_exp) == BSP_ERROR_NONE)
  {
    if (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0_now) == BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][DBG] CR0 restored=0x%04X", (unsigned int)cr0_now);
    }
  }

  XSPI_RAM_LOG("[W958][DBG] CR mismatch debug end");
}
/* ------- driver W958D6NBKX : helper functions end ------- */
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */

/**
  * @brief  Initializes the XSPI interface.
  * @param  Instance   XSPI Instance
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_Init(uint32_t Instance)
{
  MX_XSPI_InitTypeDef xspi_init;
  int32_t ret = BSP_ERROR_NONE;

  XSPI_RAM_W958_INIT_LOG("enter inst=%lu use_w958=%lu skip_vendor=%lu",
                         (unsigned long)Instance,
                         (unsigned long)BSP_XSPI_RAM_USE_W958D6NBKX,
                         (unsigned long)BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT);
  XSPI_RAM_W958_INIT_LOG("PATCH-MARKER: W958_CODE_EDIT_2026_03_21");

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
    XSPI_RAM_W958_INIT_LOG("Step 1 FAIL: instance check (inst=%lu out of range)", (unsigned long)Instance);
  }
  else
  {
    XSPI_RAM_W958_INIT_LOG("Step 1 OK: instance check (inst=%lu)", (unsigned long)Instance);
    XSPI_RAM_W958_INIT_LOG("ctx is_initialized=%lu",
                           (unsigned long)XSPI_Ram_Ctx[Instance].IsInitialized);
    XSPI_RAM_W958_INIT_LOG("Step 2: check init context");

    /* Check if the instance is already initialized */
    if (XSPI_Ram_Ctx[Instance].IsInitialized == XSPI_ACCESS_NONE)
    {
      XSPI_RAM_W958_INIT_LOG("Step 2 OK: context is NONE, run full init");
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      /* Msp XSPI initialization */
      XSPI_RAM_W958_INIT_LOG("Step 3: MSP init begin");
      XSPI_RAM_MspInit(&hxspi_ram[Instance]);
      XSPI_RAM_W958_INIT_LOG("Step 3 OK: MSP init done");
#else
      /* Register the XSPI MSP Callbacks */
      XSPI_RAM_W958_INIT_LOG("Step 3: callback registration begin");
      if (XSPIRam_IsMspCbValid[Instance] == 0UL)
      {
        if (BSP_XSPI_RAM_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("Step 3 FAIL: callback registration failed");
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      XSPI_RAM_W958_INIT_LOG("Step 3 OK: callback registration done");
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

      /* Fill config structure */
      XSPI_RAM_W958_INIT_LOG("Step 4: prepare base XSPI config");
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
      xspi_init.ClockPrescaler = BSP_XSPI_RAM_W958_BOOT_PRESCALER;
#else
      xspi_init.ClockPrescaler = 3;
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
      xspi_init.MemorySize     = BSP_XSPI_RAM_W958_XSPI_MEMORY_SIZE;
      xspi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
      XSPI_RAM_W958_INIT_LOG("base_cfg clk_ps=%lu mem_size=%lu sample_shift=%lu",
                             (unsigned long)xspi_init.ClockPrescaler,
                             (unsigned long)xspi_init.MemorySize,
                             (unsigned long)xspi_init.SampleShifting);
      XSPI_RAM_W958_INIT_LOG("Step 4 OK: base XSPI config ready");

      /* STM32 XSPI interface initialization */
      XSPI_RAM_W958_INIT_LOG("Step 5: MX_XSPI_RAM_Init begin");
      if (MX_XSPI_RAM_Init(&hxspi_ram[Instance], &xspi_init) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
        XSPI_RAM_LOG_ERR("[W958][BOOT][ERR] MX_XSPI_RAM_Init failed");
        XSPI_RAM_W958_INIT_LOG("Step 5 FAIL: MX_XSPI_RAM_Init failed");
      }
      else
      {
        /* Update current status parameter */
        XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;
        XSPI_Ram_Ctx[Instance].LatencyType   = BSP_XSPI_RAM_FIXED_LATENCY;
        XSPI_Ram_Ctx[Instance].BurstType     = BSP_XSPI_RAM_LINEAR_BURST;
        XSPI_RAM_W958_INIT_LOG("xspi_core_init_ok ctx_mode=%lu",
                               (unsigned long)XSPI_Ram_Ctx[Instance].IsInitialized);
        XSPI_RAM_W958_INIT_LOG("Step 5 OK: XSPI core init done");
      }
    }
    else
    {
      XSPI_RAM_W958_INIT_LOG("Step 2 SKIP: already initialized (ctx_mode=%lu)",
                             (unsigned long)XSPI_Ram_Ctx[Instance].IsInitialized);
    }

    if (ret == BSP_ERROR_NONE)
    {
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
      /* ------- driver W958D6NBKX : init path begin ------- */
      XSPI_HyperbusCfgTypeDef hb_cfg = {0};
      XSPI_RAM_W958_INIT_LOG("path=W958");
      XSPI_RAM_W958_INIT_LOG("Step 6: configure HyperBus timing");
      XSPI_RAM_W958_INIT_LOG("cfg CR0=0x%04X CR1=0x%04X reg_dmode=%lu mem_dmode=%lu wr_dqs=%lu chunk=%lu boot_ps=%lu post_ps=%lu",
                             (unsigned int)BSP_XSPI_RAM_W958_CR0_INIT,
                             (unsigned int)BSP_XSPI_RAM_W958_CR1_INIT,
                             (unsigned long)BSP_XSPI_RAM_W958_REG_DATA_MODE,
                             (unsigned long)BSP_XSPI_RAM_W958_MEM_DATA_MODE,
                             (unsigned long)BSP_XSPI_RAM_W958_WRITE_DQS_MODE,
                             (unsigned long)BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES,
                             (unsigned long)BSP_XSPI_RAM_W958_BOOT_PRESCALER,
                             (unsigned long)BSP_XSPI_RAM_W958_POST_INIT_PRESCALER);

      hb_cfg.RWRecoveryTimeCycle = BSP_XSPI_RAM_W958_RW_RECOVERY_CYCLES;
      hb_cfg.AccessTimeCycle = BSP_XSPI_RAM_W958_ACCESS_CYCLES;
      hb_cfg.WriteZeroLatency = HAL_XSPI_LATENCY_ON_WRITE;
      hb_cfg.LatencyMode = HAL_XSPI_FIXED_LATENCY;
      XSPI_RAM_W958_INIT_LOG("hb_cfg trwr=%lu tacc=%lu write_lat=%lu lat_mode=%lu",
                             (unsigned long)hb_cfg.RWRecoveryTimeCycle,
                             (unsigned long)hb_cfg.AccessTimeCycle,
                             (unsigned long)hb_cfg.WriteZeroLatency,
                             (unsigned long)hb_cfg.LatencyMode);
      if (HAL_XSPI_HyperbusCfg(&hxspi_ram[Instance], &hb_cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
        XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] HyperbusCfg failed");
        XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] init regs");
        XSPI_RAM_W958_INIT_LOG("Step 6 FAIL: HyperBus config failed");
      }
      else
      {
        XSPI_RAM_W958_INIT_LOG("hyperbus_cfg_ok");
        XSPI_RAM_W958_INIT_LOG("Step 6 OK: HyperBus config done");
      }

      /*
       * Guard delay after interface/hyperbus setup before first register/data access.
       * Helps satisfy power-up/settling margin from HyperRAM bring-up notes.
       */
      if (ret == BSP_ERROR_NONE)
      {
        XSPI_RAM_W958_INIT_LOG("Step 7: guard delay before CR access");
        HAL_Delay(1U);
        XSPI_RAM_W958_INIT_LOG("guard_delay_ms=1 before CR access");
        XSPI_RAM_W958_INIT_LOG("Step 7 OK: guard delay done");
      }

      if (ret == BSP_ERROR_NONE)
      {
        uint32_t sr_before_id = (uint32_t)READ_REG(hxspi_ram[Instance].Instance->SR);
        if ((sr_before_id & XSPI_SR_BUSY) != 0U)
        {
          XSPI_RAM_W958_INIT_LOG("Step 8 PRECHECK: BUSY set before ReadID SR=0x%08lX", (unsigned long)sr_before_id);
          XSPI_RAM_W958_INIT_LOG("Step 8 PRECHECK: clear status flags + re-sync XSPI");
          WRITE_REG(hxspi_ram[Instance].Instance->FCR, (HAL_XSPI_FLAG_TE | HAL_XSPI_FLAG_TC | HAL_XSPI_FLAG_SM | HAL_XSPI_FLAG_TO));

          XSPI_RAM_FORCE_RESET();
          XSPI_RAM_RELEASE_RESET();

          if (MX_XSPI_RAM_Init(&hxspi_ram[Instance], &xspi_init) != HAL_OK)
          {
            ret = BSP_ERROR_PERIPH_FAILURE;
            XSPI_RAM_LOG_ERR("[W958][BOOT][ERR] Step8 precheck MX_XSPI_RAM_Init failed");
          }
          else if (HAL_XSPI_HyperbusCfg(&hxspi_ram[Instance], &hb_cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
          {
            ret = BSP_ERROR_PERIPH_FAILURE;
            XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] Step8 precheck HyperbusCfg failed");
            XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] Step8 precheck regs");
          }
          else
          {
            HAL_Delay(1U);
            XSPI_RAM_W958_INIT_LOG("Step 8 PRECHECK OK: XSPI re-sync done");
          }
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        uint8_t id_probe[6] = {0};
        XSPI_RAM_W958_INIT_LOG("Step 8: ReadID probe begin");
        if (BSP_XSPI_RAM_ReadID(Instance, id_probe) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
          XSPI_RAM_W958_INIT_LOG("Step 8 FAIL: ReadID probe failed");
          XSPI_RAM_W958_INIT_LOG("Step 8 STOP: abort init because ID access failed");
        }
        else
        {
          XSPI_RAM_W958_INIT_LOG("Step 8 OK: ReadID probe done id=%02X %02X %02X %02X %02X %02X",
                                 (unsigned int)id_probe[0],
                                 (unsigned int)id_probe[1],
                                 (unsigned int)id_probe[2],
                                 (unsigned int)id_probe[3],
                                 (unsigned int)id_probe[4],
                                 (unsigned int)id_probe[5]);
        }
      }

#if (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U)
      if (ret == BSP_ERROR_NONE)
      {
        int32_t cr_ret = BSP_ERROR_NONE;
        uint16_t cr0_rb = 0U;
        uint16_t cr1_rb = 0U;
        XSPI_RAM_W958_INIT_LOG("Step 9: CR write/readback begin");
        XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR0 addr=0x%08lX val=0x%04X",
                               (unsigned long)BSP_XSPI_RAM_W958_CR0_ADDR,
                               (unsigned int)BSP_XSPI_RAM_W958_CR0_INIT);
        cr_ret = XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, BSP_XSPI_RAM_W958_CR0_INIT);
        if (cr_ret != BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR0 FAIL");
        }
        else
        {
          XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR0 done");
        }

        if (cr_ret == BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR1 addr=0x%08lX val=0x%04X",
                                 (unsigned long)BSP_XSPI_RAM_W958_CR1_ADDR,
                                 (unsigned int)BSP_XSPI_RAM_W958_CR1_INIT);
          cr_ret = XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, BSP_XSPI_RAM_W958_CR1_INIT);
          if (cr_ret != BSP_ERROR_NONE)
          {
            XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR1 FAIL");
          }
          else
          {
            XSPI_RAM_W958_INIT_LOG("CR step: WRITE CR1 done");
          }
        }

        if (cr_ret == BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR0 addr=0x%08lX",
                                 (unsigned long)BSP_XSPI_RAM_W958_CR0_ADDR);
          cr_ret = XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0_rb);
          if (cr_ret != BSP_ERROR_NONE)
          {
            XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR0 FAIL");
          }
          else
          {
            XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR0 val=0x%04X", (unsigned int)cr0_rb);
          }
        }

        if (cr_ret == BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR1 addr=0x%08lX",
                                 (unsigned long)BSP_XSPI_RAM_W958_CR1_ADDR);
          cr_ret = XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, &cr1_rb);
          if (cr_ret != BSP_ERROR_NONE)
          {
            XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR1 FAIL");
          }
          else
          {
            XSPI_RAM_W958_INIT_LOG("CR step: READBACK CR1 val=0x%04X", (unsigned int)cr1_rb);
          }
        }

        if (cr_ret == BSP_ERROR_NONE)
        {
          XSPI_RAM_LOG("[W958] CR write/readback CR0=0x%04X CR1=0x%04X",
                       (unsigned int)cr0_rb,
                       (unsigned int)cr1_rb);
          if ((cr0_rb != BSP_XSPI_RAM_W958_CR0_INIT) || (cr1_rb != BSP_XSPI_RAM_W958_CR1_INIT))
          {
            XSPI_RAM_LOG("[W958][ERR] CR readback mismatch exp(CR0=0x%04X CR1=0x%04X) got(CR0=0x%04X CR1=0x%04X)",
                         (unsigned int)BSP_XSPI_RAM_W958_CR0_INIT,
                         (unsigned int)BSP_XSPI_RAM_W958_CR1_INIT,
                         (unsigned int)cr0_rb,
                         (unsigned int)cr1_rb);
            cr_ret = BSP_ERROR_PERIPH_FAILURE;
          }
        }

        if (cr_ret == BSP_ERROR_NONE)
        {
          XSPI_RAM_W958_INIT_LOG("Step 9 OK: CR write/readback done");
        }
        else
        {
          XSPI_RAM_W958_INIT_LOG("Step 9 FAIL: CR write/readback has error");
          ret = BSP_ERROR_PERIPH_FAILURE;
          XSPI_RAM_W958_INIT_LOG("Step 9 STOP: abort init because CR access failed");
        }
      }
#else
      XSPI_RAM_W958_INIT_LOG("vendor-reg init skipped (use device default CR0/CR1)");
      XSPI_RAM_W958_INIT_LOG("Step 9 SKIP: vendor CR init disabled");
#endif /* (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U) */

      if (ret == BSP_ERROR_NONE)
      {
        uint16_t cr0_probe = 0U;
        uint16_t cr1_probe = 0U;
        XSPI_RAM_W958_INIT_LOG("Step 10: CR probe begin");
        XSPI_RAM_W958_INIT_LOG("CR step: PROBE READ CR0 addr=0x%08lX", (unsigned long)BSP_XSPI_RAM_W958_CR0_ADDR);
        XSPI_RAM_W958_INIT_LOG("CR step: PROBE READ CR1 addr=0x%08lX", (unsigned long)BSP_XSPI_RAM_W958_CR1_ADDR);
        if ((XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0_probe) == BSP_ERROR_NONE) &&
            (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, &cr1_probe) == BSP_ERROR_NONE))
        {
          XSPI_RAM_W958_INIT_LOG("CR step: PROBE RESULT CR0=0x%04X CR1=0x%04X",
                                 (unsigned int)cr0_probe,
                                 (unsigned int)cr1_probe);
          XSPI_RAM_W958_LogCR0("W958][INIT", cr0_probe);
          XSPI_RAM_W958_LogCR1("W958][INIT", cr1_probe);
          XSPI_RAM_W958_INIT_LOG("Step 10 OK: CR probe done");
        }
        else
        {
          XSPI_RAM_LOG("[W958][WARN] CR probe after init failed");
          XSPI_RAM_W958_INIT_LOG("Step 10 FAIL: CR probe failed");
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        XSPI_RAM_W958_INIT_LOG("Step 11: apply post-init prescaler");
        if (HAL_XSPI_SetClockPrescaler(&hxspi_ram[Instance], BSP_XSPI_RAM_W958_POST_INIT_PRESCALER) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
          XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] SetClockPrescaler failed");
          XSPI_RAM_W958_INIT_LOG("Step 11 FAIL: post-init prescaler failed");
        }
        else
        {
          XSPI_RAM_W958_INIT_LOG("post_init_prescaler=%lu applied",
                                 (unsigned long)BSP_XSPI_RAM_W958_POST_INIT_PRESCALER);
          XSPI_RAM_W958_INIT_LOG("Step 11 OK: post-init prescaler applied");
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        if (XSPI_RAM_W958_AddressRwTest(Instance) != BSP_ERROR_NONE)
        {
#if (BSP_XSPI_RAM_W958_ADDR_TEST_PASS_ALWAYS == 1U)
          XSPI_RAM_W958_INIT_LOG("Step 12 WARN: RAM register/buffer test failed");
          XSPI_RAM_W958_INIT_LOG("Step 12 FORCE-PASS: continue init as requested");
#else
          ret = BSP_ERROR_PERIPH_FAILURE;
          XSPI_RAM_W958_INIT_LOG("Step 12 FAIL: RAM register/buffer test failed");
          XSPI_RAM_W958_INIT_LOG("Step 12 STOP: abort init because RAM data test failed");
#endif /* (BSP_XSPI_RAM_W958_ADDR_TEST_PASS_ALWAYS == 1U) */
        }
      }
      /* ------- driver W958D6NBKX : init path end ------- */
#else
#if (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U)
      /* APS256XX-specific register tuning (disable for non-APS HyperRAM). */
      if (APS256XX_WriteReg(&hxspi_ram[Instance], 0U, 0x30U) != APS256XX_OK) /* Read latency=7 */
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if (APS256XX_WriteReg(&hxspi_ram[Instance], 4U, 0x20U) != APS256XX_OK) /* Write latency=7 */
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if (APS256XX_WriteReg(&hxspi_ram[Instance], 8U, 0x40U) != APS256XX_OK) /* x16 mode */
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
#endif /* (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U) */

      if ((ret == BSP_ERROR_NONE) && (HAL_XSPI_SetClockPrescaler(&hxspi_ram[Instance], 0U) != HAL_OK))
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
    }

  }

  XSPI_RAM_W958_INIT_LOG("exit inst=%lu ret=%ld ctx_mode=%lu",
                         (unsigned long)Instance,
                         (long)ret,
                         (unsigned long)((Instance < XSPI_RAM_INSTANCES_NUMBER) ? XSPI_Ram_Ctx[Instance].IsInitialized : 0UL));
  XSPI_RAM_W958_INIT_LOG("Step 13: init exit status=%s",
                         (ret == BSP_ERROR_NONE) ? "OK" : "FAIL");

  /* Return BSP status */
  return ret;
}

/**
  * @brief  De-Initializes the XSPI interface.
  * @param  Instance   XSPI Instance
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (XSPI_Ram_Ctx[Instance].IsInitialized != XSPI_ACCESS_NONE)
    {
      /* Disable Memory mapped mode */
      if (XSPI_Ram_Ctx[Instance].IsInitialized == XSPI_ACCESS_MMP)
      {
        if (BSP_XSPI_RAM_DisableMemoryMappedMode(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_COMPONENT_FAILURE;
        }
      }
      /* Set default XSPI_Ram_Ctx values */
      XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_NONE;
      XSPI_Ram_Ctx[Instance].LatencyType   = BSP_XSPI_RAM_FIXED_LATENCY;
      XSPI_Ram_Ctx[Instance].BurstType     = BSP_XSPI_RAM_LINEAR_BURST;

      /* Call the DeInit function to reset the driver */
      if (HAL_XSPI_DeInit(&hxspi_ram[Instance]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }

#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      XSPI_RAM_MspDeInit(&hxspi_ram[Instance]);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 0) */

    }
  }

  /* Return BSP status */
  return ret;
}
/**
  * @}
  */

/** @addtogroup STM32N6570_DK_XSPI_Exported_Init_Functions
  * @{
  */

/**
  * @brief  Initializes the XSPI interface.
  * @param  hxspi          XSPI handle
  * @param  Init           XSPI config structure
  * @retval BSP status
  */
__weak HAL_StatusTypeDef MX_XSPI_RAM_Init(XSPI_HandleTypeDef *hxspi, MX_XSPI_InitTypeDef *Init)
{
  uint32_t hspi_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI1);
  HAL_StatusTypeDef st;

  XSPI_RAM_W958_INIT_LOG("mx_init clk_ps=%lu mem_size=%lu sample_shift=%lu",
                         (unsigned long)Init->ClockPrescaler,
                         (unsigned long)Init->MemorySize,
                         (unsigned long)Init->SampleShifting);

  /* XSPI initialization */
  hxspi->Instance = XSPI1;

  hxspi->Init.FifoThresholdByte          = 8;
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
  hxspi->Init.MemoryType                 = HAL_XSPI_MEMTYPE_HYPERBUS;
#else
  hxspi->Init.MemoryType                 = HAL_XSPI_MEMTYPE_APMEM_16BITS;
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
  hxspi->Init.MemoryMode                 = HAL_XSPI_SINGLE_MEM;
  hxspi->Init.MemorySize                 = Init->MemorySize;
  hxspi->Init.MemorySelect               = HAL_XSPI_CSSEL_NCS1;
  hxspi->Init.ChipSelectHighTimeCycle    = 5;
  hxspi->Init.ClockMode                  = HAL_XSPI_CLOCK_MODE_0;
  hxspi->Init.ClockPrescaler             = Init->ClockPrescaler;
  hxspi->Init.SampleShifting             = Init->SampleShifting;
  hxspi->Init.DelayHoldQuarterCycle      = HAL_XSPI_DHQC_ENABLE;
  hxspi->Init.ChipSelectBoundary         = HAL_XSPI_BONDARYOF_16KB;
  hxspi->Init.FreeRunningClock           = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi->Init.Refresh                    = ((2U * (hspi_clk / hxspi->Init.ClockPrescaler)) / 1000000U) - 4U;
#if defined (OCTOSPI_DCR1_DLYBYP)
  hxspi->Init.DelayBlockBypass           = HAL_XSPI_DELAY_BLOCK_BYPASS;
#endif /* defined (OCTOSPI_DCR1_DLYBYP) */
  hxspi->Init.WrapSize                   = HAL_XSPI_WRAP_NOT_SUPPORTED;

  st = HAL_XSPI_Init(hxspi);
  if (st != HAL_OK)
  {
    XSPI_RAM_LOG_ERR("[RAM][MX][ERR] HAL_XSPI_Init failed st=%ld", (long)st);
  }
  return st;
}
/**
  * @}
  */

/** @addtogroup STM32N6570_DK_XSPI_RAM_Exported_Functions
  * @{
  */
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief Default BSP XSPI Msp Callbacks
  * @param Instance      XSPI Instance
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hxspi_ram[Instance], HAL_XSPI_MSP_INIT_CB_ID, ((pXSPI_CallbackTypeDef) XSPI_RAM_MspInit)) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hxspi_ram[Instance],
                                       HAL_XSPI_MSP_DEINIT_CB_ID, ((pXSPI_CallbackTypeDef) XSPI_RAM_MspDeInit)) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      XSPIRam_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief BSP XSPI Msp Callback registering
  * @param Instance     XSPI Instance
  * @param CallBacks    pointer to MspInit/MspDeInit callbacks functions
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_RegisterMspCallbacks(uint32_t Instance, BSP_XSPI_Cb_t *CallBacks)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit Callbacks */
    if (HAL_XSPI_RegisterCallback(&hxspi_ram[Instance], HAL_XSPI_MSP_INIT_CB_ID, CallBacks->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_XSPI_RegisterCallback(&hxspi_ram[Instance], HAL_XSPI_MSP_DEINIT_CB_ID,
                                       CallBacks->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      XSPIRam_IsMspCbValid[Instance] = 1U;
    }
  }

  /* Return BSP status */
  return ret;
}
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Reads an amount of data from the XSPI memory.
  * @param  Instance  XSPI instance
  * @param  pData     Pointer to data to be read
  * @param  ReadAddr  Read start address
  * @param  Size      Size of data to read
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_Read(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
    /* ------- driver W958D6NBKX : read path begin ------- */
    uint32_t done = 0U;

    if ((pData == NULL) || (Size == 0U) ||
        (ReadAddr >= BSP_XSPI_RAM_SIZE_BYTES) ||
        (Size > (BSP_XSPI_RAM_SIZE_BYTES - ReadAddr)))
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else
    {
      while (done < Size)
      {
        uint32_t chunk = Size - done;
        if (chunk > BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES)
        {
          chunk = BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES;
        }

        ret = XSPI_RAM_W958_ReadMem(Instance, &pData[done], ReadAddr + done, chunk);
        if (ret != BSP_ERROR_NONE)
        {
          XSPI_RAM_LOG("[W958][ERR] ReadChunk addr=0x%08lX chunk=%lu done=%lu/%lu ret=%ld",
                       (unsigned long)(ReadAddr + done),
                       (unsigned long)chunk,
                       (unsigned long)done,
                       (unsigned long)Size,
                       (long)ret);
          break;
        }

        done += chunk;
      }
    }

    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][ERR] Read addr=0x%08lX size=%lu ret=%ld",
                   (unsigned long)ReadAddr, (unsigned long)Size, (long)ret);
    }
    /* ------- driver W958D6NBKX : read path end ------- */
#else
    if (APS256XX_Read(&hxspi_ram[Instance], pData, ReadAddr, Size,
                      BSP_XSPI_RAM_READ_LATENCY_CODE,
                      BSP_XSPI_RAM_IO_MODE,
                      BSP_XSPI_RAM_BURST_TYPE) != APS256XX_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Writes an amount of data to the XSPI memory.
  * @param  Instance  XSPI instance
  * @param  pData     Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size      Size of data to write
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_Write(uint32_t Instance, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
    /* ------- driver W958D6NBKX : write path begin ------- */
    uint32_t done = 0U;

    if ((pData == NULL) || (Size == 0U) ||
        (WriteAddr >= BSP_XSPI_RAM_SIZE_BYTES) ||
        (Size > (BSP_XSPI_RAM_SIZE_BYTES - WriteAddr)))
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else
    {
      while (done < Size)
      {
        uint32_t chunk = Size - done;
        if (chunk > BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES)
        {
          chunk = BSP_XSPI_RAM_W958_XFER_CHUNK_BYTES;
        }

        ret = XSPI_RAM_W958_WriteMem(Instance, &pData[done], WriteAddr + done, chunk);
        if (ret != BSP_ERROR_NONE)
        {
          XSPI_RAM_LOG("[W958][ERR] WriteChunk addr=0x%08lX chunk=%lu done=%lu/%lu ret=%ld",
                       (unsigned long)(WriteAddr + done),
                       (unsigned long)chunk,
                       (unsigned long)done,
                       (unsigned long)Size,
                       (long)ret);
          break;
        }

        done += chunk;
      }
    }

    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][ERR] Write addr=0x%08lX size=%lu ret=%ld",
                   (unsigned long)WriteAddr, (unsigned long)Size, (long)ret);
    }
    /* ------- driver W958D6NBKX : write path end ------- */
#else
    if (APS256XX_Write(&hxspi_ram[Instance], pData, WriteAddr, Size,
                       BSP_XSPI_RAM_WRITE_LATENCY_CODE,
                       BSP_XSPI_RAM_IO_MODE,
                       BSP_XSPI_RAM_BURST_TYPE) != APS256XX_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Configure the XSPI in memory-mapped mode
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    XSPI_RAM_W958_INIT_LOG("mmp_enter inst=%lu ctx_mode=%lu",
                           (unsigned long)Instance,
                           (unsigned long)XSPI_Ram_Ctx[Instance].IsInitialized);
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
    /* ------- driver W958D6NBKX : memory-mapped path begin ------- */
    XSPI_HyperbusCmdTypeDef sCommand = {0};
    XSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

    sCommand.AddressSpace = HAL_XSPI_MEMORY_ADDRESS_SPACE;
    sCommand.Address = 0U;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.DataLength = 2U;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;
    sCommand.DataMode = BSP_XSPI_RAM_W958_MEM_DATA_MODE;
    XSPI_RAM_LOG_CMD("[W958][CMD] MMP prepare", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                     sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);
    if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
      XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] MMP HyperbusCmd failed");
      XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] MMP regs");
      XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] MMP");
    }
    else
    {
      sMemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
      /*
       * Debug-safe MMP config:
       * disable XSPI/AXI prefetch to avoid speculative accesses masking data-integrity issues
       * while validating HyperRAM timing and cache coherency.
       */
      sMemMappedCfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_DISABLE;
      sMemMappedCfg.NoPrefetchAXI = HAL_XSPI_AXI_PREFETCH_DISABLE;
      if (HAL_XSPI_MemoryMapped(&hxspi_ram[Instance], &sMemMappedCfg) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
        XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] MMP enable failed");
        XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] MMP regs");
        XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] MMP");
      }
      else
      {
        XSPI_RAM_W958_INIT_LOG("mmp_cfg timeout=%lu prefetch_data=%lu prefetch_axi=%lu",
                               (unsigned long)sMemMappedCfg.TimeOutActivation,
                               (unsigned long)sMemMappedCfg.NoPrefetchData,
                               (unsigned long)sMemMappedCfg.NoPrefetchAXI);
      }
    }
    /* ------- driver W958D6NBKX : memory-mapped path end ------- */
#else
    if (APS256XX_EnableMemoryMappedMode(&hxspi_ram[Instance],
                                        BSP_XSPI_RAM_READ_LATENCY_CODE,
                                        BSP_XSPI_RAM_WRITE_LATENCY_CODE,
                                        BSP_XSPI_RAM_IO_MODE,
                                        BSP_XSPI_RAM_BURST_TYPE) != APS256XX_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_MMP;
    }
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */

    if (ret == BSP_ERROR_NONE)
    {
      XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_MMP;
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
      XSPI_RAM_W958_INIT_LOG("mmp_enabled");
#else
      XSPI_RAM_LOG("[APS256XX] MMP enabled");
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
    }
  }


  /* Return BSP status */
  return ret;
}

/**
  * @brief  Exit the memory-mapped mode
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_DisableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (XSPI_Ram_Ctx[Instance].IsInitialized != XSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_XSPI_MMP_UNLOCK_FAILURE;
    }
    /* Abort MMP back to indirect mode */
    else if (HAL_XSPI_Abort(&hxspi_ram[Instance]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    /* Update XSPI HyperRAM context if all operations are well done */
    else
    {
      XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Get RAM ID 3 Bytes:
  *         Vendor ID, Device ID, Device Density
  * @param  Instance  XSPI instance
  * @param  Id Pointer to RAM ID bytes
  * @retval BSP status
  */
int32_t BSP_XSPI_RAM_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
    uint16_t id0 = 0U;
    uint16_t id1 = 0U;
    int32_t reg_ret = BSP_ERROR_NONE;

    if (Id == NULL)
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else
    {
      ret = BSP_ERROR_NONE;
      XSPI_RAM_W958_REGTRACE_LOG("ReadID begin inst=%lu id0_addr=0x%08lX id1_addr=0x%08lX",
                                 (unsigned long)Instance,
                                 (unsigned long)BSP_XSPI_RAM_W958_ID0_ADDR,
                                 (unsigned long)BSP_XSPI_RAM_W958_ID1_ADDR);
      XSPI_RAM_LOG_SR_DECODE(Instance, "ReadID/begin");
      XSPI_RAM_W958_REGTRACE_LOG("ReadID step: read ID0 from 0x%08lX",
                                 (unsigned long)BSP_XSPI_RAM_W958_ID0_ADDR);
      reg_ret = XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID0_ADDR, &id0);
      if (reg_ret != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
        XSPI_RAM_LOG("[W958][ERR] ReadID failed on reg0 addr=0x%08lX", (unsigned long)BSP_XSPI_RAM_W958_ID0_ADDR);
        XSPI_RAM_W958_REGTRACE_LOG("ReadID step FAIL: ID0 read ret=%ld", (long)reg_ret);
      }
      else
      {
        XSPI_RAM_W958_REGTRACE_LOG("ReadID step OK: ID0=0x%04X", (unsigned int)id0);
      }
    }

    if (ret == BSP_ERROR_NONE)
    {
      XSPI_RAM_W958_REGTRACE_LOG("ReadID step: read ID1 from cfg addr=0x%08lX",
                                 (unsigned long)BSP_XSPI_RAM_W958_ID1_ADDR);
      reg_ret = XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID1_ADDR, &id1);
      if (reg_ret != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
        XSPI_RAM_LOG("[W958][ERR] ReadID failed on reg1 addr=0x%08lX", (unsigned long)BSP_XSPI_RAM_W958_ID1_ADDR);
        XSPI_RAM_W958_REGTRACE_LOG("ReadID step FAIL: ID1 read ret=%ld", (long)reg_ret);
      }
      else
      {
        XSPI_RAM_W958_REGTRACE_LOG("ReadID step OK: ID1=0x%04X", (unsigned int)id1);

        Id[0] = (uint8_t)(id0 & 0xFFU);
        Id[1] = (uint8_t)((id0 >> 8) & 0xFFU);
        Id[2] = (uint8_t)(id1 & 0xFFU);
        Id[3] = (uint8_t)((id1 >> 8) & 0xFFU);
        Id[4] = 0U;
        Id[5] = 0U;
        XSPI_RAM_LOG("[W958] ReadID id0=0x%04X id1=0x%04X",
                     (unsigned int)id0, (unsigned int)id1);
        XSPI_RAM_W958_LogID("W958][ID", id0, id1);
        XSPI_RAM_LOG_SR_DECODE(Instance, "ReadID/end");
        XSPI_RAM_W958_REGTRACE_LOG("ReadID done inst=%lu", (unsigned long)Instance);
        ret = BSP_ERROR_NONE;
      }
    }
#else
    if (APS256XX_ReadID(&hxspi_ram[Instance], Id, 6U) != APS256XX_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Build an RGB-color YUV422 (YUYV) test image, push it to PSRAM test buffer,
  *         then read back and verify byte-by-byte.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
int32_t BSP_XSPI_RAM_W958_TestYuv422RgbBuffer(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  int32_t mmp_ret = BSP_ERROR_NONE;
  uint8_t mmp_was_enabled = 0U;
  uint8_t run_mmp_path = 0U;
  uint32_t i;
  uint32_t mismatch = 0U;
  uint32_t first_bad = 0U;
  uint8_t exp_bad = 0U;
  uint8_t got_bad = 0U;
  const uint32_t width = BSP_XSPI_RAM_W958_TESTIMG_WIDTH;
  const uint32_t height = BSP_XSPI_RAM_W958_TESTIMG_HEIGHT;
  const uint32_t frame_bytes = BSP_XSPI_RAM_W958_TESTIMG_FRAME_BYTES;
  const uint32_t off = BSP_XSPI_RAM_W958_TESTBUF_ADDR;
  volatile uint8_t *mmp_ptr = (volatile uint8_t *)(BSP_XSPI_RAM_MMP_BASE + off);
  static uint8_t wr[BSP_XSPI_RAM_W958_TESTIMG_FRAME_BYTES];
  static uint8_t rd[BSP_XSPI_RAM_W958_TESTIMG_FRAME_BYTES];
  char wr_hex[(3U * 32U) + 1U];
  char rd_hex[(3U * 32U) + 1U];
  static const uint8_t rgb_bars[8][3] = {
    {255U,   0U,   0U}, /* red */
    {  0U, 255U,   0U}, /* green */
    {  0U,   0U, 255U}, /* blue */
    {255U, 255U,   0U}, /* yellow */
    {  0U, 255U, 255U}, /* cyan */
    {255U,   0U, 255U}, /* magenta */
    {255U, 255U, 255U}, /* white */
    {  0U,   0U,   0U}  /* black */
  };

  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((width == 0U) || (height == 0U) || ((width & 1U) != 0U))
  {
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] invalid size w=%lu h=%lu (w must be even)",
                     (unsigned long)width, (unsigned long)height);
    return BSP_ERROR_WRONG_PARAM;
  }
  if ((off + frame_bytes) > BSP_XSPI_RAM_SIZE_BYTES)
  {
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] range overflow off=0x%08lX bytes=%lu size=%lu",
                     (unsigned long)off,
                     (unsigned long)frame_bytes,
                     (unsigned long)BSP_XSPI_RAM_SIZE_BYTES);
    return BSP_ERROR_WRONG_PARAM;
  }

  if ((XSPI_Ram_Ctx[Instance].IsInitialized == XSPI_ACCESS_MMP) &&
      (BSP_XSPI_RAM_W958_YUV_TEST_USE_MMP == 1U))
  {
    run_mmp_path = 1U;
    XSPI_RAM_LOG_ERR("[W958][YUVTEST] using mapped window base=0x%08lX off=0x%08lX",
                     (unsigned long)BSP_XSPI_RAM_MMP_BASE,
                     (unsigned long)off);
  }

  /* Legacy indirect path (kept for debug switch). */
  if ((run_mmp_path == 0U) && (XSPI_Ram_Ctx[Instance].IsInitialized == XSPI_ACCESS_MMP))
  {
    mmp_was_enabled = 1U;
    mmp_ret = BSP_XSPI_RAM_DisableMemoryMappedMode(Instance);
    if (mmp_ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] disable MMP failed ret=%ld", (long)mmp_ret);
      return mmp_ret;
    }
    XSPI_RAM_LOG_ERR("[W958][YUVTEST] switched MMP->INDIRECT");
  }

  XSPI_RAM_LOG_ERR("[W958][YUVTEST] begin off=0x%08lX w=%lu h=%lu bytes=%lu",
                   (unsigned long)off,
                   (unsigned long)width,
                   (unsigned long)height,
                   (unsigned long)frame_bytes);

  /* Build YUYV from RGB bars (each pair: Y0 U Y1 V). */
  {
    uint32_t y;
    uint32_t x;
    uint32_t idx = 0U;
    for (y = 0U; y < height; y++)
    {
      for (x = 0U; x < width; x += 2U)
      {
        uint32_t bar = (x * 8U) / width;
        int r, g, b;
        int yv, uv, vv;

        if (bar > 7U)
        {
          bar = 7U;
        }

        r = (int)rgb_bars[bar][0];
        g = (int)rgb_bars[bar][1];
        b = (int)rgb_bars[bar][2];

        yv = (77 * r + 150 * g + 29 * b + 128) >> 8;
        uv = ((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128;
        vv = ((128 * r - 107 * g - 21 * b + 128) >> 8) + 128;

        if (yv < 0) { yv = 0; } else if (yv > 255) { yv = 255; }
        if (uv < 0) { uv = 0; } else if (uv > 255) { uv = 255; }
        if (vv < 0) { vv = 0; } else if (vv > 255) { vv = 255; }

        wr[idx++] = (uint8_t)yv;
        wr[idx++] = (uint8_t)uv;
        wr[idx++] = (uint8_t)yv;
        wr[idx++] = (uint8_t)vv;
      }
    }
  }

  for (i = 0U; i < frame_bytes; i++)
  {
    rd[i] = 0U;
  }

  if (run_mmp_path != 0U)
  {
#if (BSP_XSPI_RAM_W958_YUV_XCHECK_ENABLE == 1U)
    uint8_t xchk_wr[BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES];
    uint8_t xchk_rd[BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES];
    uint32_t xchk_len = (frame_bytes < BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES) ? frame_bytes : BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES;
    uint32_t xbad_i = 0U;
    uint8_t xbad_exp = 0U;
    uint8_t xbad_got = 0U;
    char xw_hex[(3U * BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES) + 1U];
    char xr_hex[(3U * BSP_XSPI_RAM_W958_YUV_XCHECK_BYTES) + 1U];
    int32_t xret;

    for (i = 0U; i < xchk_len; i++)
    {
      xchk_wr[i] = (uint8_t)(0xA5U + (i * 13U));
      xchk_rd[i] = 0U;
    }
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK] A begin: INDIRECT->MMP len=%lu", (unsigned long)xchk_len);

    xret = BSP_XSPI_RAM_DisableMemoryMappedMode(Instance);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] A disable MMP failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    xret = BSP_XSPI_RAM_Write(Instance, xchk_wr, off, xchk_len);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] A indirect write failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    xret = BSP_XSPI_RAM_EnableMemoryMappedMode(Instance);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] A re-enable MMP failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    XSPI_RAM_W958_DCacheInvalidateRange((const void *)(uintptr_t)mmp_ptr, xchk_len);
    XSPI_RAM_W958_MmpRead((const volatile uint8_t *)mmp_ptr, xchk_rd, xchk_len);

    if (memcmp(xchk_wr, xchk_rd, xchk_len) != 0)
    {
      for (i = 0U; i < xchk_len; i++)
      {
        if (xchk_wr[i] != xchk_rd[i])
        {
          xbad_i = i;
          xbad_exp = xchk_wr[i];
          xbad_got = xchk_rd[i];
          break;
        }
      }
      XSPI_RAM_W958_HexHead(xchk_wr, xchk_len, xw_hex, (uint32_t)sizeof(xw_hex));
      XSPI_RAM_W958_HexHead(xchk_rd, xchk_len, xr_hex, (uint32_t)sizeof(xr_hex));
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] A mismatch i=%lu exp=%02X got=%02X WR=%s RD=%s",
                       (unsigned long)xbad_i,
                       (unsigned int)xbad_exp,
                       (unsigned int)xbad_got,
                       xw_hex,
                       xr_hex);
      ret = BSP_ERROR_PERIPH_FAILURE;
      goto yuvtest_exit;
    }
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK] A pass");

    for (i = 0U; i < xchk_len; i++)
    {
      xchk_wr[i] = (uint8_t)(0x3CU + (i * 7U));
      xchk_rd[i] = 0U;
    }
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK] B begin: MMP->INDIRECT len=%lu", (unsigned long)xchk_len);

    XSPI_RAM_W958_MmpWrite((volatile uint8_t *)mmp_ptr, xchk_wr, xchk_len);
    XSPI_RAM_W958_DCacheCleanRange((const void *)(uintptr_t)mmp_ptr, xchk_len);
    __DSB();
    __ISB();

    xret = BSP_XSPI_RAM_DisableMemoryMappedMode(Instance);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] B disable MMP failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    xret = BSP_XSPI_RAM_Read(Instance, xchk_rd, off, xchk_len);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] B indirect read failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    xret = BSP_XSPI_RAM_EnableMemoryMappedMode(Instance);
    if (xret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] B re-enable MMP failed ret=%ld", (long)xret);
      ret = xret;
      goto yuvtest_exit;
    }

    if (memcmp(xchk_wr, xchk_rd, xchk_len) != 0)
    {
      for (i = 0U; i < xchk_len; i++)
      {
        if (xchk_wr[i] != xchk_rd[i])
        {
          xbad_i = i;
          xbad_exp = xchk_wr[i];
          xbad_got = xchk_rd[i];
          break;
        }
      }
      XSPI_RAM_W958_HexHead(xchk_wr, xchk_len, xw_hex, (uint32_t)sizeof(xw_hex));
      XSPI_RAM_W958_HexHead(xchk_rd, xchk_len, xr_hex, (uint32_t)sizeof(xr_hex));
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK][ERR] B mismatch i=%lu exp=%02X got=%02X WR=%s RD=%s",
                       (unsigned long)xbad_i,
                       (unsigned int)xbad_exp,
                       (unsigned int)xbad_got,
                       xw_hex,
                       xr_hex);
      ret = BSP_ERROR_PERIPH_FAILURE;
      goto yuvtest_exit;
    }
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][XCHK] B pass");
#endif /* (BSP_XSPI_RAM_W958_YUV_XCHECK_ENABLE == 1U) */

    XSPI_RAM_W958_MmpWrite((volatile uint8_t *)mmp_ptr, wr, frame_bytes);
    XSPI_RAM_W958_DCacheCleanRange((const void *)(uintptr_t)mmp_ptr, frame_bytes);
    __DSB();
    __ISB();
    XSPI_RAM_W958_DCacheInvalidateRange((const void *)(uintptr_t)mmp_ptr, frame_bytes);
    XSPI_RAM_W958_MmpRead((const volatile uint8_t *)mmp_ptr, rd, frame_bytes);
  }
  else
  {
    ret = BSP_XSPI_RAM_Write(Instance, wr, off, frame_bytes);
    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] write fail off=0x%08lX bytes=%lu ret=%ld",
                       (unsigned long)off, (unsigned long)frame_bytes, (long)ret);
      goto yuvtest_exit;
    }

    ret = BSP_XSPI_RAM_Read(Instance, rd, off, frame_bytes);
    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] read fail off=0x%08lX bytes=%lu ret=%ld",
                       (unsigned long)off, (unsigned long)frame_bytes, (long)ret);
      goto yuvtest_exit;
    }
  }

  for (i = 0U; i < frame_bytes; i++)
  {
    if (rd[i] != wr[i])
    {
      mismatch++;
      if (mismatch == 1U)
      {
        first_bad = i;
        exp_bad = wr[i];
        got_bad = rd[i];
      }
    }
  }

  {
    uint32_t p = 0U;
    uint32_t nbytes = (frame_bytes < 32U) ? frame_bytes : 32U;
    for (i = 0U; i < nbytes; i++)
    {
      int n = snprintf(&wr_hex[p], (size_t)(sizeof(wr_hex) - p), "%02X%s",
                       (unsigned int)wr[i], (i + 1U < nbytes) ? " " : "");
      if (n < 0) { break; }
      p += (uint32_t)n;
      if (p >= sizeof(wr_hex)) { break; }
    }
    wr_hex[sizeof(wr_hex) - 1U] = '\0';
    XSPI_RAM_LOG_ERR("[W958][YUVTEST] WR head=%s", wr_hex);
  }

  {
    uint32_t p = 0U;
    uint32_t nbytes = (frame_bytes < 32U) ? frame_bytes : 32U;
    for (i = 0U; i < nbytes; i++)
    {
      int n = snprintf(&rd_hex[p], (size_t)(sizeof(rd_hex) - p), "%02X%s",
                       (unsigned int)rd[i], (i + 1U < nbytes) ? " " : "");
      if (n < 0) { break; }
      p += (uint32_t)n;
      if (p >= sizeof(rd_hex)) { break; }
    }
    rd_hex[sizeof(rd_hex) - 1U] = '\0';
    XSPI_RAM_LOG_ERR("[W958][YUVTEST] RD head=%s", rd_hex);
  }

  if (mismatch != 0U)
  {
    XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] mismatch=%lu first_i=%lu exp=%02X got=%02X",
                     (unsigned long)mismatch,
                     (unsigned long)first_bad,
                     (unsigned int)exp_bad,
                     (unsigned int)got_bad);
    ret = BSP_ERROR_PERIPH_FAILURE;
    goto yuvtest_exit;
  }

  XSPI_RAM_LOG_ERR("[W958][YUVTEST] pass off=0x%08lX bytes=%lu",
                   (unsigned long)off,
                   (unsigned long)frame_bytes);
  ret = BSP_ERROR_NONE;

yuvtest_exit:
  if ((run_mmp_path != 0U) && (XSPI_Ram_Ctx[Instance].IsInitialized != XSPI_ACCESS_MMP))
  {
    mmp_ret = BSP_XSPI_RAM_EnableMemoryMappedMode(Instance);
    if (mmp_ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] restore MMP after xchk failed ret=%ld", (long)mmp_ret);
      if (ret == BSP_ERROR_NONE)
      {
        ret = mmp_ret;
      }
    }
    else
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST] restored to MMP");
    }
  }

  if ((mmp_was_enabled != 0U) && (run_mmp_path == 0U))
  {
    mmp_ret = BSP_XSPI_RAM_EnableMemoryMappedMode(Instance);
    if (mmp_ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST][ERR] restore MMP failed ret=%ld", (long)mmp_ret);
      if (ret == BSP_ERROR_NONE)
      {
        ret = mmp_ret;
      }
    }
    else
    {
      XSPI_RAM_LOG_ERR("[W958][YUVTEST] restored INDIRECT->MMP");
    }
  }

  return ret;
}
#endif /* (BSP_XSPI_RAM_USE_W958D6NBKX == 1U) */

/**
  * @}
  */
#endif /* #if (USE_RAM_MEMORY_APS256XX == 1) */

#if (USE_NOR_MEMORY_MX66UW1G45G == 1)
/** @addtogroup STM32N6570_DK_XSPI_NOR_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the XSPI MSP.
  * @param  hxspi XSPI handle
  * @retval None
  */
static void XSPI_NOR_MspInit(const XSPI_HandleTypeDef *hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hxspi unused argument(s) compilation warning */
  UNUSED(hxspi);

  /* Enable the XSPI memory interface clock */
  XSPI_NOR_CLK_ENABLE();

  /* Reset the XSPI memory interface */
  XSPI_NOR_FORCE_RESET();
  XSPI_NOR_RELEASE_RESET();

  /* XSPI power enable */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO3();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO3, PWR_VDDIO_RANGE_1V8);

  /* Enable GPIO clocks */
  XSPI_NOR_CLK_GPIO_CLK_ENABLE();
  XSPI_NOR_CS_GPIO_CLK_ENABLE();
  XSPI_NOR_D0_GPIO_CLK_ENABLE();
  XSPI_NOR_D1_GPIO_CLK_ENABLE();
  XSPI_NOR_D2_GPIO_CLK_ENABLE();
  XSPI_NOR_D3_GPIO_CLK_ENABLE();

  /* XSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = XSPI_NOR_CS_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_CS_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = XSPI_NOR_CLK_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_D0_PIN;
  GPIO_InitStruct.Alternate = XSPI_NOR_D0_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_D0_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_D1_PIN;
  GPIO_InitStruct.Alternate = XSPI_NOR_D1_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_D1_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_D2_PIN;
  GPIO_InitStruct.Alternate = XSPI_NOR_D2_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_D2_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_NOR_D3_PIN;
  GPIO_InitStruct.Alternate = XSPI_NOR_D3_PIN_AF;
  HAL_GPIO_Init(XSPI_NOR_D3_GPIO_PORT, &GPIO_InitStruct);

}

/**
  * @brief  De-Initializes the XSPI MSP.
  * @param  hxspi XSPI handle
  * @retval None
  */
static void XSPI_NOR_MspDeInit(const XSPI_HandleTypeDef *hxspi)
{
  /* hxspi unused argument(s) compilation warning */
  UNUSED(hxspi);

  /* XSPI GPIO pins de-configuration  */
  HAL_GPIO_DeInit(XSPI_NOR_CLK_GPIO_PORT, XSPI_NOR_CLK_PIN);
  HAL_GPIO_DeInit(XSPI_NOR_CS_GPIO_PORT, XSPI_NOR_CS_PIN);
  HAL_GPIO_DeInit(XSPI_NOR_D0_GPIO_PORT, XSPI_NOR_D0_PIN);
  HAL_GPIO_DeInit(XSPI_NOR_D1_GPIO_PORT, XSPI_NOR_D1_PIN);
  HAL_GPIO_DeInit(XSPI_NOR_D2_GPIO_PORT, XSPI_NOR_D2_PIN);
  HAL_GPIO_DeInit(XSPI_NOR_D3_GPIO_PORT, XSPI_NOR_D3_PIN);

  /* Reset the XSPI memory interface */
  XSPI_NOR_FORCE_RESET();
  XSPI_NOR_RELEASE_RESET();

  /* Disable the XSPI memory interface clock */
  XSPI_NOR_CLK_DISABLE();
}

/**
  * @brief  This function reset the XSPI memory.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
static int32_t XSPI_NOR_ResetMemory(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t reg[2], ResetRecoTime = 0U;

  /* Check first the register memory in current mode */
  if (MX66UW1G45G_ReadStatusRegister(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, reg) != MX66UW1G45G_OK)
  {
     /* Do nothing as maybe nor flash is not accessible */
  }
  /* Check the value of the register */
  else if ((reg[0] & MX66UW1G45G_SR_WIP) != 0U)
  {
    /* Enable Wait Reset Recovery Time after Reset cmd (WIP detected)*/
    ResetRecoTime = 1U;
  }
  else
  {
    /* Do nothing */
  }

  if (MX66UW1G45G_ResetEnable(&hxspi_nor[Instance], BSP_XSPI_NOR_SPI_MODE,
                                    BSP_XSPI_NOR_STR_TRANSFER) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (MX66UW1G45G_ResetMemory(&hxspi_nor[Instance], BSP_XSPI_NOR_SPI_MODE,
                                    BSP_XSPI_NOR_STR_TRANSFER) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Optional resets for boards supporting OPI: keep best-effort only. */
    (void) MX66UW1G45G_ResetEnable(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                   BSP_XSPI_NOR_STR_TRANSFER);
    (void) MX66UW1G45G_ResetMemory(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                   BSP_XSPI_NOR_STR_TRANSFER);
    (void) MX66UW1G45G_ResetEnable(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                   BSP_XSPI_NOR_DTR_TRANSFER);
    (void) MX66UW1G45G_ResetMemory(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                   BSP_XSPI_NOR_DTR_TRANSFER);

    XSPI_Nor_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;     /* After reset S/W setting to indirect access  */
    XSPI_Nor_Ctx[Instance].InterfaceMode = BSP_XSPI_NOR_SPI_MODE;    /* After reset H/W back to SPI mode by default */
    XSPI_Nor_Ctx[Instance].TransferRate  = BSP_XSPI_NOR_STR_TRANSFER; /* After reset S/W setting to STR mode        */

    /* After SWreset CMD, wait in case SWReset occurred during erase operation */
    if (ResetRecoTime == 1U)
    {
      HAL_Delay(MX66UW1G45G_RESET_MAX_TIME);
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enables the octal DTR mode of the memory.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
static int32_t XSPI_NOR_EnterDOPIMode(uint32_t Instance)
{
  int32_t ret;
  uint8_t reg[2];

  if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with new dummy cycles) */
  else if (MX66UW1G45G_WriteCfg2Register(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_CR2_REG3_ADDR,
                                          MX66UW1G45G_CR2_DC_20_CYCLES) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Enable write operations */
  else if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  else if (MX66UW1G45G_WriteCfg2Register(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_CR2_REG1_ADDR,
                                          MX66UW1G45G_CR2_DOPI) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MX66UW1G45G_WRITE_REG_MAX_TIME);

    if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                              BSP_XSPI_NOR_DTR_TRANSFER) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    /* Check the configuration has been correctly done */
    else if (MX66UW1G45G_ReadCfg2Register(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE, BSP_XSPI_NOR_DTR_TRANSFER,
                                           MX66UW1G45G_CR2_REG1_ADDR, reg) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (reg[0] != MX66UW1G45G_CR2_DOPI)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function enables the octal STR mode of the memory.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
static int32_t XSPI_NOR_EnterSOPIMode(uint32_t Instance)
{
  int32_t ret;
  uint8_t reg[2];

  if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with new dummy cycles) */
  else if (MX66UW1G45G_WriteCfg2Register(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_CR2_REG3_ADDR,
                                          MX66UW1G45G_CR2_DC_20_CYCLES) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Enable write operations */
  else if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  else if (MX66UW1G45G_WriteCfg2Register(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                          XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_CR2_REG1_ADDR,
                                          MX66UW1G45G_CR2_SOPI) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MX66UW1G45G_WRITE_REG_MAX_TIME);

    /* Check Flash busy ? */
    if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE,
                                         BSP_XSPI_NOR_STR_TRANSFER) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    /* Check the configuration has been correctly done */
    else if (MX66UW1G45G_ReadCfg2Register(&hxspi_nor[Instance], BSP_XSPI_NOR_OPI_MODE, BSP_XSPI_NOR_STR_TRANSFER,
                                           MX66UW1G45G_CR2_REG1_ADDR, reg) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (reg[0] != MX66UW1G45G_CR2_SOPI)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function disables the octal DTR or STR mode of the memory.
  * @param  Instance  XSPI instance
  * @retval BSP status
  */
static int32_t XSPI_NOR_ExitOPIMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t reg[2];

  if (MX66UW1G45G_WriteEnable(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                    XSPI_Nor_Ctx[Instance].TransferRate) != MX66UW1G45G_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    /* Write Configuration register 2 (with SPI protocol) */
    reg[0] = 0;
    reg[1] = 0;
    if (MX66UW1G45G_WriteCfg2Register(&hxspi_nor[Instance], XSPI_Nor_Ctx[Instance].InterfaceMode,
                                       XSPI_Nor_Ctx[Instance].TransferRate, MX66UW1G45G_CR2_REG1_ADDR,
                                       reg[0]) != MX66UW1G45G_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Wait that the configuration is effective and check that memory is ready */
      HAL_Delay(MX66UW1G45G_WRITE_REG_MAX_TIME);

      if (XSPI_Nor_Ctx[Instance].TransferRate == BSP_XSPI_NOR_DTR_TRANSFER)
      {
        /* Reconfigure the memory type of the peripheral */
        hxspi_nor[Instance].Init.MemoryType            = HAL_XSPI_MEMTYPE_MICRON;
        hxspi_nor[Instance].Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
        if (HAL_XSPI_Init(&hxspi_nor[Instance]) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        /* Check Flash busy ? */
        if (MX66UW1G45G_AutoPollingMemReady(&hxspi_nor[Instance], BSP_XSPI_NOR_SPI_MODE,
                                             BSP_XSPI_NOR_STR_TRANSFER) != MX66UW1G45G_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        /* Check the configuration has been correctly done */
        else if (MX66UW1G45G_ReadCfg2Register(&hxspi_nor[Instance], BSP_XSPI_NOR_SPI_MODE, BSP_XSPI_NOR_STR_TRANSFER,
                                               MX66UW1G45G_CR2_REG1_ADDR, reg) != MX66UW1G45G_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else if (reg[0] != 0U)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          /* Nothing to do */
        }
      }
    }
  }

  /* Return BSP status */
  return ret;
}


/**
  * @}
  */
#endif /* #if (USE_NOR_MEMORY_MX66UW1G45G == 1) */

#if (USE_RAM_MEMORY_APS256XX == 1)
/** @addtogroup STM32N6570_DK_XSPI_RAM_Private_Functions
  * @{
  */
/**
  * @brief  Initializes the XSPI MSP.
  * @param  hxspi XSPI handle
  * @retval None
  */
static void XSPI_RAM_MspInit(const XSPI_HandleTypeDef *hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hxspi unused argument(s) compilation warning */
  UNUSED(hxspi);
  XSPI_RAM_LOG("[RAM][MSP] init begin");
  XSPI_RAM_LOG("[RAM][MSP] pinmap CS=O0 DQS0=O2 DQS1=O3 CLK=O4 CLK_N=O5 DQ0..DQ15=P0..P15 AF=AF9_XSPIM_P1");

 /* XSPI power enable */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  HAL_PWREx_ConfigVddIORange(PWR_VDDIO2, PWR_VDDIO_RANGE_1V8);

  /* Enable the XSPI memory interface clock */
  XSPI_RAM_CLK_ENABLE();

  /* Reset the XSPI memory interface */
  XSPI_RAM_FORCE_RESET();
  XSPI_RAM_RELEASE_RESET();

  /* Enable and reset XSPI I/O Manager */
  __HAL_RCC_XSPIM_CLK_ENABLE();
  __HAL_RCC_XSPIM_FORCE_RESET();
  __HAL_RCC_XSPIM_RELEASE_RESET();

  /* Enable GPIO clocks */
  XSPI_RAM_CLK_GPIO_CLK_ENABLE();
  XSPI_RAM_CLK_N_GPIO_CLK_ENABLE();
  XSPI_RAM_DQS_GPIO_CLK_ENABLE();
  XSPI_RAM_CS_GPIO_CLK_ENABLE();
  XSPI_RAM_D0_GPIO_CLK_ENABLE();
  XSPI_RAM_D1_GPIO_CLK_ENABLE();
  XSPI_RAM_D2_GPIO_CLK_ENABLE();
  XSPI_RAM_D3_GPIO_CLK_ENABLE();
  XSPI_RAM_D4_GPIO_CLK_ENABLE();
  XSPI_RAM_D5_GPIO_CLK_ENABLE();
  XSPI_RAM_D6_GPIO_CLK_ENABLE();
  XSPI_RAM_D7_GPIO_CLK_ENABLE();

  /* XSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = XSPI_RAM_CS_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_CS_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI DQS0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_DQS0_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_DQS0_PIN_AF;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(XSPI_RAM_DQS0_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI DQS1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_DQS1_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_DQS1_PIN_AF;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(XSPI_RAM_DQS1_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = XSPI_RAM_CLK_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI CLK_N GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_CLK_N_PIN;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = XSPI_RAM_CLK_N_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_CLK_N_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D0_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D0_PIN_AF;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  HAL_GPIO_Init(XSPI_RAM_D0_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D1_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D1_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D1_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D2_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D2_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D2_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D3_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D3_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D3_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D4 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D4_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D4_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D4_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D5 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D5_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D5_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D5_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D6 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D6_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D6_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D6_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D7 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D7_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D7_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D7_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D8 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D8_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D8_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D8_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D9 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D9_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D9_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D9_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D10 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D10_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D10_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D10_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D11 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D11_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D11_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D11_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D12 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D12_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D12_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D12_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D13 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D13_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D13_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D13_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D14 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D14_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D14_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D14_GPIO_PORT, &GPIO_InitStruct);

  /* XSPI D15 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = XSPI_RAM_D15_PIN;
  GPIO_InitStruct.Alternate = XSPI_RAM_D15_PIN_AF;
  HAL_GPIO_Init(XSPI_RAM_D15_GPIO_PORT, &GPIO_InitStruct);
  XSPI_RAM_LOG("[RAM][MSP] init done");
}

/**
  * @brief  De-Initializes the XSPI MSP.
  * @param  hxspi XSPI handle
  * @retval None
  */
static void XSPI_RAM_MspDeInit(const XSPI_HandleTypeDef *hxspi)
{
  /* hxspi unused argument(s) compilation warning */
  UNUSED(hxspi);

  /* XSPI GPIO pins de-configuration  */
  HAL_GPIO_DeInit(XSPI_RAM_CLK_GPIO_PORT, XSPI_RAM_CLK_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_CLK_N_GPIO_PORT, XSPI_RAM_CLK_N_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_DQS0_GPIO_PORT, XSPI_RAM_DQS0_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_DQS1_GPIO_PORT, XSPI_RAM_DQS1_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_CS_GPIO_PORT, XSPI_RAM_CS_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D0_GPIO_PORT, XSPI_RAM_D0_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D1_GPIO_PORT, XSPI_RAM_D1_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D2_GPIO_PORT, XSPI_RAM_D2_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D3_GPIO_PORT, XSPI_RAM_D3_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D4_GPIO_PORT, XSPI_RAM_D4_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D5_GPIO_PORT, XSPI_RAM_D5_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D6_GPIO_PORT, XSPI_RAM_D6_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D7_GPIO_PORT, XSPI_RAM_D7_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D8_GPIO_PORT, XSPI_RAM_D8_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D9_GPIO_PORT, XSPI_RAM_D9_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D10_GPIO_PORT, XSPI_RAM_D10_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D11_GPIO_PORT, XSPI_RAM_D11_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D12_GPIO_PORT, XSPI_RAM_D12_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D13_GPIO_PORT, XSPI_RAM_D13_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D14_GPIO_PORT, XSPI_RAM_D14_PIN);
  HAL_GPIO_DeInit(XSPI_RAM_D15_GPIO_PORT, XSPI_RAM_D15_PIN);

  /* Reset the XSPI memory interface */
  XSPI_RAM_FORCE_RESET();
  XSPI_RAM_RELEASE_RESET();

  /* Disable the XSPI memory interface clock */
  XSPI_RAM_CLK_DISABLE();
}
#endif /* #if (USE_RAM_MEMORY_APS256XX == 1) */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
