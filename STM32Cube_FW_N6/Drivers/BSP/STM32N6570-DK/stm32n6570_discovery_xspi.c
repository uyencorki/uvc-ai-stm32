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

#ifndef BSP_XSPI_RAM_DEBUG_LOG_ENABLE
#define BSP_XSPI_RAM_DEBUG_LOG_ENABLE 1U
#endif

#if (BSP_XSPI_RAM_DEBUG_LOG_ENABLE == 1U)
#define XSPI_RAM_LOG(fmt, ...) printf("[XSPI-RAM] " fmt "\r\n", ##__VA_ARGS__)
#else
#define XSPI_RAM_LOG(fmt, ...)
#endif

#define XSPI_RAM_LOG_HAL_ERR(inst, tag) \
  XSPI_RAM_LOG("%s HAL state=%lu err=0x%08lX SR=0x%08lX", \
               (tag), \
               (unsigned long)HAL_XSPI_GetState(&hxspi_ram[(inst)]), \
               (unsigned long)HAL_XSPI_GetError(&hxspi_ram[(inst)]), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->SR))

#define XSPI_RAM_LOG_REGS(inst, tag) \
  XSPI_RAM_LOG("%s CR=0x%08lX DCR1=0x%08lX HLCR=0x%08lX CCR=0x%08lX WCCR=0x%08lX AR=0x%08lX DLR=%lu", \
               (tag), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->CR), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->DCR1), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->HLCR), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->CCR), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->WCCR), \
               (unsigned long)READ_REG(hxspi_ram[(inst)].Instance->AR), \
               (unsigned long)(READ_REG(hxspi_ram[(inst)].Instance->DLR) + 1U))

#define XSPI_RAM_LOG_CMD(tag, aspace, addr, aw, len, dqs, dmode) \
  XSPI_RAM_LOG("%s aspace=%lu addr=0x%08lX aw=%lu len=%lu dqs=%lu dmode=%lu", \
               (tag), \
               (unsigned long)(aspace), \
               (unsigned long)(addr), \
               (unsigned long)(aw), \
               (unsigned long)(len), \
               (unsigned long)(dqs), \
               (unsigned long)(dmode))

#define XSPI_RAM_TRY_ABORT(inst, tag) \
  do { \
    CLEAR_BIT(hxspi_ram[(inst)].Instance->CR, XSPI_CR_FMODE); \
    hxspi_ram[(inst)].State = HAL_XSPI_STATE_READY; \
    XSPI_RAM_LOG("%s soft-recover to READY", (tag)); \
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
static int32_t XSPI_RAM_W958_WriteReg(uint32_t Instance, uint32_t RegIndex, uint16_t Value);
static int32_t XSPI_RAM_W958_ReadReg(uint32_t Instance, uint32_t RegIndex, uint16_t *Value);
static int32_t XSPI_RAM_W958_ReadMem(uint32_t Instance, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
static int32_t XSPI_RAM_W958_WriteMem(uint32_t Instance, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
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
  sCommand.DQSMode = BSP_XSPI_RAM_W958_WRITE_DQS_MODE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_REG_DATA_MODE;

  XSPI_RAM_LOG_CMD("[W958][CMD] WriteReg", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteReg HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Transmit(&hxspi_ram[Instance], wr, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteReg TX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }

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
  sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;
  sCommand.DataMode = BSP_XSPI_RAM_W958_REG_DATA_MODE;

  XSPI_RAM_LOG_CMD("[W958][CMD] ReadReg", sCommand.AddressSpace, sCommand.Address, sCommand.AddressWidth,
                   sCommand.DataLength, sCommand.DQSMode, sCommand.DataMode);

  if (HAL_XSPI_HyperbusCmd(&hxspi_ram[Instance], &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadReg HyperbusCmd failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Receive(&hxspi_ram[Instance], rd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadReg RX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadReg regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadReg");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  *Value = ((uint16_t)rd[1] << 8) | rd[0];
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
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Receive(&hxspi_ram[Instance], pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] ReadMem RX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] ReadMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] ReadMem");
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
    return BSP_ERROR_PERIPH_FAILURE;
  }
  if (HAL_XSPI_Transmit(&hxspi_ram[Instance], pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] WriteMem TX failed");
    XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] WriteMem regs");
    XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] WriteMem");
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return BSP_ERROR_NONE;
}
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

  /* Check if the instance is supported */
  if (Instance >= XSPI_RAM_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (XSPI_Ram_Ctx[Instance].IsInitialized == XSPI_ACCESS_NONE)
    {
#if (USE_HAL_XSPI_REGISTER_CALLBACKS == 0)
      /* Msp XSPI initialization */
      XSPI_RAM_MspInit(&hxspi_ram[Instance]);
#else
      /* Register the XSPI MSP Callbacks */
      if (XSPIRam_IsMspCbValid[Instance] == 0UL)
      {
        if (BSP_XSPI_RAM_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

      /* Fill config structure */
      xspi_init.ClockPrescaler = 3;
      xspi_init.MemorySize     = HAL_XSPI_SIZE_256MB;
      xspi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;

      /* STM32 XSPI interface initialization */
      if (MX_XSPI_RAM_Init(&hxspi_ram[Instance], &xspi_init) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        /* Update current status parameter */
        XSPI_Ram_Ctx[Instance].IsInitialized = XSPI_ACCESS_INDIRECT;
        XSPI_Ram_Ctx[Instance].LatencyType   = BSP_XSPI_RAM_FIXED_LATENCY;
        XSPI_Ram_Ctx[Instance].BurstType     = BSP_XSPI_RAM_LINEAR_BURST;
      }
    }

    if (ret == BSP_ERROR_NONE)
    {
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
      XSPI_HyperbusCfgTypeDef hb_cfg = {0};
      XSPI_RAM_LOG("[W958] init path selected");
      XSPI_RAM_LOG("[W958] map ID0=0x%08lX ID1=0x%08lX CR0=0x%08lX CR1=0x%08lX",
                   (unsigned long)BSP_XSPI_RAM_W958_ID0_ADDR,
                   (unsigned long)BSP_XSPI_RAM_W958_ID1_ADDR,
                   (unsigned long)BSP_XSPI_RAM_W958_CR0_ADDR,
                   (unsigned long)BSP_XSPI_RAM_W958_CR1_ADDR);

      hb_cfg.RWRecoveryTimeCycle = BSP_XSPI_RAM_W958_RW_RECOVERY_CYCLES;
      hb_cfg.AccessTimeCycle = BSP_XSPI_RAM_W958_ACCESS_CYCLES;
      hb_cfg.WriteZeroLatency = HAL_XSPI_LATENCY_ON_WRITE;
      hb_cfg.LatencyMode = HAL_XSPI_FIXED_LATENCY;
      XSPI_RAM_LOG("[W958] hb_cfg trwr=%lu tacc=%lu write_lat=%lu lat_mode=%lu reg_dmode=%lu mem_dmode=%lu wr_dqs=%lu post_ps=%lu",
                   (unsigned long)hb_cfg.RWRecoveryTimeCycle,
                   (unsigned long)hb_cfg.AccessTimeCycle,
                   (unsigned long)hb_cfg.WriteZeroLatency,
                   (unsigned long)hb_cfg.LatencyMode,
                   (unsigned long)BSP_XSPI_RAM_W958_REG_DATA_MODE,
                   (unsigned long)BSP_XSPI_RAM_W958_MEM_DATA_MODE,
                   (unsigned long)BSP_XSPI_RAM_W958_WRITE_DQS_MODE,
                   (unsigned long)BSP_XSPI_RAM_W958_POST_INIT_PRESCALER);
      if (HAL_XSPI_HyperbusCfg(&hxspi_ram[Instance], &hb_cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
        XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] HyperbusCfg failed");
        XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] init regs");
      }
      else
      {
        XSPI_RAM_LOG_REGS(Instance, "[W958] init regs");
      }

#if (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U)
      if (ret == BSP_ERROR_NONE)
      {
        if (XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, BSP_XSPI_RAM_W958_CR0_INIT) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else if (XSPI_RAM_W958_WriteReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, BSP_XSPI_RAM_W958_CR1_INIT) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
      }
#endif /* (BSP_XSPI_RAM_SKIP_VENDOR_REG_INIT == 0U) */

      if ((ret == BSP_ERROR_NONE) &&
          (HAL_XSPI_SetClockPrescaler(&hxspi_ram[Instance], BSP_XSPI_RAM_W958_POST_INIT_PRESCALER) != HAL_OK))
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
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

  return HAL_XSPI_Init(hxspi);
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
    ret = XSPI_RAM_W958_ReadMem(Instance, pData, ReadAddr, Size);
    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][ERR] Read addr=0x%08lX size=%lu ret=%ld",
                   (unsigned long)ReadAddr, (unsigned long)Size, (long)ret);
    }
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
    ret = XSPI_RAM_W958_WriteMem(Instance, pData, WriteAddr, Size);
    if (ret != BSP_ERROR_NONE)
    {
      XSPI_RAM_LOG("[W958][ERR] Write addr=0x%08lX size=%lu ret=%ld",
                   (unsigned long)WriteAddr, (unsigned long)Size, (long)ret);
    }
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
#if (BSP_XSPI_RAM_USE_W958D6NBKX == 1U)
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
      sMemMappedCfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
      sMemMappedCfg.NoPrefetchAXI = HAL_XSPI_AXI_PREFETCH_ENABLE;
      if (HAL_XSPI_MemoryMapped(&hxspi_ram[Instance], &sMemMappedCfg) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
        XSPI_RAM_LOG_HAL_ERR(Instance, "[W958][ERR] MMP enable failed");
        XSPI_RAM_LOG_REGS(Instance, "[W958][ERR] MMP regs");
        XSPI_RAM_TRY_ABORT(Instance, "[W958][ERR] MMP");
      }
    }
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
      XSPI_RAM_LOG("[W958] MMP enabled");
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
  int32_t ret;

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
    uint16_t cr0 = 0U;
    uint16_t cr1 = 0U;
    uint32_t alt_id1_addr;
    int32_t reg_ret;

    if (Id == NULL)
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else if (XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID0_ADDR, &id0) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
      XSPI_RAM_LOG("[W958][ERR] ReadID failed on reg0");
    }
    else
    {
      alt_id1_addr = (BSP_XSPI_RAM_W958_ID1_ADDR == 0x00000001U) ? 0x00000002U : 0x00000001U;
      reg_ret = XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_ID1_ADDR, &id1);
      if (reg_ret != BSP_ERROR_NONE)
      {
        XSPI_RAM_LOG("[W958][WARN] ID1 read failed at cfg addr=0x%08lX, try alt=0x%08lX",
                     (unsigned long)BSP_XSPI_RAM_W958_ID1_ADDR,
                     (unsigned long)alt_id1_addr);
        reg_ret = XSPI_RAM_W958_ReadReg(Instance, alt_id1_addr, &id1);
      }

      if (reg_ret != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
        XSPI_RAM_LOG("[W958][ERR] ReadID failed on reg1/alt");
      }
      else
      {
        /* Extra probes for debugging register map and CR readability */
        (void)XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR0_ADDR, &cr0);
        (void)XSPI_RAM_W958_ReadReg(Instance, BSP_XSPI_RAM_W958_CR1_ADDR, &cr1);

        Id[0] = (uint8_t)(id0 & 0xFFU);
        Id[1] = (uint8_t)((id0 >> 8) & 0xFFU);
        Id[2] = (uint8_t)(id1 & 0xFFU);
        Id[3] = (uint8_t)((id1 >> 8) & 0xFFU);
        Id[4] = 0U;
        Id[5] = 0U;
        XSPI_RAM_LOG("[W958] ReadID id0=0x%04X id1=0x%04X cr0=0x%04X cr1=0x%04X",
                     (unsigned int)id0, (unsigned int)id1, (unsigned int)cr0, (unsigned int)cr1);
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
