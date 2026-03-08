################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Subway/x-cube-n6-ai-h264-usb-uvc-main/STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.c 

OBJS += \
./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.o 

C_DEPS += \
./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.o: D:/Subway/x-cube-n6-ai-h264-usb-uvc-main/STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.c STM32Cube_FW_N6/Utilities/lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DUSE_VD66GY_SENSOR -DUSE_VD55G1_SENSOR -DUSE_VD1943_SENSOR -DTX_MAX_PARALLEL_NETWORKS=1 -DLL_ATON_PLATFORM=LL_ATON_PLAT_STM32N6 -DLL_ATON_OSAL=LL_ATON_OSAL_FREERTOS -DLL_ATON_RT_MODE=LL_ATON_RT_ASYNC -DLL_ATON_SW_FALLBACK -DLL_ATON_DBG_BUFFER_INFO_EXCLUDED=1 -DAPP_HAS_PARALLEL_NETWORKS=0 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../Inc -I../../Model -I../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../STM32Cube_FW_N6/Utilities/Fonts -I../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../Lib/AI_Runtime/Inc -I../../Lib/AI_Runtime/Npu/ll_aton -I../../Lib/AI_Runtime/Npu/Devices/STM32N6xx -I../../Lib/lib_vision_models_pp/lib_vision_models_pp/Inc -I../../Lib/ai-postprocessing-wrapper -I../../Lib/Camera_Middleware -I../../Lib/Camera_Middleware/sensors -I../../Lib/Camera_Middleware/sensors/imx335 -I../../Lib/Camera_Middleware/sensors/vd55g1 -I../../Lib/Camera_Middleware/sensors/vd6g -I../../Lib/Camera_Middleware/sensors/vd1943 -I../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../Lib/Camera_Middleware/ISP_Library/evision/Inc -I../../Lib/FreeRTOS/Source/include -I../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../STM32Cube_FW_N6/Middlewares/ST/VideoEncoder_EWL -I../../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/inc -I../../STM32Cube_FW_N6/Middlewares/Third_Party/VideoEncoder/source/common -I../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/core/inc -I../../STM32Cube_FW_N6/Middlewares/ST/usbx/ports/generic/inc -I../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_device_classes/inc -I../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../../Lib/uvcl/Inc/usbx -I../../Lib/uvcl/Src/usbx -I../../Lib/uvcl/Inc -I../../Lib/uvcl/Src -Os -ffunction-sections -fdata-sections -Wall -std=c11 -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32Cube_FW_N6-2f-Utilities-2f-lcd

clean-STM32Cube_FW_N6-2f-Utilities-2f-lcd:
	-$(RM) ./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.cyclo ./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.d ./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.o ./STM32Cube_FW_N6/Utilities/lcd/stm32_lcd.su

.PHONY: clean-STM32Cube_FW_N6-2f-Utilities-2f-lcd

