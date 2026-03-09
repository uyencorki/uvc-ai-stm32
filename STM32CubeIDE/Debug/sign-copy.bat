del /f /q Project_sign.bin 2>nul

"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_SigningTool_CLI.exe" -bin x-cube-n6-ai-h264-usb-uvc.bin -nk -t ssbl -hv 2.3 -o Project_sign.bin -dump Project_sign.bin -align

del "D:\STM32-N6\Work-space\STM32CubeN6-main\STM32CubeN6-main\Projects\STM32N6570-DK\Applications\OpenBootloader\Binaries\NOR_Binary\ai-h264-uvc\Project_sign.bin"

copy Project_sign.bin "D:\STM32-N6\Work-space\STM32CubeN6-main\STM32CubeN6-main\Projects\STM32N6570-DK\Applications\OpenBootloader\Binaries\NOR_Binary\ai-h264-uvc\"
