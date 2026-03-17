del /f /q Project_sign.bin 2>nul

"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_SigningTool_CLI.exe" -bin x-cube-n6-ai-h264-usb-uvc.bin -nk -t ssbl -hv 2.3 -o Project_sign.bin -dump Project_sign.bin -align

del /f /q "D:\Subway\FLASH\Project_sign.bin"

copy /Y Project_sign.bin "D:\Subway\FLASH"