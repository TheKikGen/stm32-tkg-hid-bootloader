# STM32F10x TKG-HID-BOOTLOADER

This is a HID (Human Interface Device) driverless booloader for the STM32F10x family line, including the famous Bluepill.  It was largely inspired from https://github.com/Serasidis/STM32_HID_Bootloader from Vassilis Serasidis (respect to him !).  
This project is not really a fork anymore due to a lot of enhancements, code optimizations and bug corrections.  

The bootloder **supports transparently low-medium and high density devices without recompilation**. As the STM32_HID_Bootloader project, tkg-hid-bootloader doesn't use any ST libraries, but only CMSIS from the ST SDK. 
The bootloader size is under 2 Kbytes, allowing more space for user programs (user flash memory starts at 0x08000800).

The TKG-FLASH has many new features, like dump, simulation mode, progression bar, etc... and compatibilty with the STM32DUINO platform, and original HID-FLASH CLI is preserved.

# Entering the bootloader

* Automatic when flashing with TKG-FLASH if the right com port is given to toggle DTR
* BOOT1 set to HIGH (permanent with jumper set)
* Writing the value **0x424C** in the DR4 backup register (ex : from you own firmware before rebooting) 

# TKG-FLASH
``````
+-----------------------------------------------------------------------+
|           TKG-Flash v2.2.1 STM32F103 HID Bootloader Flash Tool        |
|                     High density device support.                      |
+-----------------------------------------------------------------------+
|   (c)      2020 - The KikGen Labs     https://github.com/TheKikGen    |
|   (c)      2018 - Bruno Freitas       http://www.brunofreitas.com     |
|   (c) 2018-2019 - Vassilis Serasidis  https://www.serasidis.gr        |
|  Customized for STM32duino ecosystem  https://www.stm32duino.com      |
+-----------------------------------------------------------------------+

  Usage: tkg-flash <firmware file name> [<options>]

  Options are :
  -d=16 -d=32   : hexa dump sectors by 16 or 32 bytes line length.
  -p=<com port> : serial com port used to toggle DTR for MCU reset.
  -s            : flashing simulation.
  -w=<time s>   : HID device waiting time (10s default).

  Examples :
  tkg-flash myfirmare.bin -p=COM4 -w=30
  tkg-flash myfirmare.bin -d=16 -s
``````



The TKG-FLASH tool can be considered as a new version of HID-FLASH 2.2.  The following features were added :
* Permanent flashing capability if com port passed in the command line
* Waiting time parameter to wait HID device to be ready
* Flashing simulation : same behaviour but not writes at all to the flash memory
* Dump file feature

Latest version of the GCC ARM toolchain is recommended for building the bootloader.

