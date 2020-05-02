# STM32F10x TKG-HID-BOOTLOADER

BETA VERSION !!!!


This is a HID (Human Interface Device) driverless booloader for the STM32F10x family line, including the famous Bluepill.  It was largely inspired from https://github.com/Serasidis/STM32_HID_Bootloader from Vassilis Serasidis (respect to him !).  
This project is not really a fork anymore due to a lot of enhancements, code optimizations and bug corrections.  

The bootloder **supports transparently low-medium and high density devices without recompilation**. 
As the STM32_HID_Bootloader project, tkg-hid-bootloader doesn't use any ST libraries, but only CMSIS from the ST SDK. So, the bootloader size is under 4 Kbytes, allowing more space for user programs (user flash memory starts at 0x08001000).

The TKG-FLASH has many new features, like dump, simulation mode, progression bar, etc... and compatibilty with the STM32DUINO platform, and original HID-FLASH CLI is preserved.

Latest version of the GCC ARM toolchain is recommended for building the bootloader.

# Entering the bootloader

* Automatic when flashing with TKG-FLASH if the right com port is given to toggle DTR
* BOOT1 set to HIGH (permanent with jumper set)
* Writing the value **0x424C** in the DR4 backup register (ex : from you own firmware before rebooting) 
* Double push on the reset button

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

# Adding a new upload method to the Arduino platform
(todo)
Exemple for an STM32103RCB
``````
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K=TKG-HID bootloader
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.upload.tool=tkg-hid
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.upload_flags=-DSERIAL_USB -DGENERIC_BOOTLOADER
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.vect=VECT_TAB_ADDR=0x8001000
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.ldscript=ld/hid_bootloader_tkg_cb.ld
``````
You must also edit the linker script file mentioned in the upload method (ex above is hid_bootloader_tkg_cb.ld) and adjust the ram and rom origin and lengths. Ram origin start at 0x20000000 is the length is the one of your MCU (20K for the STM32103CB). Rom origin is 0x08000000 + TKG bootloader size (4K = 0x1000).  The flash memory size is the MCU one minus the TKG bootloader size. 

``````
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx)  : ORIGIN = 0x08001000, LENGTH = 124K
}

/* Provide memory region aliases for common.inc */
REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom);

/* Let common.inc handle the real work. */
INCLUDE common.inc
``````
