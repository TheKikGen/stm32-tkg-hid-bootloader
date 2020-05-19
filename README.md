News : First release with binaries ready to use for Windows and Linux platforms here :   
https://github.com/TheKikGen/stm32-tkg-hid-bootloader/releases/tag/V2.3

# STM32F10x TKG-HID-BOOTLOADER

This is a HID (Human Interface Device) driverless booloader for the STM32F10x family line, including the famous Bluepill.  It was largely inspired from https://github.com/Serasidis/STM32_HID_Bootloader from Vassilis Serasidis (respect to him !).  
This project is not really a fork anymore due to a lot of enhancements, code optimizations and bug corrections.

The bootloder **supports transparently low-medium and high density devices without recompilation**. 
As the STM32_HID_Bootloader project, tkg-hid-bootloader doesn't use any ST libraries, but only CMSIS from the ST SDK. So, the bootloader size is under 4 Kbytes, allowing more space for user programs (user flash memory starts at 0x08001000).

It was not possible (reasonable) to keep the size under the 2K, because before jumping to the user application, the bootloader must ensure that the current state of the MCU is clean. More, depending on your own hardware configuration, the size can vary with GPIO settings. So 4K is a good compromize between size and code quality, and will allow new features !

The TKG-FLASH has many new features, like dump, simulation mode, progression bar, etc... and compatibilty with the STM32DUINO platform, and original HID-FLASH CLI is preserved.

Latest version of the GCC ARM toolchain is recommended for building the bootloader.

# Entering the bootloader

* "Double push" on the reset button
* Automatic when flashing with TKG-FLASH if the right com port is given to toggle DTR (not always reliable)
* BOOT1 set to HIGH (permanent with jumper set fro ultimate case !)
* Writing the value **0x424C** in the DR4 backup register (ex : from you own firmware before rebooting) 


# TKG-FLASH
``````
+-----------------------------------------------------------------------+
|           TKG-Flash v2.2.1 STM32F103 HID Bootloader Flash Tool        |
|                     High density device support.                      |
|   (c)      2020 - The KikGen Labs     https://github.com/TheKikGen    |
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
* Flashing simulation : same behaviour but no writes at all to the flash memory
* Dump file feature

# Adding a new upload method to the Arduino platform (short description)

First, you must add a new upload method in board.txt (usually located at ..\Arduino\hardware\Arduino_STM32\STM32F1

Exemple for a STM32103C Bluepill :
``````
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K=TKG-HID bootloader
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.upload.tool=tkg-hid
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.upload_flags=-DSERIAL_USB -DGENERIC_BOOTLOADER
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.vect=VECT_TAB_ADDR=0x8001000
genericSTM32F103C.menu.upload_method.HIDUploadMethod2K.build.ldscript=ld/hid_bootloader_tkg_cb.ld
``````
You must also edit the linker script file mentioned in the upload method (ex above is hid_bootloader_tkg_cb.ld) in the (...)\STM32F1\variants\generic_stm32f103c\ld and adjust the ram and rom origin and lengths. Ram origin start at 0x20000000. The length is the RAM size of your MCU (20K for the STM32103CB). Rom origin is the flash memory available , starts at 0x08000000 + TKG bootloader size (4K = 0x1000).  The flash memory size is the MCU one minus the TKG bootloader size. 

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
You must then add an upload method in platform.txt :

    # TKG-HID upload 2.2.2
    tools.tkg_hid_upload.cmd=tkg_hid_upload
    tools.tkg_hid_upload.cmd.windows=tkg-flash.exe
    tools.tkg_hid_upload.cmd.macosx=tkg_flash
    tools.tkg_hid_upload.path={runtime.hardware.path}/tools/win
    tools.tkg_hid_upload.path.macosx={runtime.hardware.path}/tools/macosx
    tools.tkg_hid_upload.path.linux={runtime.hardware.path}/tools/linux
    tools.tkg_hid_upload.path.linux64={runtime.hardware.path}/tools/linux64
    tools.tkg_hid_upload.upload.params.verbose=-d
    tools.tkg_hid_upload.upload.params.quiet=n
    tools.tkg_hid_upload.upload.pattern="{path}/{cmd}" "{build.path}/{build.project_name}.bin" -p={serial.port.file} -ide

and copy the tkg-flash tool in the Arduino\hardware\Arduino_STM32\tool\win .
You need to restart the Arduino IDE to see your changes.

Note that the "HID Booloader 2.0" (in the Arduino IDE existing upload methods) can be used to flash your firmware with TKG-HID-BOOTLOADER. Obviously, better to do that with the companion CLI tool TKG-FLASH...
