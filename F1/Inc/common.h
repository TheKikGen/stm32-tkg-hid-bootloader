/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
* TKG HID Bootloader - USB HID bootloader for STM32F10X family MCU
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the TKG-HID Bootloader distribution
  https://github.com/TheKikGen/stm32-tkg-hid-bootloader
  Copyright (c) 2020 TheKikGen Labs team.

  Initial inspiration and parts of this project comes from the STM32 HID Bootloader,
  from Bruno Freitas and Vassilis Serasidis <avrsite@yahoo.gr>,
	Modified January 2019, 	by Michel Stempin <michel.stempin@wanadoo.fr>

  -----------------------------------------------------------------------------
  Disclaimer.

  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/
  or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

  NON COMMERCIAL - PERSONAL USE ONLY : You may not use the material for pure
  commercial closed code solution without the licensor permission.

  You are free to copy and redistribute the material in any medium or format,
  adapt, transform, and build upon the material.

  You must give appropriate credit, a link to the github site
  https://github.com/TheKikGen/stm32-tkg-hid-bootloader , provide a link to the license,
  and indicate if changes were made. You may do so in any reasonable manner,
  but not in any way that suggests the licensor endorses you or your use.

  You may not apply legal terms or technological measures that legally restrict
  others from doing anything the license permits.

  You do not have to comply with the license for elements of the material
  in the public domain or where your use is permitted by an applicable exception
  or limitation.

  No warranties are given. The license may not give you all of the permissions
  necessary for your intended use.  This program is distributed in the hope that
  it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/


#ifndef COMMON_H_
#define COMMON_H_

// Memory addresses common for all the STM32F103 family

/*!< FLASH base address in the alias region */
#define FLASH_BASE_ADDR            ((uint32_t)0x08000000)

/*!< SRAM base address in the alias region */
#define SRAM_BASE_ADDR             ((uint32_t)0x20000000)

/* Bootloader nb pages size in high density (2048 bytes)*/
#define BOOTLOADER_PAGE_SIZE_H		2

/* minimal SRAM size for the bootloader */
#define SRAM_SIZE			(8 * 1024)

/* SRAM end (bottom of stack) */
#define SRAM_END			(SRAM_BASE_ADDR + SRAM_SIZE)

// User code memory start address .
#define USER_ADDR	(FLASH_BASE_ADDR + BOOTLOADER_PAGE_SIZE_H * 2048)

// Magic word to enter the bootloader
#define MAGIC_WORD1 0x424C

// macro to check if user code present in flash memeory
#define CHECK_USER_CODE(addr) ( ( (*(volatile uint32_t *) addr ) & 0x2FFE0000 ) != SRAM_BASE )

// macro to detect a high density device from the flash memory size
#define IS_HIGH_DENSITY (*(uint16_t *)0x1FFFF7E0 > 128)

// macro to validate a flash address
#define IS_VALID_FLASH_ADDRESS(addr) (((addr) >= 0x08000000) && ((addr) < 0x0807FFFF))

// uSec delay macro (not really accurate !)
#define SLEEP_U(us) delay(7*us)

// millis delay macro
#define SLEEP_M(ms) delay(72*ms*100)

// seonds delay macro
#define SLEEP_S(s) delay(72*s*100000)

// BTL Commands
typedef enum {
  CMD_START,
  CMD_END,
  CMD_ACK,
  CMD_INFO,
  CMD_PAGE_OFFSET,
  CMD_NOT_A_CMD = 0xFF
} BTLCommand_t ;

// Bootloader states
typedef enum {
  BTL_WAITING,
  BTL_STARTED,
  BTL_END
} BootloaderState_t;

/* Function Prototypes */
void delay(uint32_t timeout);

// Jump to user code 
void BigJump(void);

/* The bootloader entry point function prototype */
void Reset_Handler(void);

#endif /* COMMON_H_ */
