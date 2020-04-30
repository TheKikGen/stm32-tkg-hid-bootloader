/*
* STM32 HID Bootloader - USB HID bootloader for STM32F10X
* Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HID_H_
#define HID_H_

// Memory addresses common for all the STM32F103 family

/*!< FLASH base address in the alias region */
#define FLASH_BASE_ADDR            ((uint32_t)0x08000000)

/*!< SRAM base address in the alias region */
#define SRAM_BASE_ADDR             ((uint32_t)0x20000000)

/* Short macro to detect a high density device from the flash memory size */
#define IS_HIGH_DENSITY ( *(uint16_t *)0x1FFFF7E0 > 128 )

/* Bootloader nb pages size in high density (2048 bytes)*/
#define BOOTLOADER_PAGE_SIZE_H		1

/* minimal SRAM size for the bootloader */
#define SRAM_SIZE			(8 * 1024)

/* SRAM end (bottom of stack) */
#define SRAM_END			(SRAM_BASE_ADDR + SRAM_SIZE)

/* HID Bootloader takes 2 kb flash. */
#define USER_PROGRAM	(FLASH_BASE_ADDR + BOOTLOADER_PAGE_SIZE_H * 2048)

// Magic word to enter the bootloader
#define MAGIC_WORD1 0x424C


// BTL Commands
typedef enum {
  CMD_START,
  CMD_END,
  CMD_ACK,
  CMD_NOT_A_CMD = 0xFF
} BTLCommand_t ;

// Bootloader states
typedef enum {
  BTL_WAITING,
  BTL_STARTED,
  BTL_END
} BootloaderState_t;

/* Global Variables */
extern volatile BootloaderState_t BootloaderState;

/* Function Prototypes */
void USB_Reset(void);
void USB_EPHandler(uint16_t Status);
void delay(uint32_t timeout);

#endif /* HID_H_ */
