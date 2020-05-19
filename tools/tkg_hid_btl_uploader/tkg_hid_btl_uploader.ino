/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
  USBMIDIKLIK 4X4 - USB Midi advanced firmware for STM32F1 platform.
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2019 TheKikGen Labs team.
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
  https://github.com/TheKikGen/USBMidiKliK4x4 , provide a link to the license,
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
///////////////////////////////////////////////////////////////////////////////
//  EEPROM UTILITIES
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
// EEPROM library is not used anymore due to low frequenry rate of update of
// setting.  To avoid conflicts with that library, some core ST flash FUNCTIONS
// are included here directly in the source code.
//-----------------------------------------------------------------------------
// The FPEC block handles the program and erase operations of the Flash memory.
// The FPEC consists of seven 32-bit registers.
// . FPEC key register (FLASH_KEYR)
// . Option byte key register (FLASH_OPTKEYR)
// . Flash control register (FLASH_CR)
// . Flash status register (FLASH_SR)
// . Flash address register (FLASH_AR)
// . Option byte register (FLASH_OBR)
// . Write protection register (FLASH_WRPR)
//
// After reset, the FPEC block and so the FLASH_CR register are locked.
// To unlock the FPEC block, where two key values (KEY1 and KEY2) must be
// written to the FLASH_KEYR.  Any wrong sequence locks up the FPEC block
// and FLASH_CR register until the next reset.
//
// FLASH_SR register :
// Bits 31:6 Reserved, must be kept cleared.
// Bit 5EOP: End of operation
// Set by hardware when a Flash operation (programming / erase) is completed. Reset by
// writing a 1
// Note: EOP is asserted at the end of each successful program or erase operation
// Bit 4WRPRTERR: Write protection error
// Set by hardware when programming a write-protected address of the Flash memory.
// Reset by writing 1.
// Bit 3 Reserved, must be kept cleared.
// Bit 2PGERR: Programming error
// Set by hardware when an address to be programmed contains a value different from
// '0xFFFF' before programming.
// Reset by writing 1.
// Note: The STRT bit in the FLASH_CR register should be reset before starting a programming
// operation.
// Bit 1 Reserved, must be kept cleared
// Bit 0BSY: Busy
// This indicates that a Flash operation is in progress. This is set on the beginning of a Flash
// operation and reset when the operation finishes or when an error occurs
///////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdarg.h>
#include <libmaple/nvic.h>
#include "libmaple/flash.h"
#include "libmaple/pwr.h"
#include "libmaple/rcc.h"
#include "libmaple/bkp.h"

#include "tkg_hid_generic_pc13.bin.h"
#include "tkg_hid_miditech4x4.bin.h"

#define FORCE_MODE_MIDITECH

#define EE_FLASH_MEMORY_BASE 0x08000000

typedef enum
	{
	FLASH_COMPLETE = 0,
	FLASH_BUSY,
	FLASH_ERROR_WRITE,
	FLASH_ERROR_PG,
	FLASH_ERROR_WRP,
	FLASH_ERROR_OPT,
	FLASH_ERROR_BAD_PAGE_SIZE,
	FLASH_BAD_ADDRESS
	} FLASH_Status;

#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x0807FFFF))
#define FLASH_KEY1  0x45670123
#define FLASH_KEY2  0xCDEF89AB

uint8_t IsMidiface = 0;

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
// Low level functions
void    FLASH_Unlock();
void    FLASH_Lock();
void   FLASH_WaitEndOfOperation();
FLASH_Status  FLASH_ErasePage(uint32);
FLASH_Status  FLASH_ProgramHalfWord(uint32, uint16);
FLASH_Status  FLASH_WritePage(uint8_t,uint8_t *,uint16_t,uint16_t);
boolean  FLASH_DiffPage(uint8_t,uint8_t *,uint16_t,uint16_t);

///////////////////////////////////////////////////////////////////////////////
// STM32F103 flash memory FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Unlock FPEC block
///////////////////////////////////////////////////////////////////////////////
void FLASH_Unlock()
{
	// If already unlocked, do nothing
	if ( FLASH_BASE->CR & FLASH_CR_LOCK ) {
		FLASH_BASE->KEYR = FLASH_KEY1;
		FLASH_BASE->KEYR = FLASH_KEY2;
	}
}

///////////////////////////////////////////////////////////////////////////////
// lock FPEC block
///////////////////////////////////////////////////////////////////////////////
void FLASH_Lock()
{
	FLASH_BASE->CR |= FLASH_CR_LOCK;
}

///////////////////////////////////////////////////////////////////////////////
// Wait end of current flash operaiton. return false if timeout reached.
///////////////////////////////////////////////////////////////////////////////
void FLASH_WaitEndOfOperation() {
	while ( (FLASH_BASE->SR & FLASH_SR_BSY) ) ;
}

///////////////////////////////////////////////////////////////////////////////
// Erase a flash memory page
//-----------------------------------------------------------------------------
// . Check the BSY bit in the FLASH_SR register
// . Set the PER bit in the FLASH_CR register
// . Program the FLASH_AR register to select a page to erase
// . Set the STRT bit in the FLASH_CR register
// . Wait for the BSY bit to be reset
// . Read the erased page and verify (eventually)
///////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_ErasePage(uint32 pageAddress)
{
	if ( ! IS_FLASH_ADDRESS(pageAddress) ) return FLASH_BAD_ADDRESS;

	FLASH_WaitEndOfOperation();

	FLASH_Status status = FLASH_COMPLETE;

	FLASH_Unlock(); FLASH_WaitEndOfOperation();

	// Erase the page
	FLASH_BASE->CR |= FLASH_CR_PER;
	FLASH_BASE->AR =  pageAddress;
	FLASH_BASE->CR = FLASH_CR_STRT | FLASH_CR_PER;;
	FLASH_WaitEndOfOperation();

	// PER bit in CR register must be cleared to allows other operations on flash to be run after that
	FLASH_BASE->CR &= ~FLASH_CR_PER;
	FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);

	FLASH_Lock();	FLASH_WaitEndOfOperation();

	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Program a half word 16 bits value
//-----------------------------------------------------------------------------
// . Check the BSY bit in the FLASH_SR register.
// . Set the PG bit in the FLASH_CR register.
// . Perform the data write (half-word) at the desired address.
// . Wait for the BSY bit to be reset.
// . Read the programmed value and verify.
///////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_ProgramHalfWord(uint32 writeAddress, uint16 value )
{

	if ( ! IS_FLASH_ADDRESS(writeAddress) ) return FLASH_BAD_ADDRESS;

	// If value not changed, nothing to do...
	if ( value == *(uint16*)writeAddress ) return FLASH_COMPLETE;

	FLASH_WaitEndOfOperation();

	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock(); FLASH_WaitEndOfOperation() ;

	FLASH_BASE->CR |= FLASH_CR_PG;
	*(uint16*)writeAddress = value;

	FLASH_WaitEndOfOperation();
	if (FLASH_BASE->SR & FLASH_SR_PGERR) status = FLASH_ERROR_PG;
	else if ( FLASH_BASE->SR & FLASH_SR_WRPRTERR )status = FLASH_ERROR_WRP;
	//Verify the value written
	else if ( value != *(volatile uint16_t *)writeAddress ) status = FLASH_ERROR_WRITE;

	FLASH_BASE->CR &= ~FLASH_CR_PG;
	FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);

	FLASH_Lock();	FLASH_WaitEndOfOperation();

	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Write bytes to a page of the flash memory
//////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_WritePage(uint8_t page, uint8_t *bloc,uint16_t sz,uint16_t pageSize)
{
	// The page must be prepared before the call.
	if ( sz < 2 || sz > pageSize ) return FLASH_ERROR_BAD_PAGE_SIZE;

	// No necessary to write when no differences
	if ( ! FLASH_DiffPage(page, bloc,sz,pageSize) ) return FLASH_COMPLETE;

	FLASH_WaitEndOfOperation();

	uint32_t volatile addressWrite = (EE_FLASH_MEMORY_BASE + page * pageSize) ;

	FLASH_Status status = FLASH_ErasePage(addressWrite);
	if ( status != FLASH_COMPLETE ) return status;

// A byte is 8 bits, but flash write operation is mandatory 16 bit.
	uint16_t *pAddress = (uint16_t *) addressWrite;
	uint16_t *pValue   = (uint16_t *) bloc ;

	uint16_t size = sz / 2 + ( sz % 2 ? 1:0 );
	boolean  cleanLast =  ( (size * 2) > sz );

  while(size) {
		// Write only if differences to preserve the Flash memory
		if ( *pAddress != *pValue ) {
			uint16_t flashValue = 0XFF00;  // 0xFF is the "format" value
			// Write our last value without out of bound garbage byte
			if (size == 1 && cleanLast ) flashValue += *( (uint8_t*)pValue ) ;
			else flashValue = *pValue;
			status = FLASH_ProgramHalfWord((uint32_t)pAddress, flashValue);
			if (  status != FLASH_COMPLETE ) return status ;
		}
		pAddress++; pValue++; size--;
	}
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Check if differences exist between a page and a buffer
//////////////////////////////////////////////////////////////////////////////
boolean FLASH_DiffPage(uint8_t page, uint8_t *bloc,uint16_t sz,uint16_t pageSize)
{
  uint32_t addressRead = EE_FLASH_MEMORY_BASE + page * pageSize ;
	return ( memcmp((void *)addressRead,(void *)bloc,sz) != 0 ) ;
}

///////////////////////////////////////////////////////////////////////////////
// USB serial getchar
///////////////////////////////////////////////////////////////////////////////
char AskChar()
{
  while (!Serial.available() >0);
  char c = Serial.read();
  // Flush
  while (Serial.available()>0) Serial.read();
  return c;
}
///////////////////////////////////////////////////////////////////////////////
// Flash the bootloader in the C bytes array
///////////////////////////////////////////////////////////////////////////////
void FlashHidBTL(uint16_t pageSize) {

	uint8_t * bloc;
	uint16_t sz;

	if ( IsMidiface ) {
		bloc = (uint8_t *)tkg_hid_miditech4x4;
		sz = sizeof(tkg_hid_miditech4x4);

	} else {
		bloc = (uint8_t *)tkg_hid_generic_pc13;
		sz = sizeof(tkg_hid_generic_pc13);
	};

	Serial.print("HID Bootloader size is ");Serial.print(sz);Serial.println(" bytes.");

	uint16_t nbPageWrite = ( sz / pageSize ) + ( sz % pageSize ? 1:0);
	uint16_t NbBytesWriteSum = 0;
	// Start at page 0
	for ( uint16_t page = 0 ; page != nbPageWrite ; page++ ) {
			uint16_t NbBytesWrite = (sz > pageSize ? pageSize : sz);
			FLASH_WritePage(page , bloc, NbBytesWrite ,pageSize);
			bloc += NbBytesWrite;
			sz -= NbBytesWrite;
			NbBytesWriteSum += NbBytesWrite;
			Serial.print("....");
	}

	Serial.print( NbBytesWriteSum);Serial.println(" bytes written.");

}


void setup()
{
	Serial.end();
	// Activate USB with DISC PIN for Miditech
	// Will do nothing for others boards...
	gpio_set_mode(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, GPIO_OUTPUT_PP);
	gpio_write_bit(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, 1);

	// start USB serial
	Serial.begin(115200);

	while (!Serial);

}

void loop()
{

	uint16_t pageSize = 1024;
	// #if defined(FORCE_MODE_MIDITECH)
	// IsMidiface = 1;
	// FlashHidBTL(pageSize);
	// delay(10000);
	//
	// while (1);
	// #endif

	uint8_t c =0;
	Serial.println("STM32F103 TKG HID BOOTLOADER HIGH DENSITY SUPPORT 2.2 - UPLOADER BY THE KIGEN LABS");
	Serial.println();
	Serial.println("This Arduino sketch will flash the unified HID Bootloader at address 0x00000000,");
	Serial.println("and will replace any other bootloader in place.That will free about 6144 bytes for user programs.");
	Serial.println("NB : Applications must be compiled against the right upload method as the vector table address is 0x08001000.");
	Serial.println();
	Serial.println("Do you want continue ? (Y/n)");

	if ( (c= AskChar()) == 'Y' ) {
		Serial.println("Is your device is High density (n if Bluepill) ? (Yn)");
		if ( (c= AskChar()) == 'Y' ) {
			pageSize = 2048;
		}
		Serial.println("Is your device is a MidiTech/MidiPlus midiface 4x4 interface ? (Yn)");
		if ( (c= AskChar()) == 'Y' ) IsMidiface = 1;

		Serial.println("Proceed ? (Y/n)");
		if ( (c= AskChar()) == 'Y' ) {
			FlashHidBTL(pageSize);
			Serial.println("Bootloader flashed.");
			Serial.println("Please reset the board.");
			while(1);
		}
	}


}
