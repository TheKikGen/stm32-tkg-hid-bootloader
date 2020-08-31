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

#include <stm32f10x.h>
#include <system_stm32f10x.h>
#include <stdbool.h>
#include "usb.h"
#include "config.h"
#include "common.h"
#include "hid.h"

// Indexes vector table settings
#define INITIAL_MSP			0              // Initial stack pointer index
#define RESET_HANDLER		1              // Reset handler index
#define USB_LP_CAN1_RX0_IRQ_HANDLER	36 // USB Low-Priority and CAN1 RX0 IRQ handler index
#define BACKUP_REG DR10
// Minimal initial Flash-based vector table */
uint32_t *VectorTable[] __attribute__((section(".isr_vector"))) = {
	(uint32_t *) SRAM_END,     // Initial stack pointer (MSP) (see common.h)
	(uint32_t *) Reset_Handler // Initial program counter (PC): Reset handler
};

///////////////////////////////////////////////////////////////////////////////
// Delay function. For uSec and mSec, use SLEEP_U and SLEEP_M macros
///////////////////////////////////////////////////////////////////////////////
void delay(uint32_t t) {
	for (uint32_t i = 0; i !=t; i++) 	__NOP();
}

///////////////////////////////////////////////////////////////////////////////
// Jump to user code
//-----------------------------------------------------------------------------
// Dedicated function with no call to any function (appart the last call)
// This way, there is no manipulation of the stack here, ensuring that GGC
// didn't insert any pop from the SP after having set the MSP.
// If msp must be set outside a dedicated function, use __set_MSP from CMSIS kit
///////////////////////////////////////////////////////////////////////////////
void BigJump(void) {
	// Setup the vector table to the final user-defined one in Flash memory
	__DSB();
	WRITE_REG(SCB->VTOR, USER_ADDR);

	// Setup the stack pointer to the user-defined one
	//__asm__ volatile("msr msp, %0"::"g"	(*(volatile uint32_t *)USER_ADDR));
	__set_MSP((*(volatile uint32_t *) USER_ADDR));

	// Jump to the user firmware entry point
	(*(void (**)())(USER_ADDR + 4))();
}

///////////////////////////////////////////////////////////////////////////////
// BOOTLOADER MAIN ENTRY POINT
///////////////////////////////////////////////////////////////////////////////
void Reset_Handler(void)
{
	USB_Shutdown();
	SystemInit();
	SystemCoreClockUpdate(); // 72 Mhz default

	// Initialize GPIOs /////////////////////////////////////////////////////////
	SET_BIT(RCC->APB2ENR, LED1_CLOCK | LED2_CLOCK | DISC_CLOCK | RCC_APB2ENR_IOPBEN);
	LED1_BIT_0;	LED1_BIT_1; LED1_MODE;
	LED2_BIT_0; LED2_BIT_1; LED2_MODE;
	DISC_BIT_0; DISC_BIT_1;	DISC_MODE; DISC_LOW;
  #if defined PB2_PULLDOWN
	SET_BIT(GPIOB->CRL, GPIO_CRL_CNF2_1); CLEAR_BIT(GPIOB->ODR, GPIO_ODR_ODR2);
	#endif // NB : PB2 is already in FLOATING mode by default.
	#if defined PB2_LOW
	CLEAR_BIT(GPIOB->CRL, GPIO_CRL_CNF2_0);
	CLEAR_BIT(GPIOB->CRL, GPIO_CRL_CNF2_1);
	SET_BIT(GPIOB->CRL, GPIO_CRL_MODE2);
	WRITE_REG(GPIOA->BRR, GPIO_BRR_BR2);

	#endif
	SLEEP_M(1);

	// GPIOs end ////////////////////////////////////////////////////////////////

	// Get Magic word from DR10 if any ///////////////////////////////////////////
	// Enable the power and backup interface clocks by setting the
	// PWREN and BKPEN bits in the RCC_APB1ENR register
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	bool magicHere = ( READ_REG(BKP->BACKUP_REG) == MAGIC_WORD1 );


	/// Magic word end //////////////////////////////////////////////////////////

	// Entering bootloader //////////////////////////////////////////////////////
	// Enter if :
	// no User Code is uploaded to the MCU or
	// MAGIC_WORD1 was stored in the DR10 battery-backed RAM register or
	// No MAGIC_WORD && Reset occurs before waiting loop end or
	// BOOT 1 jumper is set to HIGH. (PB2 / GPIO_IDR_IDR2) or
	// TKG-FLASH CLI activated upload during the bootloader startup
	BootloaderState = BTL_WAITING;

	bool MustEnterBooloader = (
		CHECK_USER_CODE(USER_ADDR)
		|| magicHere
		|| READ_BIT(GPIOB->IDR, GPIO_IDR_IDR2 )
	);

	// // Waiting loop around 3s

	// Enable write access to the backup registers and the RTC.
	SET_BIT(PWR->CR, PWR_CR_DBP);

	if ( ! MustEnterBooloader ) {
			// If reset occurs before the end of loop,
			// that will activate bootloader mode at the next reset

			WRITE_REG(BKP->BACKUP_REG, MAGIC_WORD1);

			for (uint32_t i = 0 ; i<3 ; i++) {
				LED1_ON;  SLEEP_M(200);
				LED1_OFF;	SLEEP_M(200);
			}
	}

	// Set DR4 backup register to zero
	// Then reset backup registers and the RTC.
	WRITE_REG(BKP->BACKUP_REG, 0x0000);
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

	// Lock Flash because of security.
	//SET_BIT(FLASH->CR, FLASH_CR_LOCK);

	// Activate HID
	// Setup a temporary vector table into SRAM, so we can handle USB IRQs //////
	volatile uint32_t *const ram_vectors =	(volatile uint32_t *const) SRAM_BASE;
	ram_vectors[INITIAL_MSP]                 = SRAM_END;
	ram_vectors[RESET_HANDLER]               = (uint32_t) Reset_Handler;
	ram_vectors[USB_LP_CAN1_RX0_IRQ_HANDLER] = (uint32_t) USB_LP_CAN1_RX0_IRQHandler;
	__DSB();
	WRITE_REG(SCB->VTOR, (volatile uint32_t) ram_vectors);

	USB_Init();

	// 2nd Waiting round for an eventual bootloader START cmd
	if ( ! MustEnterBooloader ) {
			for (uint16_t i = 1 ; i<15 ; i++) {
				if ( BootloaderState == BTL_STARTED ) {
					MustEnterBooloader = true;
					break;
				}
				LED1_ON; 	SLEEP_M(100);
				LED1_OFF; SLEEP_M(100);
			}
	}

	if (	MustEnterBooloader 	) {
			// Flashing loop. Flashing is done in the ISR.
			do {
					SLEEP_M(1);
					if ( BootloaderState == BTL_WAITING ) {
							LED1_ON; 	SLEEP_M(20);
							LED1_OFF;	SLEEP_M(20);
					}
			}	while (BootloaderState != BTL_END ) ;
			USB_Shutdown();
			SLEEP_M(10);
			NVIC_SystemReset();
	}

	USB_Shutdown();
	SLEEP_S(2);

	LED1_OFF;
	// Turn GPIO clocks off
	CLEAR_BIT(RCC->APB2ENR,	LED1_CLOCK | LED2_CLOCK | DISC_CLOCK/* | RCC_APB2ENR_IOPBEN*/);

	// Go to user code
	BigJump();
	for (;;) {
		;
	}

}
