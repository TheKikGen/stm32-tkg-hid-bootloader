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
#include <stdbool.h>
#include "usb.h"
#include "config.h"
#include "hid.h"
#include "led.h"


/* Initial stack pointer index in vector table*/
#define INITIAL_MSP			0

/* Reset handler index in vector table*/
#define RESET_HANDLER			1

/* USB Low-Priority and CAN1 RX0 IRQ handler idnex in vector table */
#define USB_LP_CAN1_RX0_IRQ_HANDLER	36

/* The bootloader entry point function prototype */
void Reset_Handler(void);

/* Minimal initial Flash-based vector table */
uint32_t *VectorTable[] __attribute__((section(".isr_vector"))) = {
	/* Initial stack pointer (MSP) */
	(uint32_t *) SRAM_END,
	/* Initial program counter (PC): Reset handler */
	(uint32_t *) Reset_Handler
};

void delay(uint32_t timeout)
{
	for (uint32_t i = 0; i < timeout; i++) {
		__NOP();
	}
}

static uint16_t get_and_clear_magic_word(void)
{

	/* Enable the power and backup interface clocks by setting the
	 * PWREN and BKPEN bits in the RCC_APB1ENR register
	 */
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	uint16_t value = READ_REG(BKP->DR4);
	/* Enable write access to the backup registers and the
 * RTC.
 */
	SET_BIT(PWR->CR, PWR_CR_DBP);
	WRITE_REG(BKP->DR4, 0x0000);
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);

	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	return value;
}

static void set_sysclock_to_72_mhz(void)
{

	/* Enable HSE */
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	/* Wait until HSE is ready */
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) ;

	/* Enable Prefetch Buffer & set Flash access to 2 wait states */
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2);

	/* SYSCLK = PCLK2 = HCLK */
	/* PCLK1 = HCLK / 2 */
	/* PLLCLK = HSE * 9 = 72 MHz */
	SET_BIT(RCC->CFGR,
		RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2 |
		RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);

	/* Enable PLL */
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	/* Wait until PLL is ready */
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ;

	/* Select PLL as system clock source */
	SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);

	/* Wait until PLL is used as system clock source */
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_1) == 0) ;
}

// Entry point - Main.
void Reset_Handler(void)
{
	volatile uint32_t *const ram_vectors = (volatile uint32_t *const) SRAM_BASE;

	/* Setup the system clock (System clock source, PLL Multiplier
	 * factors, AHB/APBx prescalers and Flash settings)
	 */
	set_sysclock_to_72_mhz();

	/* Setup a temporary vector table into SRAM, so we can handle
	 * USB IRQs
	 */
	ram_vectors[INITIAL_MSP]                 = SRAM_END;
	ram_vectors[RESET_HANDLER]               = (uint32_t) Reset_Handler;
	ram_vectors[USB_LP_CAN1_RX0_IRQ_HANDLER] = (uint32_t) USB_LP_CAN1_RX0_IRQHandler;
	__DSB();
	WRITE_REG(SCB->VTOR, (volatile uint32_t) ram_vectors);
	__DSB();

	/* Initialize GPIOs */
	pins_init();

	/* Wait 1us so the pull-up settles... */
	delay(72);

	// Initial state
	BootloaderState = BTL_WAITING;

	// Activate USB HID and wait for an eventual START command from CLI
	USB_Init();
	LED2_ON;
	delay(18000000L);
	LED2_OFF;

	// Entering bootloader if :
	//
	// no User Code is uploaded to the MCU or
	// MAGIC_WORD1 was stored in the DR4 battery-backed RAM register or
	// BOOT 1 jumper is set to HIGH. (PB2 / GPIO_IDR_IDR2) or
	// TKG-FLASH CLI activated upload during the bootloader startup
	if (	 ( ( (*(volatile uint32_t *) USER_PROGRAM ) & 0x2FFE0000 ) != SRAM_BASE )
				 || get_and_clear_magic_word() == MAGIC_WORD1
				 || READ_BIT(GPIOB->IDR, GPIO_IDR_IDR2)
				 || BootloaderState == BTL_STARTED
		 )
	{
			// Flashing loop. Flashing is done in the ISR.
			do {
					delay(400L);
					if ( BootloaderState == BTL_WAITING ) {
							LED1_ON; 	delay(200000L);
							LED1_OFF;	delay(200000L);
					}
			}	while (BootloaderState != BTL_END ) ;

			// Hardware Reset
			while(1) NVIC_SystemReset();
	}

	USB_Shutdown();
	// Start user code
	LED2_ON;

	/* Turn GPIO clocks off */
	CLEAR_BIT(RCC->APB2ENR,
			LED1_CLOCK | LED2_CLOCK | DISC_CLOCK/* | RCC_APB2ENR_IOPBEN*/);

	// Go to user code
	__DSB();
	 WRITE_REG(SCB->VTOR, (volatile uint32_t)(USER_PROGRAM & 0xFFFF) );
	__DSB();

	 // Initialise master stack pointer.
	 __asm__ volatile("msr msp, %0"::"g"
				(*(volatile uint32_t *)USER_PROGRAM));

	 // Jump to application.
	 (*(void (**)())(USER_PROGRAM + 4))();

		// NEVER REACHED
}
