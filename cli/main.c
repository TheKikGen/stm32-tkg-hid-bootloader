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
*
* Modified 20 April 2018
*	by Vassilis Serasidis <avrsite@yahoo.gr>
*	This HID bootloader work with bluepill + STM32duino + Arduino IDE <http://www.stm32duino.com/>
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include "rs232.h"
#include "hidapi.h"

/* Maximum packet size. Must be a multiple of 1024 (128 USB packets)  */
#define HID_TX_SIZE    65
#define HID_RX_SIZE     9

// Bootloader command
static const uint8_t CMD_SIGN[] = {'B','T','L','D','C','M','D'};

#define VID           0x1209
#define PID           0xBEBA
#define FIRMWARE_VER  0x0100

// BTL Commands
typedef enum {
  CMD_START = 0,
  CMD_END = 1,
  CMD_ACK = 2,
  CMD_NOT_A_CMD = 0XFF
} BTLCommand_t ;


// HID device handle
hid_device *HidDeviceHandle = NULL;

// Firmware is sent in 1024 bytes packets, including the last one even if eof.
// Do not change size as this is crucial for synchronization with the bootloader..
uint8_t FileBlock1024[1024];

// Firmware File handle
FILE *FileHandle = NULL;

// 10s Default timeout for USB
uint8_t USBTimeout = 10;

////////////////////////////////////////////////////////////////////////////////
// Functions prototypes
////////////////////////////////////////////////////////////////////////////////

int serial_init(char *);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Clean DUMP of a buffer to screen
////////////////////////////////////////////////////////////////////////////////
static void ShowBufferHexDump(uint32_t addr,uint8_t* data, uint16_t sz, uint8_t nl)
{
    uint8_t b;
    char asciiBuff[33];
    uint8_t c=0;

    for (uint16_t idx = 0 ; idx < sz; idx++) {
        if (c == 0 ) printf("%08x : ",addr + idx);
        b = (*data++);
  			printf("%02x ",b);
        asciiBuff[c++] = ( b >= 0x20 && b< 127? b : '.' ) ;
        if ( c == nl ) {
          asciiBuff[c] = 0;
          c = 0;
          printf(" | %s\n", &asciiBuff[0]);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// A simple animation during search
////////////////////////////////////////////////////////////////////////////////
static char HourGlass(void) {
  static char * hourGlass="|/-\\";
  static uint8_t i =0;

  if ( ! hourGlass[i] ) i=0;
  return hourGlass[i++];
};

////////////////////////////////////////////////////////////////////////////////
// Printf a char n times
////////////////////////////////////////////////////////////////////////////////
static void print_str_repeat(char c, int n) {
  for ( int i = 0; i !=n ; i++) printf("%c",c);
}

////////////////////////////////////////////////////////////////////////////////
// Show user help
////////////////////////////////////////////////////////////////////////////////
static void ShowHelp() {
  printf("  Usage: tkg-flash <firmware file name> [<options>]\n\n");
  printf("  Options are :\n");
  printf("  -d=16 -d=32   : hexa dump sectors by 16 or 32 bytes line length.\n");
  printf("  -p=<com port> : serial com port used to toggle DTR for MCU reset.\n");
  printf("  -s            : flashing simulation.\n");
  printf("  -w=<time s>   : HID device waiting time (10s default).\n");
  printf("\n  Examples :\n  tkg-flash myfirmare.bin -p=COM4 -w=30\n");
  printf("  tkg-flash myfirmare.bin -d=16 -s\n");
}

////////////////////////////////////////////////////////////////////////////////
// Write to usb with retries
////////////////////////////////////////////////////////////////////////////////
static int usb_write(uint8_t *buffer, int len) {
  int retries = 20;
  int retval;

  while(((retval = hid_write(HidDeviceHandle, buffer, len)) < len) && --retries) {
    if(retval < 0) {
      usleep(100 * 1000); // No data has been sent here. Delay and retry.
    } else {
      return 0; // Partial data has been sent. Firmware will be corrupted. Abort process.
    }
  }
  if(retries <= 0) {
    return 0;
  }
  return 1;
}
////////////////////////////////////////////////////////////////////////////////
// Start serial port
////////////////////////////////////////////////////////////////////////////////
int serial_init(char *serialPort) {

  printf("> Opening the [%s] serial port...\n",serialPort);

  if( RS232_OpenComport(serialPort) ) {
    sleep(1);
    RS232_CloseComport();
    return(1);
  }

  printf("> Toggling DTR...\n");

  RS232_disableRTS();
  RS232_enableDTR();
  usleep(200000L);
  RS232_disableDTR();
  usleep(200000L);
  RS232_enableDTR();
  usleep(200000L);
  RS232_disableDTR();
  usleep(200000L);
  RS232_send_magic();
  usleep(200000L);
  RS232_CloseComport();

  // Wait for reset...
  usleep(250 * 1000);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Look up for our bootloader in the USB devices
//------------------------------------------------------------------------------
// Return 1 if Found, -1: if Not found, -2: if version error
// If no error, the device is opened with HidDeviceHandle
////////////////////////////////////////////////////////////////////////////////
int HIDDeviceLookUp(void) {
  struct hid_device_info *deviceInfo, *nextDevInfo;
  int r = -1;

  // N try, more or less N sec to open the right HID device.
  for( uint8_t i = 0; i < USBTimeout ; i++ ) {
    printf("> Searching for [%04X:%04X] HID device...",VID,PID);
    printf("%c\r",HourGlass());

    deviceInfo = hid_enumerate(VID, PID);
    nextDevInfo = deviceInfo;
    // Search for valid HID Bootloader USB devices
    while ( nextDevInfo && r == -1 ) {
      // Version check
      if( nextDevInfo->release_number < FIRMWARE_VER ) r = -2;
      else if ( (HidDeviceHandle = hid_open(VID, PID, NULL)) ) r = 1;
      nextDevInfo = deviceInfo->next;
    }
    hid_free_enumeration(deviceInfo);
    if ( r != -1 ) break;
    sleep(1); // Wait 1 sec
  }
  printf("\n");
  return r;
}
////////////////////////////////////////////////////////////////////////////////
// Send a bootloader command
////////////////////////////////////////////////////////////////////////////////
static bool SendBTLCmd(BTLCommand_t cmd) {
  uint8_t cmdBuff[HID_TX_SIZE];
  memset(cmdBuff, 0, HID_TX_SIZE);

  // Byte 0 is the Report USB HID ID. Copy from byte 1.
  memcpy(&cmdBuff[1],CMD_SIGN,sizeof(CMD_SIGN));
	cmdBuff[sizeof(CMD_SIGN)+1] = cmd;

  if ( !usb_write(cmdBuff, HID_TX_SIZE ) )
        return false;

  return true;
}
////////////////////////////////////////////////////////////////////////////////
// Wait for a valid bootloader ACK command
////////////////////////////////////////////////////////////////////////////////
static void WaitForACK(void) {
  uint8_t cmdBuff[HID_RX_SIZE];
  do {
      hid_read(HidDeviceHandle,cmdBuff, HID_RX_SIZE) ;
      usleep(500);
  } while( cmdBuff[7] != CMD_ACK);
}

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {

  int       error    = 0;
  char *serialPortId = (char *) NULL;
  uint8_t dumpSector = 0;
  boolean simulFlash = false;

  setbuf(stdout, NULL);

  printf("\n+-----------------------------------------------------------------------+\n");
  printf  ("|           TKG-Flash v2.2.1 STM32F103 HID Bootloader Flash Tool        |\n");
  printf  ("|                     High density device support.                      |\n");
  printf  ("+-----------------------------------------------------------------------+\n");
  printf  ("|   (c)      2020 - The KikGen Labs     https://github.com/TheKikGen    |\n");
  printf  ("|   (c)      2018 - Bruno Freitas       http://www.brunofreitas.com     |\n");
  printf  ("|   (c) 2018-2019 - Vassilis Serasidis  https://www.serasidis.gr        |\n");
  printf  ("|  Customized for STM32duino ecosystem  https://www.stm32duino.com      |\n");
  printf  ("+-----------------------------------------------------------------------+\n\n");

  if(argc < 2) {
    ShowHelp();
    return 1;
  } else {
    // Scan command line
    for ( int i = 2 ; i < argc ; i++ ) {
      // Com port
      if ( ( strncmp("-p=",argv[i],3) == 0 ) && ( strlen(argv[i]) >3 ) ) {
        serialPortId = argv[i] + 3;
      }
      else

      // USB waiting time
      if ( ( strncmp("-w=",argv[i],3) == 0 ) && ( strlen(argv[i]) >3 ) ) {
        USBTimeout = atol( argv[i] + 3);
      }
      else

      // Dump 16
      if ( strncmp("-d=16",argv[i],5) == 0 ) dumpSector = 16;
      else

      // Dump 32
      if ( strncmp("-d=32",argv[i],5) == 0 ) dumpSector = 32;
      else

      // Flash simulation
      if ( strncmp("-s",argv[i],2) == 0 ) simulFlash = true;

      else  {
        printf("  ** Error - Unknow command line parameter.\n");
        return 1;
      }
    }
  }

  // Open the firmware file in binary mode
  FileHandle = fopen(argv[1], "rb");
  if(!FileHandle) {
    printf("  ** Error opening firmware file: %s\n", argv[1]);
    return error;
  }

  // Get the file size to show a progression bar.
  unsigned long file_size = 0;
  fseek(FileHandle, 0L, SEEK_END);
  file_size = ftell(FileHandle);
  rewind(FileHandle);
  printf("> Firmware file size is %ld bytes.\n",file_size);

  //Setting up Serial port
  if (serialPortId) {
    if( serial_init(serialPortId) != 0 )
      printf("  ** Warning - Unable to open serial port [%s]\n",serialPortId);
  }


    // Start and check Human Interface Device
  if ( !simulFlash ) {
    hid_init();

    int r = HIDDeviceLookUp();
    if ( r < 1 ) {
      if ( r == -2 ) printf("\n  ** Error - Incompatible HID bootloader version.\n");
      else printf("  ** Error - [%04X:%04X] HID device was not found.\n",VID,PID);
      error = 1;
      goto exit;
    }
    printf("> [%04X:%04X] HID device found !\n",VID,PID);

    //HidDeviceHandle = hid_open(VID, PID, NULL);
    if (HidDeviceHandle == NULL) {
      printf("  ** Error - Unable to open the [%04X:%04X] HID device.\n",VID,PID);
      error = 1;
      goto exit;
    }
  }

  // Set HID read blocking
  //hid_set_nonblocking(HidDeviceHandle, 0);

  // START BOOTLOADER

  printf("> Flashing firmware");
  if (simulFlash) printf(" (SIMULATION)");
  printf(" :\n");

  if (!simulFlash &&  ! SendBTLCmd(CMD_START)  ) {
    printf("  ** Error while sending bootloader START command.\n");
    error = 1;
    goto exit;
  }

  // Now the bootloader knows we are starting the firmware file transmission

  // Prepare the progression bar
  if ( !dumpSector ) {
    printf("["); print_str_repeat('.',50); printf("]\r");
  }
  uint8_t   bar ;
  float progression = 0;

  // Prepare a first 1024 bytes bloc.
  uint32_t  bytesSent = 0;
  memset(FileBlock1024, 0, 1024);
  size_t    bytesRead = fread(FileBlock1024, 1, 1024, FileHandle);

  while( bytesRead > 0 ) {
    // Send a 1024 bytes bloc (even if eof).
    // Value are voluntarly hard coded
    for( uint16_t i = 0; i < 1024 ; i += (HID_TX_SIZE-1) ) {

      if ( !simulFlash ) {
        uint8_t USB_Buffer[HID_TX_SIZE];
        memcpy(&USB_Buffer[1], FileBlock1024 + i, HID_TX_SIZE - 1 );

        // Send data block to USB
        if ( !usb_write(USB_Buffer, HID_TX_SIZE ) ) {
          printf("\n  ** Error while sending firmware data to the HID device.\n");
          error = 1;
          goto exit;
        }
        usleep(500);
      }

      bytesSent += ( HID_TX_SIZE - 1 );

      if ( ! dumpSector) {
        // Show the progress
        progression = (float)bytesSent / (float)file_size * 50.0;
        bar = progression ; if (bar > 50) bar = 50;
        printf("["); print_str_repeat('+',bar); print_str_repeat('.',50 - bar); printf("]");
        printf("  %6d bytes\r",bytesSent);
      }
    }

    // End of block. Wait for the bootloader ACK command.
    if (!simulFlash) WaitForACK();

    // Dump the sector
    if (dumpSector) {
      printf("\n");
      ShowBufferHexDump(bytesSent - 1024,FileBlock1024, 1024, dumpSector);
    }


    // Read next bytes
    memset(FileBlock1024, 0, 1024);
    bytesRead = fread(FileBlock1024, 1, 1024, FileHandle);
  }

  printf("\n");

  // End of file. Send CMD_END to finish flashing and reboot the microcontroller...
  if (!simulFlash && !SendBTLCmd(CMD_END)  ) {
    printf("  ** Error while sending bootloader END command.\n");
    error = 1;
    goto exit;
  }

  printf("> Firmware flashing done ! %d sectors written",bytesSent/1024);
  if (simulFlash) printf(" (SIMULATION)");
  printf(".\n");

exit:
  if ( FileHandle ) fclose( FileHandle );
  if ( HidDeviceHandle ) hid_close(HidDeviceHandle);
  hid_exit();

  // Reopen initial serial port.
  if (serialPortId) {
    int i = 0;
    for(  ; i < 5 ; i ++) {
      printf("> Searching for [%s] serial port...",serialPortId);
      printf("%c\r",HourGlass());
      if( RS232_OpenComport(serialPortId) == 0 ) break;
      sleep(1);
    }
    printf("\n");
    if( i == 5) printf("  ** Warning - serial port %s was not found\n",serialPortId);
    else printf("> Serial port [%s] found !\n",serialPortId );
  }

  printf("> End of firmware update.\n");
  return error;
}
