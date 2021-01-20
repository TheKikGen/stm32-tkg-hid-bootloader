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
  from Bruno Freitas and Vassilis Serasidis <avrsite@yahoo.gr>
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
  https://github.com/TheKikGen/stm32-tkg-hid-bootloader, provide a link to the license,
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <hidapi.h>
#include "rs232.h"
#include "build_number_defines.h"

// Headers needed for sleeping.
#ifdef _WIN32
	#include <windows.h>
#else
	#include <unistd.h>
#endif


/*!< MCU FLASH base address in the alias region */
#define FLASH_BASE_ADDR            ((uint32_t)0x08000000)

// HID read timeout in milliseconds
#define HID_READ_TIMEOUT 1000000

// Serial port RETRY
#define SERIAL_PORT_RETRY 5

/* Maximum packet size. Must be a multiple of 1024 (128 USB packets)  */
#define HID_TX_SIZE    65
#define HID_RX_SIZE     9

// Bootloader command
static const uint8_t CMD_SIGN[] = {'B','T','L','D','C','M','D'};

#define VID              0x1209
#define PID              0xBEBA

// Minimal compatible bootloader firmware
#define MIN_FIRMWARE_VER 0x0240

// BTL Commands
typedef enum {
  CMD_START       = 0,
  CMD_END         = 1,
  CMD_ACK         = 2,
  CMD_INFO        = 3,
  CMD_PAGE_OFFSET = 4,
  CMD_SIMUL       = 5,
  CMD_NOT_A_CMD   = 0XFF
} BTLCommand_t ;

#define BAUD_RATE 115200
#define PORT_MODE '8','N','1',0
//serial.Serial(maple_path, baudrate=115200, xonxoff=1)

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
// Sleep sec, millis, micro time funtions. Redefined due to random timing..;
//------------------------------------------------------------------------------
// Timing is key and notably used by the UsbWrite function.
// When compiling under MinGW64/Windows, sleep and usleep are unstable...
////////////////////////////////////////////////////////////////////////////////

#ifdef WIN32

void Sleep_u(unsigned long us) {

    long long int  t1, t2, freq;
    QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
    QueryPerformanceCounter((LARGE_INTEGER*)&t1);

    do {
        QueryPerformanceCounter((LARGE_INTEGER*)&t2);
    } while( ( (t2 - t1)*1000000/freq ) < us );
}

void Sleep_m(unsigned long ms) { Sleep_u(ms*1000); }
void Sleep_s(unsigned long s) { Sleep_u(s*1000000); }

#else
// Standard functions for Linux or other platforms
void Sleep_m(unsigned long ms) { for (unsigned long t=0; t < ms ; t++) usleep(1000); }
void Sleep_s(unsigned long s) { sleep(s); }
void Sleep_u(unsigned long us) { usleep(us); }

#endif

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
  printf("  Usage: tkg-flash <firmware file name> [<options>]\n");
  printf("         tkg-flash -info\n\n");

  printf("  Options are :\n");
  printf("  -info         : get only some information from the MCU without flashing.\n\n");
  printf("  Flashing options :\n");
  printf("  -d=16 -d=32   : hexa dump sectors by 16 or 32 bytes line length.\n");
  printf("  -ide          : ide embedded, no progression bar, basic info.\n");
  printf("  -o=<n>        : offset the default flash start page of 1-255 page(s)\n");
  printf("  -p=<com port> : serial com port used to toggle DTR for MCU reset.\n");
  printf("  -sim          : flashing simulation.\n");
  printf("  -w=<time s>   : HID device waiting time (10s default).\n");
  printf("\n  Examples :\n  tkg-flash myfirmare.bin -p=COM4 -w=30\n");
  printf("  tkg-flash myfirmare.bin -d=16 -s\n");
}

////////////////////////////////////////////////////////////////////////////////
// Write to usb with retries
////////////////////////////////////////////////////////////////////////////////
static int UsbWrite(uint8_t *data, size_t sz) {
  uint8_t retries = 20;
  int r;

  do {
    // All bytes were sent
    if ( (r = hid_write(HidDeviceHandle, data, sz) ) == sz  ) return sz;
    // USB Error
    if ( r < 0 ) {
      Sleep_m(100);
    }
    // No error but partial data sent. Abort
    else break;
  } while ( --retries );

  return -1;
}

////////////////////////////////////////////////////////////////////////////////
// Start serial port
////////////////////////////////////////////////////////////////////////////////
bool SerialToggleDTR(char *serialPort) {
  const  char portMode[]={PORT_MODE};

  uint8_t nbTry = SERIAL_PORT_RETRY;

  int port = RS232_GetPortnr(serialPort);
  if ( port < 0 ) return false;


  do {
    printf("> Toggling DTR on the [%s] serial port...(%d attempt(s) left)\n",serialPort,nbTry);
    if ( RS232_OpenComport(port, BAUD_RATE, portMode, 0) == 0 ) {
      Sleep_s(1);
      RS232_disableRTS(port);
      Sleep_m(10);

      RS232_disableDTR(port);
      Sleep_m(1);

      RS232_enableDTR(port);
      Sleep_m(1);

      RS232_disableDTR(port);

      // Try magic number
      RS232_enableRTS(port);
      Sleep_m(1);
      RS232_enableDTR(port);
      Sleep_m(1);
      RS232_disableDTR(port);
      Sleep_m(1);
      RS232_SendBuf(port, (unsigned char*)"1EAF", 4);
      Sleep_m(10);
      RS232_flushTX(port);
      RS232_SendBuf(port, (unsigned char*)"1EAF", 4);
      Sleep_m(10);
      RS232_flushTX(port);
      Sleep_m(100);

      RS232_CloseComport(port);

      return true;
    }
    Sleep_s(1);
  } while ( --nbTry );

  return false;
}


////////////////////////////////////////////////////////////////////////////////
// Look up for our bootloader in the USB devices
//------------------------------------------------------------------------------
// Return  -1: if Not found, else version
////////////////////////////////////////////////////////////////////////////////
int HIDDeviceLookUp(bool oneShot) {
  struct hid_device_info *deviceInfo;
  int v = -1;

  // N try, more or less N sec to open the right HID device.
  for( uint8_t i = 0; i < USBTimeout ; i++ ) {
    printf("> Searching for [%04X:%04X] HID device...",VID,PID);
    printf("%c\r",HourGlass());

    deviceInfo = hid_enumerate(VID, PID);
    if ( deviceInfo) v = deviceInfo->release_number;
    hid_free_enumeration(deviceInfo);
    if ( v != -1 || oneShot ) break;
    Sleep_s(1);
  }
  printf("\n");
  return v;
}

////////////////////////////////////////////////////////////////////////////////
// Send a bootloader command
////////////////////////////////////////////////////////////////////////////////
static bool SendBTLCmd(BTLCommand_t cmd, uint8_t cmdData) {
  uint8_t cmdBuff[HID_TX_SIZE];
  memset(cmdBuff, 0, HID_TX_SIZE);

  // Byte 0 is the Report USB HID ID. Copy from byte 1.
  memcpy(&cmdBuff[1],CMD_SIGN,sizeof(CMD_SIGN));
	cmdBuff[sizeof(CMD_SIGN)+1] = cmd;
  cmdBuff[sizeof(CMD_SIGN)+2] = cmdData;

  if ( UsbWrite(cmdBuff, HID_TX_SIZE ) < 0 ) return false;

  return true;
}
////////////////////////////////////////////////////////////////////////////////
// Wait for a valid bootloader ACK command
////////////////////////////////////////////////////////////////////////////////
static bool WaitForACK(void) {
  uint8_t cmdBuff[HID_RX_SIZE];

  while ( hid_read(HidDeviceHandle,cmdBuff, HID_RX_SIZE) != HID_RX_SIZE
            && cmdBuff[7] != CMD_ACK )
  {
              Sleep_m(10);
  }

  return true;
}

////////////////////////////////////////////////////ac////////////////////////////
// Wait for a valid bootloader info block command
////////////////////////////////////////////////////////////////////////////////
static bool WaitForInfoBlock(uint8_t *infoBlock) {

  if ( ! SendBTLCmd(CMD_INFO,0)) return false ;
  while ( hid_read(HidDeviceHandle,infoBlock, HID_RX_SIZE) != HID_RX_SIZE
            && infoBlock[7] != CMD_INFO )
	{

              Sleep_m(10) ;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Wait for a checksum. Return false if checksum doesn't match
////////////////////////////////////////////////////////////////////////////////
static bool WaitForChecksum(uint8_t ck) {
  uint8_t cmdBuff[HID_RX_SIZE];

  while ( hid_read(HidDeviceHandle,cmdBuff, HID_RX_SIZE) != HID_RX_SIZE
            && cmdBuff[7] != CMD_END )
  {
              Sleep_m(10);
  }
	//printf("Checksum F/CLI %d/%d",cmdBuff[0],ck);
  return ( cmdBuff[0] == ck );
}

////////////////////////////////////////////////////////////////////////////////
// Get MCU informations
////////////////////////////////////////////////////////////////////////////////
static bool ShowBootloaderInfo(int version) {

  uint8_t infoBlock[HID_RX_SIZE];
  // Wait for info blockc
  if ( ! WaitForInfoBlock(infoBlock) ) {
      printf("\n  ** Error or timeout while waiting info block from the bootloader.\n");
      return false;
  } else {
      uint16_t memSz     = *( (uint16_t *)infoBlock );
      uint16_t pageSz    = (memSz > 128 ? 2048:1024);

      // Since version 0310, the bootloader send only the page offset
      // so we can recalculate the base address
      uint32_t flashAddr = 0;
      uint8_t  pageOfs = 0 ;

      if ( version >= 0X310 ) {
        pageOfs   = infoBlock[2];
        flashAddr = FLASH_BASE_ADDR + pageOfs * pageSz;
      }
      else {
        flashAddr =  *(uint32_t *)(infoBlock + sizeof(uint16_t));
        pageOfs = ( flashAddr -  FLASH_BASE_ADDR ) / pageSz;
      }

      printf("  INFO - Informations reported by the bootloader :\n");
      printf("      Firmware version       : %04X\n",version);
      printf("      MCU Flash memory size  : %d K (%s density device)\n",memSz,
                     (memSz <= 128 ? "medium":"high") );
      printf("      Page size              : %d bytes\n", pageSz );
      printf("      Page offset            : %d pages\n", pageOfs );
      printf("      Flash base address     : 0x%08X\n",flashAddr);
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {

  int      error = 0;
  char     *serialPortId = (char *) NULL;
  uint8_t  dumpSector = 0;
  bool     simulFlash = false;
  bool     ideEmb = false;
  uint8_t  PageOffset = 0;
  bool     infoMode = false;
  uint8_t  cksum = 0;

	// Build string
	const char TimeStampedVersion[] = {
		VERSION_MAJOR_INIT,
		'.',
			BUILD_YEAR_CH2,
		BUILD_YEAR_CH3,
			BUILD_MONTH_CH0,
		BUILD_MONTH_CH1,
			BUILD_DAY_CH0,
		BUILD_DAY_CH1,
		'.',
		BUILD_HOUR_CH0,
	BUILD_HOUR_CH1,
		BUILD_MIN_CH0,
	BUILD_MIN_CH1,
	0
	};

  setbuf(stdout, NULL);

  printf("\n+-----------------------------------------------------------------------+\n");
  printf  ("|             TKG-Flash v%d.%d STM32F103 HID Bootloader Flash Tool        |\n",
  VERSION_MAJOR,VERSION_MINOR);
  printf  ("|                     High density device support.                      |\n");
  printf  ("|       (c) 2020 - The KikGen Labs     https://github.com/TheKikGen     |\n");
  printf  ("+-----------------------------------------------------------------------+\n");
  printf  ("Build : %s\n\n",(char*) TimeStampedVersion);

  if(argc < 2 ) {
    ShowHelp();
    return 1;
  }

  else if ( argc == 2 && strcmp("-info",argv[1]) == 0 ) {
      // Info mode without flashing
      infoMode = true;
  }
  else {
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
      // Page offset
      if ( ( strncmp("-o=",argv[i],3) == 0 ) && ( strlen(argv[i]) >3 ) ) {
        PageOffset = atoi( argv[i] + 3);
      }
      else
      // Dump 16
      if ( strcmp("-d=16",argv[i]) == 0 ) dumpSector = 16;
      else

      // Dump 32
      if ( strcmp("-d=32",argv[i]) == 0 ) dumpSector = 32;
      else

      // IDE integration (ex Arduino)
      if ( strcmp("-ide",argv[i]) == 0 ) ideEmb = true;
      else

      // Flash simulation
      if ( strcmp("-sim",argv[i]) == 0 ) simulFlash = true;
      else  {
        printf("  ** Error - Unknow command line parameter.\n");
        return 1;
      }
    }
  }

  // HID init
  if (hid_init()) {
      printf("  ** Error - HID init unknow error.\n");
      error = 1;
      goto exit;
  }

  // Check first HID presence to avoid tries loop if already here
  int btlVersion = HIDDeviceLookUp(true);
  if ( btlVersion < 0 ) {

    // Reset the board by toggling DTR
    if ( serialPortId ) {
      if( !SerialToggleDTR(serialPortId) != 0 ) {
        printf("  ** Warning - Unable to open serial port [%s]\n",serialPortId);
      }
    }

    // Check HID again
    btlVersion = HIDDeviceLookUp(false);
    if ( btlVersion < 0 ) {
      printf("  ** Error - [%04X:%04X] HID device was not found.\n",VID,PID);
      error = 1;
      goto exit;
     }
  }

  if ( btlVersion < MIN_FIRMWARE_VER ) {
    printf("  ** Error - Incompatible TKGL HID bootloader version %04X.\n",btlVersion);
    printf("             Minimal version must be %04X.\n",MIN_FIRMWARE_VER);
    error = 1;
    goto exit;
  }

  printf("> [%04X:%04X] HID device found !\n",VID,PID);

  if ( !(HidDeviceHandle = hid_open(VID, PID, NULL)) ) {
    printf("  ** Error - Unable to open the [%04X:%04X] HID device.\n",VID,PID);
    error = 1;
    goto exit;
  }

  // Get info block and check if in INFO mode.
  // Anyway, the bootloader mode will stay active.
  if ( infoMode ) {
    if ( ! ShowBootloaderInfo(btlVersion) )
      error = 1 ;
    else
      printf("  Bootloader mode still active.\n");
    goto exit;
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

  // Set HID read  blocking
  //hid_set_nonblocking(HidDeviceHandle, 0);

  // START BOOTLOADER

  printf("> Flashing firmware");
  if (simulFlash) printf(" (SIMULATION)");
  printf(" :\n");

  // Pages offset
  if ( PageOffset && ! SendBTLCmd(CMD_PAGE_OFFSET,PageOffset)  ) {
    printf("  ** Error while sending bootloader PAGE OFFSET command.\n");
    error = 1;
    goto exit;
  }

  // Send SIMUL command if simulation mode
  if (simulFlash) {
    if ( ! SendBTLCmd(CMD_SIMUL,0)  ) {
      printf("  ** Error while sending bootloader SIMUL command.\n");
      error = 1;
      goto exit;
    }
    // Wait for an ACK because we want to secure the simulation mode
    if (! WaitForACK() ) {
      printf("\n  ** Error or timeout while waiting simulation mode ACK from the bootloader.\n");
      error = 1;
      goto exit;
    }
  }

  // Get info block .
  if ( !infoMode && !ShowBootloaderInfo(btlVersion) ) {
    error = 1 ;
    goto exit;
  }

  if ( ! SendBTLCmd(CMD_START,0)  ) {
    printf("  ** Error while sending bootloader START command.\n");
    error = 1;
    goto exit;
  }

  // Since the version 3.10, we need an ACK for start
  // because we want to secure the upload
  if ( (btlVersion >= 0X0310) && !WaitForACK() ) {
    printf("\n  ** Error or timeout while waiting START ACK from the bootloader.\n");
    error = 1;
    goto exit;
  }

  // Now the bootloader knows we are starting the firmware file transmission
  printf("  START command sent successfully.\n");

  // Prepare the progression bar
  uint8_t bar ;
  float   progression = 0;
  char    wfChar ;

  printf("\n");
  if ( !dumpSector && !ideEmb) {
      printf("  [");
      print_str_repeat('.',50); printf("]\r");
  }

  if (simulFlash) wfChar ='-'; else wfChar = '+';

  // Prepare a first 1024 bytes block.
  uint32_t  bytesSent = 0;
  memset(FileBlock1024, 0, 1024);
  size_t    bytesRead = fread(FileBlock1024, 1, 1024, FileHandle);

  uint8_t   USB_Buffer[HID_TX_SIZE];

  while( bytesRead > 0 ) {
    // Send a 1024 bytes bloc (even if eof) of HID_TX_SIZE-1 bytes packets
    // Value are voluntarly hard coded
    for( uint16_t i = 0; i < 1024 ; i += (HID_TX_SIZE-1) ) {

      memcpy(&USB_Buffer[1], FileBlock1024 + i, HID_TX_SIZE - 1 );

      // Update the checksum
      for ( uint8_t c = 1 ; c <= HID_TX_SIZE - 1 ; c++ )
        cksum ^= USB_Buffer[c];

      // Send data block to USB
      if ( UsbWrite(USB_Buffer, HID_TX_SIZE ) < 0 ) {
        printf("\n  ** Error while sending firmware data to the HID device.\n");
        error = 1;
        goto exit;
      }

      bytesSent += ( HID_TX_SIZE - 1 );
      Sleep_m(1);

      if ( !dumpSector && !ideEmb) {
        // Show the progress
        progression = (float)bytesSent / (float)file_size * 50.0;
        bar = progression ; if (bar > 50) bar = 50;
        printf("  ["); print_str_repeat(wfChar,bar); print_str_repeat('.',50 - bar); printf("]");
        printf("  %6d bytes sent.\r",bytesSent);
      }
    }

    if ( ideEmb ) printf("%c",wfChar);

    // End of block. Wait for the bootloader ACK command.
    if ( ! WaitForACK() ) {
      printf("\n  ** Error or timeout while waiting ACK from the bootloader.\n");
      error = 1;
      goto exit;
    }

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
  if (!SendBTLCmd(CMD_END,0)  ) {
    printf("\n  ** Error while sending bootloader END command.\n");
    error = 1;
    goto exit;
  }

  // Since the version 3.10, we control the checksum
  if ( (btlVersion >= 0X0310) ) {
    if ( !WaitForChecksum(cksum) )
      printf("  ** Checksum error. Firmware could be corrupted. !\n");
    else
      printf("  No checksum error.\n");
  }

  printf("\n> Firmware flashing done !\n");
  printf("  %d sectors written",bytesSent/1024);
  if (simulFlash) printf(" (SIMULATION)");
  printf(".\n");
  if (ideEmb) printf("  %d bytes sent.\n",bytesSent);

exit:
  if ( FileHandle ) fclose( FileHandle );
  if ( HidDeviceHandle ) hid_close(HidDeviceHandle);
  hid_exit();
  printf("> TKG-Flash end.\n");
  return error;
}
