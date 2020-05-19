
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

// TOOLS.  HEXA BYTES ARRAY C SOURCE GENERATOR FROM A BINARY FILE

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

int main(int argc, char** argv) {
    bool OutPutAsText = false;

    printf("// C-Array generator. (c) The KikGen Labs, 2020.\n");
    if (argc < 2 ) {
	    printf ("Usage    : gencarray <file name> [<c array var name>] [options]\n");
      printf ("           -a generate an ascii text file\n");
      printf ("           -c<array name> generate c array named <array name> (default)\n");
	    return 1;
    }

    char* fileName = argv[1];
    char* arrayName = "file_array";

    // Scan command line
    for ( int i = 2 ; i < argc ; i++ ) {

      if ( strncmp("-c",argv[i],2) == 0 && strlen(argv[i]) > 3 )  {
          OutPutAsText = false;
          arrayName = argv[i] + 2 ;
      }
      else
      if ( strncmp("-a",argv[i],2) == 0 ) OutPutAsText = true;
      else  {
        printf("  ** Error - Unknow or invalid command line parameter.\n");
        return 1;
      }
    }


    FILE* firmwareFile = fopen(fileName, "rb");
    if (! firmwareFile ) {
	    printf ("Error while opening %s.\n",fileName);
	    return 1;
    }

    // Get the file size
    unsigned long fileSize = 0;
    fseek(firmwareFile, 0L, SEEK_END);
    fileSize = ftell(firmwareFile);
    rewind(firmwareFile);

    if (fileSize == 0 ) {
	    printf ("Error : size of file %s is 0.\n",fileName);
	    return 1;
    }

    printf("// Generated from binary file %s .\n",fileName);
    printf("// File size is %ld bytes.\n",fileSize);

    if (!OutPutAsText)  printf("uint8_t %s[] = {\n",arrayName);
    uint8_t n = 0;
    int c = 0;
    while( (c = fgetc(firmwareFile) ) != EOF ) {
      if (!OutPutAsText) {
        printf("0x%.2X,", c);
        if(++n % 16 == 0) { n=0; printf("\n");  };
      } else printf("%.2X", c);
    }
    fclose(firmwareFile);
    if (!OutPutAsText) printf("};\n");
}
