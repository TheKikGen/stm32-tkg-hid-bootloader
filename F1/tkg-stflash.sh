#!/bin/bash

# Flash with ST-LINK command line utility
# e.g. tkg-stflash generic_pc13

st-flash --reset write ./bootloader_only_binaries/tkg_hid_$1.bin 0x08000000
 
