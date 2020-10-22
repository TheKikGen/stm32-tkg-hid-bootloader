# To cross compile for windows, install mingw32
# 
i686-w64-mingw32-gcc -c -Wall -I . main.c -o main.o
i686-w64-mingw32-gcc -c -Wall -I . hid-win.c -o hid-win.o
i686-w64-mingw32-gcc -c -Wall -I . rs232.c -o rs232.o
i686-w64-mingw32-gcc main.o hid-win.o rs232.o -lsetupapi -lhid -o tkg-flash.exe

