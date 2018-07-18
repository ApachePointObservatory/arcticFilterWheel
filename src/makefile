device.so: device.o ArcusPerformaxDriver.o evgpio.o
	gcc -shared -Wl,-soname,libfilter.so.1 -o device.so device.o ArcusPerformaxDriver.o evgpio.o -lusb-1.0 -mcpu=arm9

ArcusPerformaxDriver.o: ArcusPerformaxDriver.c
	gcc -Wall -fPIC -c ArcusPerformaxDriver.c -lusb-1.0 -mcpu=arm9

evgpio.o: evgpio.c
	gcc -Wall -fPIC -c evgpio.c -lusb-1.0 -mcpu=arm9

device.o: device.c
	gcc -Wall -fPIC -c device.c -lusb-1.0 -mcpu=arm9

clean:
	rm -f *.o *.so