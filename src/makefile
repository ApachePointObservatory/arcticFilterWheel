all: scan scan.so

scan: scan.o ArcusPerformaxDriver.o evgpio.o
	gcc -o scan scan.o ArcusPerformaxDriver.o evgpio.o -lusb-1.0 -mcpu=arm9

scan.so: scan.o scan.o ArcusPerformaxDriver.o evgpio.o
	gcc -shared -Wl,-soname,libfilter.so.1 -o scan.so scan.o ArcusPerformaxDriver.o evgpio.o -lusb-1.0 -mcpu=arm9

ArcusPerformaxDriver.o: ArcusPerformaxDriver.c
	gcc -Wall -fPIC -c ArcusPerformaxDriver.c -lusb-1.0 -mcpu=arm9

evgpio.o: evgpio.c
	gcc -Wall -fPIC -c evgpio.c -lusb-1.0 -mcpu=arm9

scan.o: scan.c
	gcc -Wall -fPIC -c scan.c -lusb-1.0 -mcpu=arm9

clean:
	rm -f *.o scan scan.so