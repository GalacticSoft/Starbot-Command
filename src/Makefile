PROG = starbot
DEMO = demo
CLEAN = clean
CC = g++
CPPFLAGS = -g -Wall -Wno-write-strings
LDFLAGS = -lrt -lusb-1.0 -lgps -lcurses -Wl,-rpath=/usr/local/lib -Lusr/local/lib -lGeographic
DEMOLDFLAGS = -lrt -lgps -Wl,-rpath=/usr/local/lib -Lusr/local/lib -lGeographic
OBJS = main.o gps.o compass.o kalman.o usb.o starbot.o wmm.o
DEMOOBJS = demo.o gps.o compass.o kalman.o wmm.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
main.o gps.o compass.o kalman.o usb.o starbot.o wmm.o: 
	$(CC) $(CPPFLAGS) -c main.cpp gps.cpp compass.cpp kalman.cpp usb.cpp starbot.cpp wmm.cpp
	
$(DEMO) : $(OBJS)
	$(CC) -o $(DEMO) $(DEMOOBJS) $(DEMOLDFLAGS)
demo.o gps.o compass.o kalman.o wmm.o: 
	$(CC) $(CPPFLAGS) -c demo.cpp gps.cpp compass.cpp kalman.cpp wmm.cpp
	
$(CLEAN):
	rm -f core $(PROG) $(OBJS)
	rm -f core $(DEMO) $(DEMOOBJS)
