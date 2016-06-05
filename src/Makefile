PROG = starbot
CC = g++
CPPFLAGS = -g -Wall
LDFLAGS = -lrt -lusb-1.0 -lgps -lcurses -Wl,-rpath=/usr/local/lib -Lusr/local/lib -lGeographic
OBJS = main.o gps.o compass.o kalman.o usb.o starbot.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
main.o gps.o compass.o kalman.o usb.o starbot.o: 
	$(CC) $(CPPFLAGS) -c main.cpp gps.cpp compass.cpp kalman.cpp usb.cpp starbot.cpp
clean:
	rm -f core $(PROG) $(OBJS)
