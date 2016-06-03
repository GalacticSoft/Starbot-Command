PROG = starbot
CC = g++
CPPFLAGS = -g -Wall
LDFLAGS = -lrt -lusb-1.0 -lgps -lcurses -Wl,-rpath=/usr/local/lib -Lusr/local/lib -lGeographic
OBJS = starbot.o gps.o compass.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
starbot.o gps.o compass.o: 
	$(CC) $(CPPFLAGS) -c starbot.cpp gps.cpp compass.cpp
clean:
	rm -f core $(PROG) $(OBJS)
