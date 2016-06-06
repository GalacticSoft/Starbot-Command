/*
 * 	ev314_test_usb_com: test program to validate USB communications.
 * 
 * 	This is a host program. It cannot be run on the EV3.
 * 	It must be run on a Linux host connected to a EV3 device.
 * 
 * 	To compile: g++ -Wall -o starbot starbot.cpp -lrt -lusb-1.0 -lgps
 * 
 * 	JG, 24.10.14
 */

/* Includes */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   //_getch
#include <termios.h>  //_getch
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include "ev314.h" // EV314 Firmware
#include "compass.h"
#include "gps.h"
#include "usb.h"
#include "starbot.h"

using namespace std;

#define STARBOT_MAX_HISTORY			  5
#define STARBOT_HISTORY_NB_CHAR_X	  73

/* StarBot Constants */

char * starbot_history[STARBOT_MAX_HISTORY];

gps * gps_sensor;
compass * compass_sensor;
starbot * starbot_instance;

void console_log( char * history_item ) {
	char *str_local = (char*)malloc(sizeof(char)*STARBOT_HISTORY_NB_CHAR_X);
	
	free(starbot_history[STARBOT_MAX_HISTORY - 1]);

	for(int i = STARBOT_MAX_HISTORY - 1; i > 0; i--) {
		starbot_history[i] = starbot_history[i - 1];
	}

	strcpy(str_local, history_item);

	starbot_history[0] = str_local;
}

int kbhit(void) {
  struct termios oldt, newt;
  int ch;
  int ch1;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF) {
	ch1 = ch;
    //ungetc(ch, stdin);
    return ch1;
  }
 
  return 0;
}

bool find_north( void )
{
	compass_sensor->update();
	
	float b = compass_sensor->bearing + starbot_instance->magneticDeclination;
	bool run = false;
	
	while( b < -0.5f || b > 0.5f)
	{
		run = true;
		
		if(b < 0)
		{
			
			if(b > -5)
			{
				starbot_instance->PanSteps(-3000, 1);
			}
			else
			{
				starbot_instance->PanDegrees( -3000, abs((int)b) );
			}
		}
		else
		{
			if( b < 5 )
			{
				starbot_instance->PanSteps(3000, 1);
			}
			else
			{
				starbot_instance->PanDegrees( 3000, abs((int)b));
			}
		}
		
		for(int i = 0; i < 100; i++)
		{
			compass_sensor->update();
			b = compass_sensor->bearing + starbot_instance->magneticDeclination;
		}
	}
	
	return run;
}

int main( void ) {

	time_t rawtime;
	struct tm * timeinfo;
	char * now;
	char buf[STARBOT_HISTORY_NB_CHAR_X];
	char cmd;

	starbot_instance = new starbot();
	starbot_instance->start();

	gps_sensor = new gps();
	console_log( "** Initializing GPS." );
	if(!gps_sensor->start()) {
		console_log( "** No GPSD running." );
	}
	
	compass_sensor = new compass();
	if(!compass_sensor->start()) {
		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Failed to open i2c bus.");
		console_log( (char *)buf );
	}

printf("\033[2J\033[?25l");

	/* Enter Control Loop */
	while(1) {
		time( &rawtime );
		timeinfo = localtime( &rawtime );
		now = asctime( timeinfo );

		now[strlen(now) - 1] = '\0';

		/* Reset Terminal Output */
		printf("\033[0;0H");
		printf("┌───────────────────────────────────────────────────────────────────────────┐\n\r");
		printf("│ StarBot by GalacticSoft                          %s │\n\r", now);
		printf("╞═══════MENU════════╤═══════════════════════EV314═══════════════════════════╡\n\r");
		printf("│                   │ Brick ID: %s                                │\n\r", EV314_EXPECTED_SERIAL);
		printf("│ 1) Main Menu      ╞═══════════════════════ GPS ═══════════════════════════╡\n\r");	
		
		if(gps_sensor->gps_fix) {
			if(gps_sensor->gps_fix == MODE_2D) {
				printf("│ 2) Motors         │ FIX: 2D FIX                               SATS: %2d/%2d │\n\r", gps_sensor->gps_use, gps_sensor->gps_sat);
			} else if(gps_sensor->gps_fix == MODE_3D) {
				printf("│ 2) Motors         │ FIX: 3D FIX                               SATS: %2d/%2d │\n\r", gps_sensor->gps_use, gps_sensor->gps_sat);
			}

			printf("│ 3) Sensors        │ LAT: %3d° %2d' %2.3f\" %c       LON: %3d° %2d' %2.3f\" %c │\n\r", 
				gps_sensor->latitude_degrees( ),  gps_sensor->latitude_minutes( ),  gps_sensor->latitude_seconds( ),  gps_sensor->gps_lat >= 0 ? 'N' : 'S',
				gps_sensor->longitude_degrees( ), gps_sensor->longitude_minutes( ), gps_sensor->longitude_seconds( ), gps_sensor->gps_lon >= 0 ? 'E' : 'W'  );

			printf("│ 4) Battery        │ Bearing: %3.2f° %c         INC: %3.2f°      DEC: %3.2f° │\n\r", 
				fabs(compass_sensor->bearing), compass_sensor->bearing < 0 ? 'W' : 'E', gps_sensor->gps_alt, starbot_instance->magneticDeclination);
			printf("│ 5) GPS            │ True Heading: %3.2f° %c   ALT: %3.2fm STR: %5.3fnT │\n\r",
				fabs(compass_sensor->bearing + starbot_instance->magneticDeclination), (compass_sensor->bearing + starbot_instance->magneticDeclination) > 0 ? 'E' : 'W', starbot_instance->magneticInclination, starbot_instance->fieldStrength);

		} else {
			printf("│ 2) Motors         │ FIX: NO FIX                                           │\n\r");
			printf("│ 3) Sensors        │                                                       │\n\r");
			printf("│ 4) Battery        │                                                       │\n\r");
			printf("│ 5) GPS            │                                                       │\n\r");
		}
		
		printf("│ 6) Camera         ╞═══════════════════════════════════════════════════════╡\n\r");
		printf("│ 7) Manual         │                                                       │\n\r");
		printf("│ 8) Settings       │                                                       │\n\r");
		printf("│ Q) Quit           │                                                       │\n\r");
		printf("│                   │                                                       │\n\r");
		printf("├───────────────────┴───────────────────────────────────────────────────────┤\n\r");

		for(int i = 0; i < STARBOT_MAX_HISTORY; i++) {
			snprintf( (char*)buf, STARBOT_HISTORY_NB_CHAR_X, starbot_history[i]);
			
			printf("│ %-73s │\n\r", buf);
		}

		printf("└───────────────────────────────────────────────────────────────────────────┘\n\r");
		
		cmd = (char)kbhit();

		if(cmd != (char)0) {
			snprintf( (char*)buf, STARBOT_HISTORY_NB_CHAR_X, "%c", cmd);

			console_log(buf);

			if(cmd == 'F' || cmd == 'f')
			{
				
				while(find_north()) {
					usleep(5 * 1000 * 1000);
				}
			}
			
			if(cmd == 'Q' || cmd == 'q') {
				break;
			}

			if(cmd == 'G' || cmd == 'g') {
				starbot_instance->CapturePanorama(3, 6);
			}
			
			// Tilt Up
			if(cmd == 'W' || cmd == 'w') {
				starbot_instance->TiltDegrees(3500, 1);
			}

			// Tilt Down
			if(cmd == 'S' || cmd == 's') {
				starbot_instance->TiltDegrees(-3500, 1);
			}

			// Pan Left Up
			if(cmd == 'A' || cmd == 'a') {
				starbot_instance->PanDegrees(-4500, 1);
			}

			// Tilt Down
			if(cmd == 'D' || cmd == 'd') {
				starbot_instance->PanDegrees(4500, 1);
			}

			if(cmd == 'P' || cmd == 'p') {
				starbot_instance->CaptureImage();
			}
		}
	
		printf("\033[2K");

		starbot_instance->update_gps(gps_sensor);
		compass_sensor->update();
	}

	printf("\033[2J\033[0;0H\033[?25h");

	if (!starbot_instance->stop()) {
		printf("** Error %d while closing USB device.\n", ret);
	}

	if(!gps_sensor->stop()) {
		printf("** Error while closing GPS.\n");
	}	
	delete gps_sensor;
	delete compass_sensor;
	delete starbot_instance;

	return 0;	
}

