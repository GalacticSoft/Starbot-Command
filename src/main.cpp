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
#include "starbot.h"

using namespace std;

#define STARBOT_MAX_HISTORY			  5
#define STARBOT_HISTORY_NB_CHAR_X	  73

/* StarBot Constants */

char * starbot_history[STARBOT_MAX_HISTORY];

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
	starbot_instance->update();
	
	float b = starbot_instance->bearing() + starbot_instance->declination();
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
			starbot_instance->update();
			b = starbot_instance->bearing() + starbot_instance->declination();
		}
	}
	
	return run;
}

int main( void ) {
	int ret = 0;
	time_t rawtime;
	struct tm * timeinfo;
	char * now;
	char buf[STARBOT_HISTORY_NB_CHAR_X];
	char cmd;

	starbot_instance = new starbot();
	starbot_instance->start();

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
		printf("│                   │ Brick ID: %s                                │\n\r", starbot_instance->expected_serial());
		printf("│ 1) Main Menu      ╞═══════════════════════ GPS ═══════════════════════════╡\n\r");	
		
		if(starbot_instance->fix()) {
			if(starbot_instance->fix() == MODE_2D) {
				printf("│ 2) Motors         │ FIX: 2D FIX                               SATS: %2d/%2d │\n\r", starbot_instance->sats_used(), starbot_instance->sats_view());
			} else if(starbot_instance->fix() == MODE_3D) {
				printf("│ 2) Motors         │ FIX: 3D FIX                               SATS: %2d/%2d │\n\r", starbot_instance->sats_used(), starbot_instance->sats_view());
			}

			printf("│ 3) Sensors        │ LAT: %3d° %2d' %2.3f\" %c       LON: %3d° %2d' %2.3f\" %c │\n\r", 
				starbot_instance->latitude_degrees( ), starbot_instance->latitude_minutes( ), starbot_instance->latitude_seconds( ), starbot_instance->latitude() >= 0 ? 'N' : 'S',
				starbot_instance->longitude_degrees( ), starbot_instance->longitude_minutes( ), starbot_instance->longitude_seconds( ), starbot_instance->longitude() >= 0 ? 'E' : 'W'  );

			printf("│ 4) Battery        │ Bearing: %3.2f° %c         INC: %3.2f°      DEC: %3.2f° │\n\r", 
				fabs(starbot_instance->bearing()), starbot_instance->bearing() < 0 ? 'W' : 'E', starbot_instance->altitude(), starbot_instance->declination());
			printf("│ 5) GPS            │ True Heading: %3.2f° %c   ALT: %3.2fm STR: %5.3fnT │\n\r",
				fabs(starbot_instance->bearing() + starbot_instance->declination()), (starbot_instance->bearing() + starbot_instance->declination()) > 0 ? 'E' : 'W', starbot_instance->inclination(), starbot_instance->field_strength());

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
			if(cmd == 'a') {
				starbot_instance->PanDegrees(-4500, 1);
			} else if (cmd == 'A') {
				starbot_instance->PanDegrees(-4500, 10);
			}

			// Pan Right
			if(cmd == 'd') {
				starbot_instance->PanDegrees(4500, 1);
			} else if (cmd == 'D') {
				starbot_instance->PanDegrees(4500, 10);
			}

			if(cmd == 'P' || cmd == 'p') {
				starbot_instance->CaptureImage();
			}
		}
	
		printf("\033[2K");

		starbot_instance->update();

	}

	printf("\033[2J\033[0;0H\033[?25h");

	if ((ret = starbot_instance->stop())) {
		printf("** Error %d while closing USB device.\n", ret);
	}

	starbot_instance->stop();

	//if(!gps_sensor->stop()) {
	//	printf("** Error while closing GPS.\n");
	//}	
	delete starbot_instance;

	return 0;	
}

