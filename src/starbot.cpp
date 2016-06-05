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
#include <GeographicLib/MagneticModel.hpp> // Magnetic Model
#include "ev314.h" // EV314 Firmware
#include "gps.h"
#include "compass.h"

using namespace std;
using namespace GeographicLib;

double magneticDeclination = 0;
double magneticInclination = 0;
double fieldStrength = 0;





/* EV314 Constants */

	#define EV314_EXPECTED_SERIAL		  "0016534957ad"
	#define EV314_TEST_NB_ITER			  27000000
	#define EV314_USB_TIMEOUT             1000    // Milliseconds
	#define EV314_INTERFACE_NUMBER        0
	#define EV314_CONFIGURATION_NB        1
	#define EV314_EP_OUT                  0x01
	#define EV314_EP_IN                   0x81
	#define EV314_PACKET_SIZE			  0x400
	#define EV314_RESET_RESPONSE_SIZE     5
	#define EV314_POWER_RESPONSE_SIZE     5
	#define EV314_VENDOR_LEGO             0x0694
	#define EV314_PRODUCT_EV3             0x0005
	#define EV314_MAX_RETRY               3
	#define EV314_RETRY_TIMEOUT           200000


/* EV314 Error codes */

#define EV314_OK                      0
#define EV314_USB_ERROR				  1
#define EV314_NOT_PRESENT             2
#define EV314_CONFIGURATION_ERROR     3
#define EV314_IN_USE                  4
#define EV314_USB_WRITE_ERROR         5
#define EV314_USB_READ_ERROR          6
#define EV314_USB_PARTIAL_TRANS		  7
#define EV314_USB_OVERFLOW			  9
#define EV314_BYTECODE_ERROR          10

/* EV314 PreProcessor Macros */

#define EV314_PROFILING_ON			  // Comment to deactivate profiling






/* StarBot Constants */

#define STARBOT_MAX_HISTORY			  5
#define STARBOT_HISTORY_NB_CHAR_X	  73

#define STEPS_PER_ARCSEC_PAN 2 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_PAN 112 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_ARCSEC_TILT 1 // Using a 1:40 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_TILT 80 // Using a 1:40 gear ratio with double encoder resolution.

//const int HMC5883L_I2C_ADDR = 0x1E;




typedef int                     EV314_error_t;

struct timespec					profiling_start;

EV314_error_t 					ret;
struct ev314_control_struct		ev314_control;
struct ev314_state_struct		ev314_state;

struct libusb_device_handle *EV314_hdl;






char * starbot_history[STARBOT_MAX_HISTORY];

gps * gps_sensor;
compass * compass_sensor;
/*
 * ev314_profiling_start: start timer
 * 
 */
void ev314_profiling_start( void )	{
#ifdef EV314_PROFILING_ON
	clock_gettime( CLOCK_MONOTONIC, &profiling_start );
#endif
}

/*
 * ev314_profiling_stop: stop timer and print time
 * 
 */
void ev314_profiling_stop( void )	{
#ifdef EV314_PROFILING_ON
	struct timespec						profiling_stop;
	
	clock_gettime( CLOCK_MONOTONIC, &profiling_stop );
	
	fprintf( 	stderr, "** Profiling duration: %d us.\n",
						(int)( ( profiling_stop.tv_sec - profiling_start.tv_sec ) * 1000000
							 + ( profiling_stop.tv_nsec - profiling_start.tv_nsec ) / 1000 ) );

#endif
}

/*
 * USB API.
 * 
 */
EV314_error_t EV314_init( void )  {
	int status;
  
  status = libusb_init( NULL );
  
	if ( status ) 
		return EV314_USB_ERROR;

  return EV314_OK;
}


struct libusb_device_handle* EV314_find_and_open( char* expected_serial )  {
	libusb_device 									*dev, **devs;
	struct libusb_device_descriptor	desc;
	struct libusb_device_handle			*EV314_hdl = NULL;
	int															i, status, transfered;
  unsigned char	                  tmpbuf[EV314_PACKET_SIZE];
  char														serial[EV314_LENGTH_SERIAL+1];
  
	if ( libusb_get_device_list( NULL, &devs ) < 0 )
		return NULL;
	
	/* Go through device list loooking for an EV3 device */
	for ( i = 0; ( dev = devs[i] ) != NULL; i++ ) {

		status = libusb_get_device_descriptor( dev, &desc );

		if ( status >= 0 ) {
			if ( 	( desc.idVendor == EV314_VENDOR_LEGO ) &&
						( desc.idProduct == EV314_PRODUCT_EV3 ) )	{
				
				/* Open the device */
				status = libusb_open( dev, &EV314_hdl );
				if ( status < 0 )	{
					libusb_free_device_list( devs, 1 );
					return NULL;
				}
				
				/* Check the serial number */
				
				status = libusb_get_string_descriptor_ascii( EV314_hdl, desc.iSerialNumber, (unsigned char*)serial, sizeof( serial ) );
				if ( status == EV314_LENGTH_SERIAL ) {
					if ( strcmp( expected_serial, serial ) )	{
						libusb_close( EV314_hdl );
            EV314_hdl = NULL;
						continue;
					}
				}
				else {
					libusb_close( EV314_hdl ); 
          EV314_hdl = NULL;
					continue;
				}
				
				/* Detach possible kernel driver bound to interface */
  			libusb_detach_kernel_driver( EV314_hdl, EV314_INTERFACE_NUMBER );
  			
  			/* Claiming the interface */
				status = libusb_claim_interface( EV314_hdl, EV314_INTERFACE_NUMBER );
				if ( status )	{
					libusb_close( EV314_hdl );
					libusb_free_device_list( devs, 1 );
					return NULL;
				}
				
        /* Request a packet until getting a zero byte packet */
        do
        {
          status = libusb_bulk_transfer( EV314_hdl, EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT );
        } while ( ( status == 0 ) && ( transfered != 0 ) ); 
        
				libusb_free_device_list( devs, 1 );
        return EV314_hdl;
      }
		}
	}
	
	libusb_free_device_list( devs, 1 );
  return NULL;
}

EV314_error_t EV314_close( struct libusb_device_handle *EV314_hdl )  {
  
  if ( EV314_hdl == NULL )
    return EV314_CONFIGURATION_ERROR;
  
  libusb_release_interface( EV314_hdl, EV314_INTERFACE_NUMBER );
  libusb_close( EV314_hdl );
  EV314_hdl = NULL;
  libusb_exit( NULL );

  return EV314_OK;
}

EV314_error_t EV314_send_buf( struct libusb_device_handle *EV314_hdl, unsigned char *buf, int len )  {
	int	status, transfered = 0;
  
  if ( EV314_hdl == NULL )
    return EV314_CONFIGURATION_ERROR;
  
  if ( len > EV314_PACKET_SIZE )
  	return EV314_USB_OVERFLOW;
  
  status = libusb_interrupt_transfer( EV314_hdl, EV314_EP_OUT, buf, len, &transfered, EV314_USB_TIMEOUT );
  
  if ( status  )
    return EV314_USB_WRITE_ERROR;
  
  if ( transfered != len )
  	return EV314_USB_PARTIAL_TRANS;

  return EV314_OK;
}

EV314_error_t EV314_recv_buf( struct libusb_device_handle *EV314_hdl, unsigned char *buf, int len )  {
  int						i, status, transfered = 0;
  unsigned char	tmpbuf[EV314_PACKET_SIZE];
  
  if ( EV314_hdl == NULL )
    return EV314_CONFIGURATION_ERROR;
  
  if ( len > EV314_PACKET_SIZE )
  	return EV314_USB_OVERFLOW;
  
  status = libusb_interrupt_transfer( EV314_hdl, EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT );

  if ( status  )
    return EV314_USB_READ_ERROR;
  
  if ( transfered != EV314_PACKET_SIZE )
  	return EV314_USB_PARTIAL_TRANS;
  
  for ( i = 0; i < len; i++ )
  	buf[i] = tmpbuf[i];

  return EV314_OK;
}











void console_log( char * history_item ) {
	char *str_local = (char*)malloc(sizeof(char)*STARBOT_HISTORY_NB_CHAR_X);
	
	free(starbot_history[STARBOT_MAX_HISTORY - 1]);

	for(int i = STARBOT_MAX_HISTORY - 1; i > 0; i--) {
		starbot_history[i] = starbot_history[i - 1];
	}

	strcpy(str_local, history_item);

	starbot_history[0] = str_local;
}

void PanSteps(int pow, int steps) {
	char buf[STARBOT_HISTORY_NB_CHAR_X];

	/* Initialize encoders */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		printf("** Error %d while resetting encoders.\n", ret);

	/* Initilize control packet */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[0] = pow; // 10000;
	ev314_control.motor_power[3] = pow; // 10000;

	/* Entering status polling loop */

	for (;;) {

		/* Send control */

		//ev314_profiling_start();

		if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control)))) {
			snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
			console_log( (char *)buf );
		}

		/* Get response */

		memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

		if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state)))) {
			snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
			console_log( (char *)buf );
		}

		//ev314_profiling_stop();

		/* Check response */

		if (ev314_state.magic != EV314_MAGIC) {
			console_log("** Received packet with bad magic number.");
		}

		/* Print response */

		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "Encoder values> Pan:%d/%d Current: %d", 							ev314_state.motor_angle[0],ev314_state.motor_angle[3], ev314_state.battery_current);

		console_log( (char *)buf );

		if (abs(ev314_state.motor_angle[0]) >= steps)
			ev314_control.motor_power[0] = 0; 

		if (abs(ev314_state.motor_angle[3]) >= steps)
			ev314_control.motor_power[3] = 0; 

		if (ev314_control.motor_power[0] == 0 && ev314_control.motor_power[3] == 0)
			break;
	}
}

void TiltSteps(int pow, int steps)
{
	char buf[STARBOT_HISTORY_NB_CHAR_X];

	/* Initialize encoders */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		printf("** Error %d while resetting encoders.\n", ret);

	/* Initilize control packet */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[1] = pow; // 10000;

	/* Entering status polling loop */

	for (;;)	{

		/* Send control */

		//ev314_profiling_start();

		if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		{
			snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
			console_log( (char *)buf );
		}

		/* Get response */

		memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

		if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state))))
		{
			snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
			console_log( (char *)buf );
		}

		//ev314_profiling_stop();

		/* Check response */

		if (ev314_state.magic != EV314_MAGIC)
		{
			console_log("** Received packet with bad magic number.");
		}

		/* Print response */

		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "Encoder values> Tilt:%d Current: %d", 							ev314_state.motor_angle[1], ev314_state.battery_current);

		console_log( (char *)buf );
		
		if (abs(ev314_state.motor_angle[1]) >= steps)
		{
			ev314_control.motor_power[1] = 0; 
			break;
		}
	}
}

void PanArcSeconds(int power, int arcSeconds)
{
	int steps = arcSeconds * STEPS_PER_ARCSEC_PAN;

	PanSteps(power, steps);
}

void PanDegrees(int power, int degrees)
{
	int steps = degrees * STEPS_PER_DEGREE_PAN;

	PanSteps(power, steps);
}

void TiltArcSeconds(int power, int arcSeconds)
{
	int steps = arcSeconds * STEPS_PER_ARCSEC_TILT;

	TiltSteps(power, steps);
}

void TiltDegrees(int power, int degrees)
{
	int steps = degrees * STEPS_PER_DEGREE_TILT;

	TiltSteps(power, steps);
}

void update_gps( ) {
	int ret = 0;
	char buf[STARBOT_HISTORY_NB_CHAR_X];
	double Bx, By, Bz;
	double H, F, D, I;
	time_t t = time(NULL);
	MagneticModel mag("emm2015"); //wmm2015
    tm* timePtr = localtime(&t);

	/* Initialize Control Structure */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_GPS;
	ev314_control.gps_fix = 0;
	ev314_control.gps_lon = 0;
	ev314_control.gps_lat = 0;
	ev314_control.gps_alt = 0;
	ev314_control.gps_sat = 0;
	ev314_control.gps_use = 0;

	if(!gps_sensor->update()) {
		console_log("** Error Reading GPS.");
	} else {
		/* Fix Obtained, Set Values. */
		ev314_control.gps_fix = gps_sensor->gps_fix;
		ev314_control.gps_lon = gps_sensor->gps_lon;
		ev314_control.gps_lat = gps_sensor->gps_lat;
		ev314_control.gps_alt = gps_sensor->gps_alt;

		ev314_control.gps_sat = gps_sensor->gps_sat;
		ev314_control.gps_use = gps_sensor->gps_use;

		// Use World Magnetic Model to determine magnetic declination.
		mag(timePtr->tm_year + 1900, gps_sensor->gps_lat, gps_sensor->gps_lon, gps_sensor->gps_alt, Bx, By, Bz);
		MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

		magneticDeclination = D;
		magneticInclination = I;
		fieldStrength = F;
	}
	
	/* Send control */
	//ev314_profiling_start();

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control)))) {
		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
		console_log( (char *)buf );
	}

	/* Get response */
	memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

	if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state)))) {
		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
		console_log( (char *)buf );
	}

	ev314_profiling_stop();

	/* Check response */
	if (ev314_state.magic != EV314_MAGIC) {
		console_log( "** Received packet with bad magic number." );
	}
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
    ungetc(ch, stdin);
    return ch1;
  }
 
  return 0;
}
int imageCount = 1;

void CaptureImage() {
	char buf[STARBOT_HISTORY_NB_CHAR_X];

	snprintf( (char*)buf, STARBOT_HISTORY_NB_CHAR_X, "raspistill  -n -t 1 -o Images/image%d.jpg", imageCount++);

	system( (char*)buf );
}

void CapturePanorama(int layers, int images)
{
	int panPower = 3500;
	int tiltPower = 3500;

	PanDegrees(panPower * -1, 270/2);
	TiltDegrees(tiltPower * -1, 90/2);

	for(int y = 0; y < layers; y++)
	{
		for(int x = 0; x < images; x++)
		{
			CaptureImage();
			PanDegrees(panPower, 270 / images);
			
			// Wait to Stabilize.
			usleep(5 * 1000 * 1000);
		}
	
		CaptureImage();

		TiltDegrees(tiltPower, 90 / layers);

		// Wait to Stabilize.
		usleep(5 * 1000 * 1000);
		
		panPower *= -1;
	}

	TiltDegrees(tiltPower * -1, 90/2);
	PanDegrees(panPower, 270/2);
}

bool find_north( void )
{
	compass_sensor->update();
	
	float b = compass_sensor->bearing + magneticDeclination;
	bool run = false;
	
	while( b < -0.5f || b > 0.5f)
	{
		run = true;
		
		if(b < 0)
		{
			
			if(b > -5)
			{
				PanSteps(-3000, 1);
			}
			else
			{
				PanDegrees( -3000, abs((int)b) );
			}
		}
		else
		{
			if( b < 5 )
			{
				PanSteps(3000, 1);
			}
			else
			{
				PanDegrees( 3000, abs((int)b));
			}
		}
		
		for(int i = 0; i < 100; i++)
		{
			compass_sensor->update();
			b = compass_sensor->bearing + magneticDeclination;
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

	/* Initializing control structure */
	memset(&ev314_control, 0, sizeof(struct ev314_control_struct));

	snprintf( (char*)buf, STARBOT_HISTORY_NB_CHAR_X, "** Looking for device with ID=%s", EV314_EXPECTED_SERIAL);
	console_log( (char*)buf );

	if (EV314_init()) {
		console_log("** Error while initializing libusb.");
	}
	
	/* Open EV3 Device */
	if (!(EV314_hdl = EV314_find_and_open(EV314_EXPECTED_SERIAL))) {
		console_log( "** Error while looking for an EV3 USB device." );
	} else {
		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Device %s found!", EV314_EXPECTED_SERIAL);
		console_log( (char *)buf );
	}

	/* Initialize encoders */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control)))) {
		snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while resetting encoders.", ret);
		console_log( (char *)buf );
	}

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
				fabs(compass_sensor->bearing), compass_sensor->bearing < 0 ? 'W' : 'E', ev314_control.gps_alt, magneticDeclination);
			printf("│ 5) GPS            │ True Heading: %3.2f° %c   ALT: %3.2fm STR: %5.3fnT │\n\r",
				fabs(compass_sensor->bearing + magneticDeclination), (compass_sensor->bearing + magneticDeclination) > 0 ? 'E' : 'W', magneticInclination, fieldStrength);

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
				CapturePanorama(3, 6);
			}
			
			// Tilt Up
			if(cmd == 'W' || cmd == 'w') {
				TiltDegrees(3500, 1);
			}

			// Tilt Down
			if(cmd == 'S' || cmd == 's') {
				TiltDegrees(-3500, 1);
			}

			// Pan Left Up
			if(cmd == 'A' || cmd == 'a') {
				PanDegrees(-4500, 1);
			}

			// Tilt Down
			if(cmd == 'D' || cmd == 'd') {
				PanDegrees(4500, 1);
			}

			if(cmd == 'P' || cmd == 'p') {
				CaptureImage();
			}
		}
	
		printf("\033[2K");

		update_gps();
		compass_sensor->update();
		
	}

	printf("\033[2J\033[0;0H\033[?25h");


	if ((ret = EV314_close(EV314_hdl))) {
		printf("** Error %d while closing USB device.\n", ret);
	}

	if(!gps_sensor->stop()) {
		printf("** Error while closing GPS.\n");
	}	
	delete gps_sensor;
	delete compass_sensor;
	
	return 0;	
}

