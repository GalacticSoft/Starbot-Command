/*
 * 	Definitions for ev314.c
 * 
 * 	JG, 25.9.2014
 */

#ifndef __EV314_H__
#define __EV314_H__

 /* EV314 Device Configuration */
#define EV314_MAGIC					0xff0fff0f
#define EV314_LENGTH_SERIAL			12		// Length of th BT serial number

#define EV314_NB_SENSORS			4
#define EV314_NB_MOTORS				4

#define EV314_MAX_CONTROL			10000		// Control signal ranges from -10000 to +10000

/* EV314 Servo Settings */
#define EV314_WATCHDOG_ON			0		// Watchdog is triggered.
#define EV314_WATCHDOG_OFF			1		// Watchdog is not triggerred. Control is active.

#define EV314_MANUAL_RELEASE_OFF	0		// Manual realease off
#define EV314_MANUAL_RELEASE_ON		1		// Manual release on: motors are floating.

/* EV314 Command Control Codes */
#define EV314_CMD_NO_CDM			0
#define EV314_CMD_RESET_ENC			1
#define EV314_CMD_HALT				2
#define EV314_CMD_CONTROL			3
#define EV314_CMD_MENU				4
#define EV314_CMD_GPS				5

/* EV314 Error Codes */
#define EV314_ERROR_FALSE			0
#define EV314_ERROR_USB_READ		1
#define EV314_ERROR_USB_WRITE		2
#define EV314_ERROR_USB_PACKET_SIZE	3
#define EV314_ERROR_USB_MAGIC		4
#define EV314_ERROR_UNKNOWN_CMD		5
#define EV314_ERROR_ENC_RESET		6
#define EV314_ERROR_SET_VOLTAGE		7
#define EV314_ERROR_START_MOTOR		8
#define EV314_ERROR_STOP_MOTOR		9

/* Starbot Menu Definitions */
#define STARBOT_MENU_HOME 						0 // Main Menu
#define STARBOT_MENU_MOTORS						1 // Detailed Motor Display
#define STARBOT_MENU_SENSORS					2 // Detailed Sensor Display
#define STARBOT_MENU_GPS						3 // Detailed GPS Display
#define STARBOT_MENU_BATTERY					4 // Detailed Battery Display
#define STARBOT_MENU_CAMERA						5 // Photo System Interactions
#define STARBOT_MENU_MAX						5 // Maximum Menu Value

struct ev314_control_struct	
{
	unsigned int				magic;
	unsigned char				cmd;
	int 						motor_power[EV314_NB_MOTORS];
	bool						motor_reset[EV314_NB_MOTORS];
	unsigned int				menu;
	int							gps_fix;
	float						gps_lon;
	float						gps_lat;
	float						gps_alt;
	int							gps_sat;
	int							gps_use;
	
};

struct ev314_state_struct	
{
	unsigned int				magic;
  	unsigned int				battery_voltage;
  	unsigned char				watchdog;
  	unsigned char				manual_release;
  	unsigned char				error;
  	int 						motor_power[EV314_NB_MOTORS];
  	int							motor_angle[EV314_NB_MOTORS];
  	int							battery_current;
	unsigned int				input_ADC[EV314_NB_SENSORS];
	unsigned long long			current_time;
	unsigned long long			control_time_stamp;
	char						serial[EV314_LENGTH_SERIAL];
	int							gps_fix;
	float						gps_lon;
	float						gps_lat;
	float						gps_alt;
	int							gps_sat;
	int							gps_use;
	unsigned int				menu;
};

#endif // !__EV314_H__