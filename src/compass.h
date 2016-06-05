
#ifndef __COMPASS_H__  
#define __COMPASS_H__

#include "kalman.h"

#define HMC5883L_I2C_ADDR 0x1E;

class compass
{
private:
	bool init;
	int i2c_fd;
	
	kalman_state state; 
	
	bool select_i2c_device(int fd, int addr, char * name);
	bool write_to_i2c(int fd, int reg, int val);

public:

	float bearing;
	
	double magneticDeclination;
	double magneticInclination;
	double fieldStrength;

	/*
	* start: Start GPS reciever.
	*
	*/
	int start();
	
	
	/*
	* update: Update values from GPS reciever
	*
	*/
	int update();
	
	float compass_angle();
};

#endif