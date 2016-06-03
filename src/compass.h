
#ifndef __COMPASS_H__  
#define __COMPASS_H__

#include <sys/ioctl.h> // I2C
#include <sys/types.h> // I2C
#include <sys/stat.h> // I2C
#include <linux/i2c-dev.h> // I2C
#include <unistd.h> 
#include <fcntl.h>

using namespace std;

typedef struct {
  float q; //process noise covariance
  float r; //measurement noise covariance
  float x; //value
  float p; //estimation error covariance
  float k; //kalman gain
} kalman_state;

class compass
{
private:
	bool init;
	int i2c_fd;
	const int HMC5883L_I2C_ADDR = 0x1E;
	
	kalman_state state; 
	
	kalman_state kalman_init(float q, float r, float p, float intial_value);
	void kalman_update(kalman_state* s, float m);
	
	bool select_i2c_device(int fd, int addr, char * name);
	bool write_to_i2c(int fd, int reg, int val);

public:

	float bearing = 0.0f;
	
	double magneticDeclination = 0;
	double magneticInclination = 0;
	double fieldStrength = 0;

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