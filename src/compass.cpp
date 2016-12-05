#include <sys/ioctl.h> // I2C
#include <sys/types.h> // I2C
#include <sys/stat.h> // I2C
#include <linux/i2c-dev.h> // I2C
#include <unistd.h> 
#include <fcntl.h>
#include "compass.h"
#include "math.h"

using namespace std;

int compass::start()
{
	init = true;
	filter = new kalman_filter();

	/* Initialize i2c compass */
	if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		return COMPASS_ERROR_OPEN;
	}

	/* initialise HMC5883L */
	select_i2c_device(i2c_fd, HMC5883L_I2C_ADDR, "HMC5883L");

	write_to_i2c(i2c_fd, 0x01, 32);
	write_to_i2c(i2c_fd, 0x02, 0);

	return COMPASS_ERROR_NONE;
}

int compass::update(float declination)
{
	unsigned char i2c_buf[16];
	i2c_buf[0] = 0x03;

	if ((write(i2c_fd, i2c_buf, 1)) != 1) {
		return COMPASS_ERROR_WRITE;
	}

	if (read(i2c_fd, i2c_buf, 6) != 6) {
		return COMPASS_ERROR_READ;
	}
	else {
		short x = (i2c_buf[0] << 8) | i2c_buf[1];
		short y = (i2c_buf[4] << 8) | i2c_buf[5];
		//short z = (i2c_buf[2] << 8) | i2c_buf[3];

		// Calculate radians
		radians = atan2(y, x);
		
		// Filter Radians
		filtered_radians = filter->update(radians);
		
		// Convert filtered radians to a range of -180 to 180
		bearing = filtered_radians * 180 / M_PI;

		// Convert bearing to 0 to 360
		degrees = bearing < 0 ? bearing + 360 : bearing;

		true_bearing = bearing + declination;

		true_degrees = true_bearing < 0 ? true_bearing + 360 : true_bearing;
	}

	return COMPASS_ERROR_NONE;
}

bool compass::select_i2c_device(int fd, int addr, char * name) {
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		return false;
	}

	return true;
}

bool compass::write_to_i2c(int fd, int reg, int val) {
	char buf[2];

	buf[0] = reg;
	buf[1] = val;

	if (write(fd, buf, 2) != 2) {
		return false;
	}

	return true;
}

compass_point compass::get_compass_point() {
	if (degrees >= compass_points[MIN_COMPASS].min || degrees <= compass_points[MIN_COMPASS].max) {
		return compass_points[MIN_COMPASS];
	}

	for (int i = MIN_COMPASS + 1; i < MAX_COMPASS; i++) {
		if (degrees >= compass_points[i].min && degrees <= compass_points[i].max) {
			return compass_points[i];
		}
	}

	return compass_points[MIN_COMPASS];
}

float compass::get_compass_point_variance(compass_point point) {
	return degrees - point.mid;
}

