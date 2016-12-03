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

	/* Initialize i2c compass */
	if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		//snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Failed to open i2c bus.");
		//console_log( (char *)buf );
		return 0;
	}

	/* initialise HMC5883L */

	select_i2c_device(i2c_fd, HMC5883L_I2C_ADDR, "HMC5883L");

	write_to_i2c(i2c_fd, 0x01, 32);
	write_to_i2c(i2c_fd, 0x02, 0);

	return 1;
}

int compass::update()
{
	float angle = 0;
	unsigned char i2c_buf[16];
	i2c_buf[0] = 0x03;

	if ((write(i2c_fd, i2c_buf, 1)) != 1) {
		// Send the register to read from
		//fprintf(stderr, "Error writing to i2c slave\n");
		return 0;
	}

	if (read(i2c_fd, i2c_buf, 6) != 6) {
		//fprintf(stderr, "Unable to read from HMC5883L\n");
		return 0;
	}
	else {
		short x = (i2c_buf[0] << 8) | i2c_buf[1];
		short y = (i2c_buf[4] << 8) | i2c_buf[5];
		//short z = (i2c_buf[2] << 8) | i2c_buf[3];

		angle = atan2(y, x) * 180 / M_PI;
	}

	//if(init) {
	//	state = kalman_init( 0.025f, 16, 1, angle );
	//	init = false;
	//}

	//kalman_update( &state, angle );

	if (angle < 0)
		angle += 360;

	bearing = angle; // state.x;

	return 1;
}

bool compass::select_i2c_device(int fd, int addr, char * name) {
	//char buf[STARBOT_HISTORY_NB_CHAR_X];

	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		//snprintf( (char *)buf, STARBOT_HISTORY_NB_CHAR_X, "%s not present\n", name);
		//console_log( (char *)buf );

		return false;
	}

	return true;
}

bool compass::write_to_i2c(int fd, int reg, int val) {
	char buf[2];
	//char log[STARBOT_HISTORY_NB_CHAR_X];

	buf[0] = reg;
	buf[1] = val;

	if (write(fd, buf, 2) != 2) {
		//snprintf( (char *)log, STARBOT_HISTORY_NB_CHAR_X, "Can't write to ADXL345\n");
		//console_log( (char *)log );
		return false;
	}

	return true;
}

compass_point compass::get_compass_point() {
	if (bearing >= compass_points[MIN_COMPASS].min || bearing <= compass_points[MIN_COMPASS].max) {
		return compass_points[MIN_COMPASS];
	}

	for (int i = MIN_COMPASS + 1; i < MAX_COMPASS; i++) {
		if (bearing >= compass_points[i].min && bearing <= compass_points[i].max) {
			return compass_points[i];
		}
	}
}

float compass::get_compass_point_variance(compass_point cp) {
	return bearing - cp.mid;
}

