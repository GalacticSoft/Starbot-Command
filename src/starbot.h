#ifndef __STARBOT_H__
#define __STARBOT_H__



#define STEPS_PER_ARCSEC_PAN 2 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_PAN 112 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_ARCSEC_TILT 1 // Using a 1:40 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_TILT 80 // Using a 1:40 gear ratio with double encoder resolution.

class starbot
{
private:
	EV314_error_t 					ret;
	struct ev314_control_struct		ev314_control;
	struct ev314_state_struct		ev314_state;
	struct libusb_device_handle *EV314_hdl;
	int imageCount = 1;

public:

	void start();

	void update();

	void stop();
};

#endif
