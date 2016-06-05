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
	int imageCount;

public:
	double magneticDeclination;
	double magneticInclination;
	double fieldStrength;

	void start();

	void update();

	void stop();

	void CaptureImage();

	void PanSteps(int pow, int steps);

	void TiltSteps(int pow, int steps);

	void PanArcSeconds(int power, int arcSeconds);

	void PanDegrees(int power, int degrees);

	void TiltArcSeconds(int power, int arcSeconds);

	void TiltDegrees(int power, int degrees);

	void CapturePanorama(int layers, int images);

	void update_gps(gps* gps_sensor);
};

#endif
