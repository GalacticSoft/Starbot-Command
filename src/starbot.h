#ifndef __STARBOT_H__
#define __STARBOT_H__

#include "ev314.h"
#include "wmm.h"
#include "compass.h"
#include "gps.h"

#define STEPS_PER_ARCSEC_PAN 2 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_PAN 112 // Using a 1:56 gear ratio with double encoder resolution.
#define STEPS_PER_ARCSEC_TILT 1 // Using a 1:40 gear ratio with double encoder resolution.
#define STEPS_PER_DEGREE_TILT 80 // Using a 1:40 gear ratio with double encoder resolution.

class starbot
{
private:
	int imageCount;

	float originX, originY;
	float currentX, currentY;
	float targetX, targetY;

	EV314_error_t 					ret;
	struct ev314_control_struct		ev314_control;
	struct ev314_state_struct		ev314_state;
	struct libusb_device_handle *EV314_hdl;


	gps* gps_sensor;
	wmm* magnetic_model;
	compass* compass_sensor;

public:
	int fix();
	int sats_used();
	int sats_view();
	double latitude();
	double longitude();
	double altitude();

	int latitude_degrees();
	int latitude_minutes();
	double latitude_seconds();

	int longitude_degrees();
	int longitude_minutes();
	double longitude_seconds();

	double declination();
	double inclination();
	double field_strength();

	double bearing();

	void start();

	void update();

	int stop();

	void CaptureImage();

	void PanSteps(int pow, int steps);

	void TiltSteps(int pow, int steps);

	void PanArcSeconds(int power, int arcSeconds);

	void PanDegrees(int power, int degrees);

	void TiltArcSeconds(int power, int arcSeconds);

	void TiltDegrees(int power, int degrees);

	void CapturePanorama(int layers, int images);

	void update_gps( );
};

#endif
