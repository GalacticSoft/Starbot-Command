#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   //_getch
#include "ev314.h"
#include "usb.h"
#include "gps.h"
#include "starbot.h"
#include <string.h>
//#include <GeographicLib/MagneticModel.hpp> // Magnetic Model
#include <time.h>
#include "wmm.h"

//using namespace GeographicLib;

void starbot::start()
{
	magnetic_model = new wmm();

	/* Initializing control structure */
	memset(&ev314_control, 0, sizeof(struct ev314_control_struct));

	//snprintf((char*)buf, STARBOT_HISTORY_NB_CHAR_X, "** Looking for device with ID=%s", EV314_EXPECTED_SERIAL);
	//console_log((char*)buf);

	if (EV314_init()) {
		//console_log("** Error while initializing libusb.");
	}

	/* Open EV3 Device */
	if (!(EV314_hdl = EV314_find_and_open(EV314_EXPECTED_SERIAL))) {
		////console_log("** Error while looking for an EV3 USB device.");
	}
	else {
		////snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Device %s found!", EV314_EXPECTED_SERIAL);
		//console_log((char *)buf);
	}

	/* Initialize encoders */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control)))) {
		//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while resetting encoders.", ret);
		//console_log((char *)buf);
	}
}

void starbot::update()
{

}

int starbot::stop()
{
	return EV314_close(EV314_hdl);
}

void starbot::CaptureImage() {
	char buf[50];

	snprintf((char*)buf, 50, "raspistill  -n -t 1 -o Images/image%d.jpg", imageCount++);

	system((char*)buf);
}

void starbot::PanSteps(int pow, int steps) {
	//char buf[STARBOT_HISTORY_NB_CHAR_X];

	/* Initialize encoders */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		//printf("** Error %d while resetting encoders.\n", ret);

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
			//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
			//console_log((char *)buf);
		}

		/* Get response */

		memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

		if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state)))) {
			//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
			//console_log((char *)buf);
		}

		//ev314_profiling_stop();

		/* Check response */

		if (ev314_state.magic != EV314_MAGIC) {
			//console_log("** Received packet with bad magic number.");
		}

		/* Print response */

		//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "Encoder values> Pan:%d/%d Current: %d", ev314_state.motor_angle[0], ev314_state.motor_angle[3], ev314_state.battery_current);

		//console_log((char *)buf);

		if (abs(ev314_state.motor_angle[0]) >= steps)
			ev314_control.motor_power[0] = 0;

		if (abs(ev314_state.motor_angle[3]) >= steps)
			ev314_control.motor_power[3] = 0;

		if (ev314_control.motor_power[0] == 0 && ev314_control.motor_power[3] == 0)
			break;
	}
}

void starbot::TiltSteps(int pow, int steps)
{
	//char buf[STARBOT_HISTORY_NB_CHAR_X];

	/* Initialize encoders */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		//printf("** Error %d while resetting encoders.\n", ret);

	/* Initilize control packet */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[1] = pow; // 10000;

										/* Entering status polling loop */

	for (;;) {

		/* Send control */

		//ev314_profiling_start();

		if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control))))
		{
			//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
			//console_log((char *)buf);
		}

		/* Get response */

		memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

		if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state))))
		{
			//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
			//console_log((char *)buf);
		}

		//ev314_profiling_stop();

		/* Check response */

		if (ev314_state.magic != EV314_MAGIC)
		{
			//console_log("** Received packet with bad magic number.");
		}

		/* Print response */

		//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "Encoder values> Tilt:%d Current: %d", ev314_state.motor_angle[1], ev314_state.battery_current);

		//console_log((char *)buf);

		if (abs(ev314_state.motor_angle[1]) >= steps)
		{
			ev314_control.motor_power[1] = 0;
			break;
		}
	}
}

void starbot::PanArcSeconds(int power, int arcSeconds)
{
	int steps = arcSeconds * STEPS_PER_ARCSEC_PAN;

	PanSteps(power, steps);
}

void starbot::PanDegrees(int power, int degrees)
{
	int steps = degrees * STEPS_PER_DEGREE_PAN;

	PanSteps(power, steps);
}

void starbot::TiltArcSeconds(int power, int arcSeconds)
{
	int steps = arcSeconds * STEPS_PER_ARCSEC_TILT;

	TiltSteps(power, steps);
}

void starbot::TiltDegrees(int power, int degrees)
{
	int steps = degrees * STEPS_PER_DEGREE_TILT;

	TiltSteps(power, steps);
}

void starbot::CapturePanorama(int layers, int images)
{
	int panPower = 3500;
	int tiltPower = 3500;

	PanDegrees(panPower * -1, 270 / 2);
	TiltDegrees(tiltPower * -1, 90 / 2);

	for (int y = 0; y < layers; y++)
	{
		for (int x = 0; x < images; x++)
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

	TiltDegrees(tiltPower * -1, 90 / 2);
	PanDegrees(panPower, 270 / 2);
}

void starbot::update_gps(gps* gps_sensor) {
	int ret = 0;
	//char buf[STARBOT_HISTORY_NB_CHAR_X];
	//double Bx, By, Bz;
	//double H, F, D, I;
	//time_t t = time(NULL);
	//MagneticModel mag("emm2015"); //wmm2015
	//tm* timePtr = localtime(&t);

	/* Initialize Control Structure */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_GPS;
	ev314_control.gps_fix = 0;
	ev314_control.gps_lon = 0;
	ev314_control.gps_lat = 0;
	ev314_control.gps_alt = 0;
	ev314_control.gps_sat = 0;
	ev314_control.gps_use = 0;

	if (!gps_sensor->update()) {
		//console_log("** Error Reading GPS.");
	}
	else {
		/* Fix Obtained, Set Values. */
		ev314_control.gps_fix = gps_sensor->gps_fix;
		ev314_control.gps_lon = gps_sensor->gps_lon;
		ev314_control.gps_lat = gps_sensor->gps_lat;
		ev314_control.gps_alt = gps_sensor->gps_alt;

		ev314_control.gps_sat = gps_sensor->gps_sat;
		ev314_control.gps_use = gps_sensor->gps_use;

		magnetic_model.update(gps_sensor->gps_lat, gps_sensor->gps_lon, gps_sensor->gps_alt);

		// Use World Magnetic Model to determine magnetic declination.
		//mag(timePtr->tm_year + 1900, gps_sensor->gps_lat, gps_sensor->gps_lon, gps_sensor->gps_alt, Bx, By, Bz);
		//MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

		magneticDeclination = magnetic_model->declination();
		magneticInclination = magnetic_model->inclination();
		fieldStrength = magnetic_model->strength();
	}

	/* Send control */
	//ev314_profiling_start();

	if ((ret = EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control)))) {
		//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while sending packet.", ret);
		//console_log((char *)buf);
	}

	/* Get response */
	memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

	if ((ret = EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state)))) {
		//snprintf((char *)buf, STARBOT_HISTORY_NB_CHAR_X, "** Error %d while receiving packet.", ret);
		//console_log((char *)buf);
	}

	ev314_profiling_stop();

	/* Check response */
	if (ev314_state.magic != EV314_MAGIC) {
		//console_log("** Received packet with bad magic number.");
	}
}