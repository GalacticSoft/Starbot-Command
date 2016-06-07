#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   //_getch
#include <string.h>
#include <time.h>
#include "starbot.h"

void starbot::start()
{
	gps_sensor = new gps();
	magnetic_model = new wmm();
	compass_sensor = new compass();

	gps_sensor->start();
	compass_sensor->start();

	/* Initializing control structure */
	memset(&ev314_control, 0, sizeof(struct ev314_control_struct));

	/* Initialize EV314 */
	EV314_init();

	/* Open EV3 Device */
	EV314_hdl = EV314_find_and_open(EV314_EXPECTED_SERIAL);

	/* Reset EV3 Encoders */
	reset_encoders();

	originX = 0;
	originY = 0;

	targetX = 0;
	targetY = 0;
}

void starbot::update()
{
	int pan_power = 0;

	gps_sensor->update();
	
	magnetic_model->update(gps_sensor->gps_lat, gps_sensor->gps_lon, gps_sensor->gps_alt);

	compass_sensor->update();

	currentX = bearing() + declination();

	if ((int)currentX != (int)targetX)
	{
		//for (int i = 0; i < 100; i++)
		//	compass_sensor->update();

		pan_power = 3000;

		if (currentX < targetX)
		{
			pan_power *= -1;
		}

		currentX = bearing() + declination();

		init_pan_servos(pan_power);

		ev314_profiling_start();

		EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control));

		memset(&ev314_state, 0, sizeof(struct ev314_state_struct));

		EV314_recv_buf(EV314_hdl, (unsigned char*)&ev314_state, sizeof(ev314_state));

		ev314_profiling_stop();
	}

	update_sensors();
}

int starbot::stop()
{
	if (!gps_sensor->stop()) {
		//printf("** Error while closing GPS.\n");
	}

	delete gps_sensor;
	delete magnetic_model;
	delete compass_sensor;

	return EV314_close(EV314_hdl);
}

void starbot::CaptureImage() {
	char buf[50];

	snprintf((char*)buf, 50, "raspistill  -n -t 1 -o Images/image%d.jpg", imageCount++);

	system((char*)buf);
}

void starbot::init_pan_servos(int pow)
{
	reset_encoders();

	/* Initilize control packet */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[0] = pow; // 10000;
	ev314_control.motor_power[3] = pow; // 10000;
}

void starbot::PanSteps(int pow, int steps) {
	//char buf[STARBOT_HISTORY_NB_CHAR_X];

	reset_encoders();

	/* Initilize control packet */

	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[0] = pow; // 10000;
	ev314_control.motor_power[3] = pow; // 10000;

	/* Entering status polling loop */
	for (;;) {

		/* Send control */

		ev314_profiling_start();

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

	reset_encoders();

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

void starbot::update_sensors() {
	int ret = 0;

	/* Initialize Control Structure */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_GPS;
	ev314_control.gps_fix = 0;
	ev314_control.gps_lon = 0;
	ev314_control.gps_lat = 0;
	ev314_control.gps_alt = 0;
	ev314_control.gps_sat = 0;
	ev314_control.gps_use = 0;

	if(gps_sensor->gps_fix) {
		/* Fix Obtained, Set Values. */
		ev314_control.gps_fix = gps_sensor->gps_fix;
		ev314_control.gps_lon = gps_sensor->gps_lon;
		ev314_control.gps_lat = gps_sensor->gps_lat;
		ev314_control.gps_alt = gps_sensor->gps_alt;

		ev314_control.gps_sat = gps_sensor->gps_sat;
		ev314_control.gps_use = gps_sensor->gps_use;
	}

	/* Send control */
	ev314_profiling_start();

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

int starbot::reset_encoders()
{
	/* Initialize encoders */
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;

	return EV314_send_buf(EV314_hdl, (unsigned char*)&ev314_control, sizeof(ev314_control));
}

int starbot::fix()
{
	return gps_sensor->gps_fix;
}

int starbot::sats_used()
{
	return gps_sensor->gps_use;
}

int starbot::sats_view()
{
	return gps_sensor->gps_sat;
}

double starbot::latitude()
{
	return gps_sensor->gps_lat;
}

double starbot::longitude()
{
	return gps_sensor->gps_lon;
}

double starbot::altitude()
{
	return gps_sensor->gps_alt;
}

int starbot::latitude_degrees()
{
	return gps_sensor->latitude_degrees();
}

int starbot::latitude_minutes()
{
	return gps_sensor->latitude_minutes();
}

double starbot::latitude_seconds()
{
	return gps_sensor->latitude_seconds();
}

int starbot::longitude_degrees()
{
	return gps_sensor->longitude_degrees();
}

int starbot::longitude_minutes()
{
	return gps_sensor->longitude_minutes();
}

double starbot::longitude_seconds()
{
	return gps_sensor->longitude_seconds();
}

double starbot::declination()
{
	return magnetic_model->declination();
}

double starbot::inclination()
{
	return magnetic_model->inclination();
}

double starbot::field_strength()
{
	return magnetic_model->strength();
}

double starbot::bearing()
{
	return compass_sensor->bearing;
}

