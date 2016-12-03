#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wmm.h"
#include "compass.h"
#include "gps.h"
#include "kalman.h"

int main()
{
	bool init = true;
	kalman_state state;

	printf("starting...\n\r");

	/* Create Components */
	gps * gps_sensor = new gps();
	wmm * magnetic_model = new wmm();
	compass * compass_sensor = new compass();

	/* Initialize Components */
	gps_sensor->start();
	compass_sensor->start();

	/* Clear Screen */
	printf("\033[2J\033[?25l");

	/* Enter Main Control Loop */
	while (1) {
		/* Update GPS Sensor */
		gps_sensor->update();

		/* Pass GPS Data to WMM to get Declination */
		magnetic_model->update(gps_sensor->gps_lat, gps_sensor->gps_lon, gps_sensor->gps_alt);

		/* Read from Compass Sensor */
		compass_sensor->update();

		if (init) {
			state = kalman_init(0.025f, 16, 1, compass_sensor->bearing);
			init = false;
		}

		kalman_update(&state, compass_sensor->bearing);

		float filtered_bearing = state.x;

		if (filtered_bearing < 0)
			filtered_bearing += 360;

		/* Calculate True Bearing from compass bearing and WMM Declination */
		float true_bearing = filtered_bearing + magnetic_model->declination();

		/* Reset Terminal Output */
		printf("\033[0;0H");

		/* Print GPS Coordinates */
		printf("Latitude: %3d° %2d' %2.3f\" %c\n\rLongitude: %3d° %2d' %2.3f\" %c\n\r",
			gps_sensor->latitude_degrees(), gps_sensor->latitude_minutes(), gps_sensor->latitude_seconds(), gps_sensor->gps_lat >= 0 ? 'N' : 'S',
			gps_sensor->longitude_degrees(), gps_sensor->longitude_minutes(), gps_sensor->longitude_seconds(), gps_sensor->gps_lon >= 0 ? 'E' : 'W');

		/* Print Altitude and Declination */
		printf("Altitude: %2.3f\n\r\n\rDeclination: %2.3f\n\r\n\r", gps_sensor->gps_alt, magnetic_model->declination());

		/* Print Raw Bearing, Filtered Bearing and True Bearing */
		//printf("\n\rMagnetic Bearing: %2.3f\n\rFiltered Bearing: %2.3f\n\rTrue Bearing %2.3f\n\r", compass_sensor->bearing, filtered_bearing, true_bearing);

		printf("Radians: %007.3f\n\Bearing: %007.3f (-180 to 180)\n\rDegrees: %007.3f (   0 to 360)\n\r",  compass_sensor->radians, compass_sensor->bearing, compass_sensor->degrees);
		compass_point cp = compass_sensor->get_compass_point();

		printf("\33[2K\r");
		printf("\n\r[%s] %s (%s) %6.2f %6.2f\n\r", cp.abb, cp.name, cp.twp, cp.mid, compass_sensor->get_compass_point_variance(cp));
	}
	
	gps_sensor->stop();

	delete gps_sensor;
	delete compass_sensor;
	delete magnetic_model;

	return 0;
}
