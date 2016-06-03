#include "gps.h"
#include "libgpsmm.h" // GPS
#include "math.h"
#include <stdlib.h>

/*
* degrees: converts decimal angle to degree component.
*
*/
int gps::degrees(float angle) {
	return abs((int)angle);
}

/*
* minutes: converts decimal angle to minutes component.
*
*/
int gps::minutes(float angle) {
	int deg = degrees(angle);
	int min = (int)((fabs(angle) - deg) * 60.00);

	return min;
}

/*
* seconds: converts decimal angle to seconds component.
*
*/
float gps::seconds(float angle) {
	int deg = degrees(angle);
	int min = minutes(angle);

	float sec = (((fabs(angle) - deg) * 60.00) - (float)min) * 60.00;

	return sec;
}

int gps::latitude_degrees() {
	return degrees(gps_lat);
}

int gps::latitude_minutes() {
	return minutes(gps_lat);
}

float gps::latitude_seconds() {
	return seconds(gps_lat);
}

int gps::longitude_degrees() {
	return degrees(gps_lon);
}

int gps::longitude_minutes() {
	return minutes(gps_lon);
}

float gps::longitude_seconds() {
	return seconds(gps_lon);
}

bool gps::start() {
	/* Initialize GPS */
	gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
	//console_log("** Initializing GPS.");

	if (gps_rec->stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
		return false;
	}

	return true;
}

bool gps::stop() {
	if(gps_rec->stream(WATCH_DISABLE) == NULL) {
		delete gps_rec;
		return false;
	}
	
	delete gps_rec;
	return true;
}

bool gps::update() {
	struct gps_data_t* gps_data;
	
	if (gps_rec->waiting(00050000)) {
		/* See if we're reading data. */
		if ((gps_data = gps_rec->read()) == NULL) {
			return false;
		}
		
		if (gps_data->status == STATUS_FIX && (gps_data->fix.mode == MODE_2D || gps_data->fix.mode == MODE_3D)) {
			/* Fix Obtained, Set Values. */
			gps_fix = gps_data->fix.mode;
			gps_lon = gps_data->fix.longitude;
			gps_lat = gps_data->fix.latitude;
			gps_alt = gps_data->fix.altitude;

			gps_sat = gps_data->satellites_visible;
			gps_use = gps_data->satellites_used;
		}
	}
	
	return true;
}