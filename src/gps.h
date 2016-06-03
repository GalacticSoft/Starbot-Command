
#ifndef __GPS_H_INCLUDED__   // if x.h hasn't been included yet...
#define __GPS_H_INCLUDED__

#include "libgpsmm.h" // GPS

class gps
{
private:
	gpsmm *gps_rec;

	/*
	* degrees: converts decimal coordinate to degree component.
	*
	*/
	int degrees(float coordinate);

	/*
	* minutes: converts decimal coordinate to minutes component.
	*
	*/
	int minutes(float coordinate);

	/*
	* seconds: converts decimal coordinate to seconds component.
	*
	*/
	float seconds(float coordinate);
	
public:
	int							gps_fix;
	float						gps_lon;
	float						gps_lat;
	float						gps_alt;
	int							gps_sat;
	int							gps_use;

	/*
	* start: Start GPS reciever.
	*
	*/
	bool start();
	
	/*
	* stop: Stop GPS reciever and clean up.
	*
	*/
	bool stop();
	
	/*
	* update: Update values from GPS reciever
	*
	*/
	bool update();
	

	int latitude_degrees();
	int latitude_minutes();
	float latitude_seconds();
	
	int longitude_degrees();
	int longitude_minutes();
	float longitude_seconds();
};

#endif