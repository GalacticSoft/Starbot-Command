#include <time.h>
#include "wmm.h"

using namespace GeographicLib;

void wmm::update(float lat, float lon, float alt)
{
	double Bx, By, Bz;
	double H;
	time_t t = time(NULL);
	tm* timePtr = localtime(&t);
	MagneticModel mag("emm2015");

	mag(timePtr->tm_year + 1900, lat, lon, alt, Bx, By, Bz);
	MagneticModel::FieldComponents(Bx, By, Bz, H, field_strength, magnetic_declination, magnetic_inclination);
}

double wmm::declination()
{
	return magnetic_declination;
}

double wmm::inclination()
{
	return magnetic_inclination;
}

double wmm::strength()
{
	return field_strength;
}