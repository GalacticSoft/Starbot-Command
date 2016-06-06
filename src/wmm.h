#ifndef __WMM_H__
#define __WMM_H__

#include <GeographicLib/MagneticModel.hpp> // Magnetic Model

using namespace GeographicLib;

class wmm
{
private:
	double magnetic_declination;
	double magnetic_inclination;
	double field_strength;

public:

	double declination();
	double inclination();
	double strength();

	void update(float lat, float lon, float alt);
};
#endif // !__WMM_H__
