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
	MagneticModel mag("emm2015"); //wmm2015
public:

	double declination();
	double inclination();
	double strength();

	void update();
};
#endif // !__WMM_H__
