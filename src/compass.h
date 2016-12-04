
#ifndef __COMPASS_H__  
#define __COMPASS_H__

#define HMC5883L_I2C_ADDR 0x1E

#define COMPASS_ERROR_NONE		 1
#define COMPASS_ERROR_OPEN		-1
#define COMPASS_ERROR_READ		-2
#define COMPASS_ERROR_WRITE		-3

#define MIN_COMPASS     0
#define NORTH			0
#define NORTHEAST		4
#define EAST			8
#define SOUTHEAST		12
#define SOUTH			16
#define SOUTHWEST		20
#define WEST			24
#define NORTHWEST		28
#define MAX_COMPASS		32

typedef struct {
	const int	 idx;
	const char * name;
	const char * abb;
	const char * twp;
	const double min;
	const double mid;
	const double max;
} compass_point;

const compass_point compass_points[] = {
	{ 1,    "North",				"N",	"Tramontana",							354.38,	 0.00,	 5.62 },
	{ 2,	"North by east",		"NbE",	"Quarto di Tramontana verso Greco",		  5.63,	 11.25,	 16.87 },
	{ 3,	"North - northeast",	"NNE",	"Greco - Tramontana",					 16.88,	 22.50,	 28.12 },
	{ 4,	"Northeast by north",	"NEbN",	"Quarto di Greco verso Tramontana",		 28.13,	 33.75,	 39.37 },
	{ 5,	"Northeast",			"NE",	"Greco",								 39.38,	 45.00,	 50.62 },
	{ 6,	"Northeast by east",	"NEbE",	"Quarto di Greco verso Levante",		 50.63,	 56.25,	 61.87 },
	{ 7,	"East - northeast",		"ENE",	"Greco - Levante",						 61.88,	 67.50,	 73.12 },
	{ 8,	"East by north",		"EbN",	"Quarto di Levante verso Greco",		 73.13,	 78.75,	 84.37 },
	{ 9,	"East",					"E",	"Levante",								 84.38,	 90.00,	 95.62 },
	{ 10,	"East by south",		"EbS",	"Quarto di Levante verso Scirocco",		 95.63,	101.25,	106.87 },
	{ 11,	"East - southeast",		"ESE",	"Levante - Scirocco",					106.88,	112.50,	118.12 },
	{ 12,	"Southeast by east",	"SEbE",	"Quarto di Scirocco verso Levante",		118.13,	123.75,	129.37 },
	{ 13,	"Southeast",			"SE",	"Scirocco",								129.38,	135.00,	140.62 },
	{ 14,	"Southeast by south",	"SEbS",	"Quarto di Scirocco verso Ostro",		140.63,	146.25,	151.87 },
	{ 15,	"South - southeast",	"SSE",	"Ostro - Scirocco",						151.88,	157.50,	163.12 },
	{ 16,	"South by east",		"SbE",	"Quarto di Ostro verso Scirocco",		163.13,	168.75,	174.37 },
	{ 17,	"South",				"S",	"Ostro",								174.38,	180.00,	185.62 },
	{ 18,	"South by west",		"SbW",	"Quarto di Ostro verso Libeccio",		185.63,	191.25,	196.87 },
	{ 19,	"South - southwest",	"SSW",	"Ostro - Libeccio",						196.88,	202.50,	208.12 },
	{ 20,	"Southwest by south",	"SWbS",	"Quarto di Libeccio verso Ostro",		208.13,	213.75,	219.37 },
	{ 21,	"Southwest",			"SW",	"Libeccio",								219.38,	225.00,	230.62 },
	{ 22,	"Southwest by west",	"SWbW",	"Quarto di Libeccio verso Ponente",		230.63,	236.25,	241.87 },
	{ 23,	"West - southwest",		"WSW",	"Ponente - Libeccio",					241.88,	247.50,	253.12 },
	{ 24,	"West by south",		"WbS",	"Quarto di Ponente verso Libeccio",		253.13,	258.75,	264.37 },
	{ 25,	"West",					"W",	"Ponente",								264.38,	270.00,	275.62 },
	{ 26,	"West by north",		"WbN",	"Quarto di Ponente verso Maestro",		275.63,	281.25,	286.87 },
	{ 27,	"West - northwest",		"WNW",	"Maestro - Ponente",					286.88,	292.50,	298.12 },
	{ 28,	"Northwest by west",	"NWbW",	"Quarto di Maestro verso Ponente",		298.13,	303.75,	309.37 },
	{ 29,	"Northwest",			"NW",	"Maestro",								309.38,	315.00,	320.62 },
	{ 30,	"Northwest by north",	"NWbN",	"Quarto di Maestro verso Tramontana",	320.63,	326.25,	331.87 },
	{ 31,	"North - northwest",	"NNW",	"Maestro - Tramontana",					331.88,	337.50,	343.12 },
	{ 32,	"North by west",		"NbW",	"Quarto di Tramontana verso Maestro",	343.13,	348.75,	354.37 },
};

class compass
{
private:
	bool init;
	int i2c_fd;

	//kalman_state state; 

	bool select_i2c_device(int fd, int addr, char * name);
	bool write_to_i2c(int fd, int reg, int val);

public:
	float radians;
	float filtered_radians;
	float degrees;
	float bearing;

	/*
	* start: Start GPS reciever.
	*
	*/
	int start();


	/*
	* update: Update values from GPS reciever
	*
	*/
	int update(float (*filter)(float));

	//float compass_angle();

	compass_point get_compass_point();
	float get_compass_point_variance(compass_point cp);
};

#endif