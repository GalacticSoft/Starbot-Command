#ifndef __KALMAN_H__  
#define __KALMAN_H__

typedef struct {
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //value
	float p; //estimation error covariance
	float k; //kalman gain
} kalman_state;

class kalman_filter
{
private:
	static bool init;
	static kalman_state state;

public:
	kalman_filter() { init = true; }

	static void initialize(float q, float r, float p, float x);

	static float update(float m);
};


#endif // __KALMAN_H__