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
	bool init = true;
	kalman_state state;

public:
	void init(float q, float r, float p, float x);

	float update(float m);
};


#endif // __KALMAN_H__