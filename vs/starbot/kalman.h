#ifndef __KALMAN_H__  
#define __KALMAN_H__

#pragma once

typedef struct {
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //value
	float p; //estimation error covariance
	float k; //kalman gain
} kalman_state;

kalman_state kalman_init(float q, float r, float p, float x);

void kalman_update(kalman_state * s, float m);

#endif // __KALMAN_H__