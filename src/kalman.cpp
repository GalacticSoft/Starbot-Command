#include "kalman.h"

void kalman_filter::initialize(float q, float r, float p, float x) {
	kalman_filter::state.q = q;
	kalman_filter::state.r = r;
	kalman_filter::state.p = p;
	kalman_filter::state.x = x;
}

float kalman_filter::update(float m) {
	if (kalman_filter::init) {
		kalman_filter::initialize(0.025f, 16, 1, m);
		kalman_filter::init = false;
	}

	//prediction update
	//omit x = x
	kalman_filter::state.p = kalman_filter::state.p + kalman_filter::state.q;

	//measurement update
	kalman_filter::state.k = kalman_filter::state.p / (kalman_filter::state.p + kalman_filter::state.r);
	kalman_filter::state.x = kalman_filter::state.x + kalman_filter::state.k * (m - kalman_filter::state.x);
	kalman_filter::state.p = (1 - kalman_filter::state.k) * kalman_filter::state.p;

	return kalman_filter::state.x;
}