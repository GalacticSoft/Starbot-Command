#include "kalman.h"

void kalman_filter::init(float q, float r, float p, float x) {
	state.q = q;
	state.r = r;
	state.p = p;
	state.x = x;
}

float kalman_filter::update(float m) {
	if (init) {
		initialize(0.025f, 16, 1, m);
		init = false;
	}

	//prediction update
	//omit x = x
	state.p = state.p + state.q;

	//measurement update
	state.k = state.p / (state.p + state.r);
	state.x = state.x + state.k * (m - state.x);
	state.p = (1 - state.k) * state.p;

	return state.x;
}