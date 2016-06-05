#include "kalman.h"

#pragma once

kalman_state kalman_init(float q, float r, float p, float x) {
	kalman_state s;
	s.q = q;
	s.r = r;
	s.p = p;
	s.x = x;

	return s;
}

void kalman_update(kalman_state * s, float m) {
	//prediction update
	//omit x = x
	s->p = s->p + s->q;

	//measurement update
	s->k = s->p / (s->p + s->r);
	s->x = s->x + s->k * (m - s->x);
	s->p = (1 - s->k) * s->p;
}