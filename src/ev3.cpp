#include "ev3.h"

/*
* ev314_profiling_start: start timer
*
*/
void ev314_profiling_start(void) {
#ifdef EV314_PROFILING_ON
	clock_gettime(CLOCK_MONOTONIC, &profiling_start);
#endif
}

/*
* ev314_profiling_stop: stop timer and print time
*
*/
void ev314_profiling_stop(void) {
#ifdef EV314_PROFILING_ON
	struct timespec						profiling_stop;

	clock_gettime(CLOCK_MONOTONIC, &profiling_stop);

	fprintf(stderr, "** Profiling duration: %d us.\n",
		(int)((profiling_stop.tv_sec - profiling_start.tv_sec) * 1000000
			+ (profiling_stop.tv_nsec - profiling_start.tv_nsec) / 1000));

#endif
}