#ifndef __STOPWATCH_H__
#define __STOPWATCH_H__

#include <stdio.h>
#include <stdlib.h>

class CStopwatch
{
	public:
	
	inline void start() {
		clock_gettime(CLOCK_MONOTONIC_RAW, &t_start);	
	}
	
	inline double stop() {
  	clock_gettime(CLOCK_MONOTONIC_RAW, &t_end);
		timespec temp;
		if ((t_end.tv_nsec-t_start.tv_nsec)<0) {
			temp.tv_sec = t_end.tv_sec-t_start.tv_sec-1;
			temp.tv_nsec = 1000000000+t_end.tv_nsec-t_start.tv_nsec;
		} else {
			temp.tv_sec = t_end.tv_sec-t_start.tv_sec;
			temp.tv_nsec = t_end.tv_nsec-t_start.tv_nsec;
		}	
		clock_gettime(CLOCK_MONOTONIC_RAW, &t_start);	
		return temp.tv_nsec/1000000000.0;	
	}
	
private:
	timespec t_start, t_end;	
};

#endif

