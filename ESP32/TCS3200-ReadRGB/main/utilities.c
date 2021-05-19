#include "utilities.h"

long map(long x, long in_min, long in_max, long out_min, long out_max){

	in_max += 1;
	out_max += 1;

	long ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	if((ret < out_min) && (out_min > out_max)){
		return ret;
	}
	else if((ret < out_min) && (out_min < out_max)){
		return out_min;
	}
	else if((ret > out_max) && (out_min > out_max)){
		return ret;
	}
	else if((ret > out_max) && (out_min < out_max)){
		return out_max;
	}
	else{
		return ret;
	}

}