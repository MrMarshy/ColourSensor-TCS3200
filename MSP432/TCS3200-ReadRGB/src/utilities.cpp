#include "utilities.h"


/**
 * @brief Map input value to a range of values
 * 
 */ 
long map(long x, long in_min, long in_max, long out_min, long out_max){

    in_max += 1;
	out_max += 1;

	long ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	if((out_min > out_max) && (ret > out_min)){
		return out_min;
	}
	else if((out_min < out_max) && (ret > out_max)){
		return out_max;
	}
	else if((out_min > out_max) && (ret < out_max)){
		return out_max;
	}
	else if((out_min < out_max) && (ret < out_min)){
		return out_min;
	}
	else{
		return ret;
	}
}