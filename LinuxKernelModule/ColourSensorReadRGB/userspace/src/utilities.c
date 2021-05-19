#include "utilities.h"

/**
 * @brief Sleep for the requested number of milliseconds. 
 * 
 */
int msleep(long msec){
    struct timespec ts;
    int res;

    if (msec < 0){
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

/**
 * @brief Map input value to a range of values
 * 
 */ 
long map(long x, long in_min, long in_max, long out_min, long out_max){
  
    long ret = -1;

    ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if(out_max >= out_min){

        if(ret < out_min){
            return out_min;
        }
        if(ret > out_max){
            return out_max;
        }
        else{
            return ret;
        }
    }
    else if(out_max <= out_min){
        if(ret > out_min){
            return out_min;
        }
        if(ret < out_max){
            return out_max;
        }
        else{
            return ret;
        }
    }
    else{
        return ret;
    }
}