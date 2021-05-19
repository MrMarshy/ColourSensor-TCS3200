#ifndef UTILITIES__H
#define UTILITIES__H

#include <time.h>
#include <errno.h>

/**
 * @brief Sleep for the requested number of milliseconds. 
 * 
 */
int msleep(long msec);


/**
 * @brief Map input value to a range of values
 * 
 */ 
long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif // UTILITIES__H