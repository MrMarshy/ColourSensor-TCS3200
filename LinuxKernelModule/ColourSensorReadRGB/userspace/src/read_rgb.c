/**
 * TCS3200 Colour Sensor to Jetson Nano Pins
 * -----------------------------------------------
 * S0 pin connected to Jetson Nano Pin 7 (GPIO216)
 * S1 pin connected to Jetson Nano Pin 11 (GPIO50)
 * S2 pin connected to Jetson Nano Pin 13 (GPIO14)
 * S3 pin connected to Jetson Nano Pin 15 (GPIO194)
 * Out pin connected to Jetson Nano Pin 19 (GPIO16)
 * OE pin connected to Jetson Nano GND
 * VCC pin connected to Jetson Nano 3v3
 * GND pin connected to Jetson Nano GND
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include "utilities.h"

struct sigaction old_action;
static sig_atomic_t go = 1;

typedef enum{
    RED_FILTER,
    GREEN_FILTER,
    BLUE_FILTER,
    NO_FILTER
}Colour_Filter;

typedef struct {
    int s0;
    int s1;
    int s2;
    int s3;
    int pulse;
    int timeout;
    Colour_Filter colour_filter;
}Colour_Sensor;


static void sigint_handler(int sig_no);
static long get_pulse_width(Colour_Sensor const *sensor);
static ssize_t select_colour_filter(Colour_Sensor const *sensor);
static ssize_t configure_sensor(Colour_Sensor const *sensor);
static ssize_t unconfigure_sensor(Colour_Sensor const *sensor);



int main(int argc, char* argv[]){
    (void)argc;
    (void)argv;
    ssize_t is_configured =  0;
    long pulse_width = -1;
    
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &sigint_handler;
    sigaction(SIGINT, &action, &old_action);
    
    Colour_Sensor sensor = {
        .s0 = 216, 
        .s1 = 50, 
        .s2 = 14, 
        .s3 = 194, 
        .pulse = 16, 
        .timeout = 1000000, 
        .colour_filter = RED_FILTER
    };

    char *fifo = "/tmp/fifopipe";
    mkfifo(fifo, 0666);

    is_configured = configure_sensor(&sensor);
    
    if(!(is_configured  > 0)){
        fprintf(stderr, "Unable to configure sensor : %ld\r\n", is_configured);
        return EXIT_FAILURE;
    }

    while(go){

        /* Get red pulse width */
        sensor.colour_filter = RED_FILTER;
        select_colour_filter(&sensor);
        
        msleep(100);
        
        pulse_width = get_pulse_width(&sensor);
        long r = map(pulse_width, 3, 40, 255, 0 );
        printf("RW: %ld RV: %ld\r\n", pulse_width, r);

       
        /* Get green pulse width */
        sensor.colour_filter = GREEN_FILTER;
        select_colour_filter(&sensor);
        
        msleep(100);

        pulse_width = get_pulse_width(&sensor);

        long g = map(pulse_width, 3, 50, 255, 0 );
        printf("GW: %ld GV: %ld\r\n", pulse_width, g);

        /* Get blue pulse width */
        sensor.colour_filter = BLUE_FILTER;
        select_colour_filter(&sensor);
        
        msleep(100);
        
        pulse_width = get_pulse_width(&sensor);

        long b = map(pulse_width, 3, 38, 255, 0 );
        printf("BW: %ld BV: %ld\r\n", pulse_width, b);

	    /* Send RGB Values to classify colours python program over pipe */
        int fd = open(fifo, O_WRONLY);
        
        char rgb[32] = {0};
        
        snprintf(rgb, sizeof(rgb) - 1, "%ld,%ld,%ld\r\n", r, g, b);

        ssize_t written = write(fd, rgb, strlen(rgb) + 1);

        if(!(written  > 0)){
            fprintf(stderr, "Unable to send rgb colours : %ld\r\n", written);
        }
        close(fd);
        
    }

    // cleanup
    fwrite("unconfiguring sensor\r\n", 1, sizeof("unconfiguring sensor\r\n"), stdout);
    unconfigure_sensor(&sensor);
    return 0;
}

static ssize_t configure_sensor(Colour_Sensor const *sensor){
    
    const char *config_str = "%d %d %d %d %d %d";
    char buff[128] = {0};

    snprintf(buff, sizeof(buff) - 1, config_str, 
        sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
        sensor->pulse, sensor->timeout);

    int fd = open("/sys/class/colour-sensor/configure", O_WRONLY);
    
    ssize_t written = write(fd, buff, strlen(buff));

    close(fd);

    return written;
}

static ssize_t unconfigure_sensor(Colour_Sensor const *sensor){
    
    const char *config_str = "-%d %d %d %d %d %d";
    char buff[128] = {0};

    snprintf(buff, sizeof(buff) - 1, config_str, 
        sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
        sensor->pulse, sensor->timeout);

    int fd = open("/sys/class/colour-sensor/configure", O_WRONLY);
    
    ssize_t written = write(fd, buff, strlen(buff));

    close(fd);

    return written;
}

static ssize_t select_colour_filter(Colour_Sensor const *sensor){

    const char *config_str = "%d %d %d %d %d %d %d";
    char buff[32] = {0};

    switch (sensor->colour_filter){

        case RED_FILTER:
            snprintf(buff, sizeof(buff) - 1, config_str, 
                sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
                sensor->pulse, 0, 0); 
            break;

        case GREEN_FILTER:
            snprintf(buff, sizeof(buff) - 1, config_str, 
                sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
                sensor->pulse, 1, 1); 
            break;

        case BLUE_FILTER:
            snprintf(buff, sizeof(buff) - 1, config_str, 
                sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
                sensor->pulse, 0, 1); 
            break;

        case NO_FILTER:
            snprintf(buff, sizeof(buff) - 1, config_str, 
                sensor->s0, sensor->s1, sensor->s2, sensor->s3, 
                sensor->pulse, 1, 0); 
            break;

        default:
            break;
    }
    
    int fd = open("/sys/class/colour-sensor/filter_type", O_WRONLY);
    
    ssize_t written = write(fd, buff, strlen(buff));

    close(fd);

    return written;
}

static long get_pulse_width(Colour_Sensor const *sensor){
    
    long avg = 0;
    char *ptr;
    char buffer[256] = {0};
    char read_buffer[32] = {0};

    const char *config_str = "/sys/class/colour-sensor/pulsewidth_%d_%d_%d_%d_%d/measure";

    snprintf(buffer, sizeof(buffer) - 1,  config_str, sensor->s0, sensor->s1, sensor->s2,
                sensor->s3, sensor->pulse);

    int fd = open(buffer, O_RDONLY);

    for(int i = 0; i < 16; ++i){

        ssize_t bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
        
        if(!(bytes_read  > 0)){

            //fprintf(stderr, "Unable to obtain pulse width : %ld\r\n", bytes_read);
        }

        avg += strtol(read_buffer, &ptr, 10);

        msleep(2);

        memset(read_buffer, '\0', sizeof(read_buffer));

    }
    close(fd);

    return avg / 16;
}


static void sigint_handler(int sig_no){
    (void)sig_no;
    sigaction(SIGINT, &old_action, NULL);
    go = 0;
}
