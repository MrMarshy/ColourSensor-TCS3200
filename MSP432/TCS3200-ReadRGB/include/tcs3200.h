/*
 * tcs3200.h
 */

#ifndef TCS3200_H_
#define TCS3200_H_

#include <stdint.h>
#include "utilities.h"

#define LED1        0U
#define RGB_RED     0U
#define RGB_GREEN   1U
#define RGB_BLUE    2U

typedef void(*timer_evt_handler)(void* context);

constexpr uint16_t RED_CALIB_MAX = 270;
constexpr uint16_t RED_CALIB_MIN = 15;
constexpr uint16_t GREEN_CALIB_MAX = 300;
constexpr uint16_t GREEN_CALIB_MIN = 19;
constexpr uint16_t BLUE_CALIB_MAX = 350;
constexpr uint16_t BLUE_CALIB_MIN = 21;


class TCS3200{

public:

    TCS3200();
    ~TCS3200() = default;

    enum ActiveFilter{
        RED_FILTER_ACTIVE,
        GREEN_FILTER_ACTIVE,
        BLUE_FILTER_ACTIVE
    };

    void setFirstEdgeTime(uint16_t timestamp){_first_edge_time = timestamp;}
    void setSecondEdgeTime(uint16_t timestamp){_second_edge_time = timestamp;}
    uint16_t getSecondEgeTime(){ return _second_edge_time;}
    uint16_t getFirstEdgeTime(){ return _first_edge_time;}
    float getDiff(){ return _diff;}
    char* getCurrFilterStr(){return _curr_filter_str;}
    TCS3200::ActiveFilter getCurrActiveFilter(){return _curr_active_filter;}
    uint8_t getRedVal();
    uint8_t getGreenVal();
    uint8_t getBlueVal();
  

    bool captureCompleted(){return _has_capture;}
    bool isFirstEdgeCaptured(){return _is_first_edge;}
    bool isAllColoursFound(){return _all_colours_found;}

    void setCaptureComplete(bool isComplete){_has_capture = isComplete;}
    void setRedFilter();
    void setGreenFilter();
    void setBlueFilter();
    void setAllColoursFound(bool status){_all_colours_found = status;}
    void setFirstEdgeCaptured(){_is_first_edge = true;};
    void unsetFirstEdgeCaptured(){ _is_first_edge = false;}
    void setDiff();
    void setCurrActiveFilter(TCS3200::ActiveFilter filter){_curr_active_filter = filter;}

    uint8_t red_val;
    uint8_t green_val;
    uint8_t blue_val;

    uint8_t nextColourToGet = 0;

    uint8_t completedCaptureCount = 0;
    

    static timer_evt_handler capture_event_handler;

    

private:
    
    uint16_t _first_edge_time;
    uint16_t _second_edge_time;
    float _diff;
    char _curr_filter_str[32] = "RW"; 
    TCS3200::ActiveFilter _curr_active_filter = RED_FILTER_ACTIVE;
    bool _has_capture;
    bool _is_first_edge;
    bool _all_colours_found = false;


};

#endif /* TCS3200_H_ */
