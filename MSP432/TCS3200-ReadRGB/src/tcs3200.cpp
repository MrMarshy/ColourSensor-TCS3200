#include <stdint.h>
#include <string.h>
#include "msp.h"
#include "tcs3200.h"
#include "timers.h"
#include "gpio.h"

static void timerA0_2_evt_handler(void *ctx);
static TCS3200* _colourSensor;

timer_evt_handler TCS3200::capture_event_handler = timerA0_2_evt_handler;

#define DO_CALIB    0

TCS3200::TCS3200(){

    GPIO_OUTPUT_CONFIG(P2, BIT4); /* P2.4 as S0 */
    GPIO_OUTPUT_CONFIG(P3, BIT0); /* P3.0 as S1 */
    GPIO_OUTPUT_CONFIG(P5, BIT6); /* P5.6 as S2 */
    GPIO_OUTPUT_CONFIG(P5, BIT7); /* P5.7 as S3 */

#if DO_CALIB

    GPIO_SWITCHES_INPUT_CONFIG(); /* P1.1 and 1.4 input switches */
    GPIO_SWITCHES_INTR_ENABLE(); /* Enable Interrupts on the switches */

    GPIO_INPUT_CONFIG(P1, BIT6); /* P1.6 as input */
    GPIO_INPUT_INTR_ENABLE(P1, BIT6, PORT1_IRQn); /* Enable Input interupt on P1.6 */

#endif

    INPUT_CAPTURE_DEFAULT_CONFIG(); /* P2.5 as Freq Out */

    /* Set Freq Scaling to 20% */
    GPIO_SET(P2, BIT4);
    GPIO_SET(P3, BIT0);

    /* Read Red Initally */
    setRedFilter();

    _colourSensor = this;

    _first_edge_time = 0;
    _second_edge_time = 0;
    _diff = 0.0f;
    _is_first_edge = false;
}

void TCS3200::setRedFilter(){
    GPIO_RESET(P5, BIT6);
    GPIO_RESET(P5, BIT7);
    memcpy(_curr_filter_str, "RW", strlen("RW"));
    setCurrActiveFilter(TCS3200::ActiveFilter::RED_FILTER_ACTIVE);
}

void TCS3200::setGreenFilter(){
    GPIO_SET(P5, BIT6);
    GPIO_SET(P5, BIT7);
    memcpy(_curr_filter_str, "GW", strlen("GW"));
    setCurrActiveFilter(TCS3200::ActiveFilter::GREEN_FILTER_ACTIVE);
}

void TCS3200::setBlueFilter(){
    GPIO_RESET(P5, BIT6);
    GPIO_SET(P5, BIT7);
    memcpy(_curr_filter_str, "BW", strlen("GW"));
    setCurrActiveFilter(TCS3200::ActiveFilter::BLUE_FILTER_ACTIVE);
}

uint8_t TCS3200::getRedVal(){
    
    return red_val;
}
uint8_t TCS3200::getGreenVal(){return green_val;}
uint8_t TCS3200::getBlueVal(){return blue_val;}

void TCS3200::setDiff(){
       
    if(_first_edge_time > _second_edge_time){ // overflow

        _diff = static_cast<float>(_second_edge_time - (0xFFFFU - _first_edge_time));
       
    }
    else{
        // Convert to useconds
        _diff = static_cast<float>(_second_edge_time - _first_edge_time);
    }

    _diff /= static_cast<float>(SystemCoreClock);
    _diff *= 1e6f;
    
}

static void timerA0_2_evt_handler(void *ctx){

    NVIC_DisableIRQ(TA0_N_IRQn);
    
    TCS3200* sensor = (TCS3200*)ctx;

    if(!sensor->captureCompleted()){

        if(!sensor->isFirstEdgeCaptured()){ // First edge
            
            sensor->setFirstEdgeCaptured();

            sensor->setFirstEdgeTime(TIMER_A0->CCR[2]); // save first rising edge, timestamp
        }

        else if(sensor->isFirstEdgeCaptured()){ // Second Edge
            
            sensor->unsetFirstEdgeCaptured();

            sensor->setSecondEdgeTime(TIMER_A0->CCR[2]); // save second rising edge timestamp

            sensor->setDiff(); // Get the difference between timestamps.

            long in = static_cast<long>(sensor->getDiff());

            switch(sensor->getCurrActiveFilter()){

                case TCS3200::RED_FILTER_ACTIVE:
                    sensor->red_val = static_cast<uint8_t>(map(in, RED_CALIB_MIN, RED_CALIB_MAX, 255, 0));
                    sensor->setCurrActiveFilter(TCS3200::ActiveFilter::GREEN_FILTER_ACTIVE);
                    break;

                case TCS3200::GREEN_FILTER_ACTIVE:
                    sensor->green_val = static_cast<uint8_t>(map(in, GREEN_CALIB_MIN, GREEN_CALIB_MAX, 255, 0));
                    sensor->setCurrActiveFilter(TCS3200::ActiveFilter::BLUE_FILTER_ACTIVE);
                    break;

                case TCS3200::BLUE_FILTER_ACTIVE:
                    sensor->blue_val = static_cast<uint8_t>(map(in, BLUE_CALIB_MIN, BLUE_CALIB_MAX, 255, 0));
                    sensor->setCurrActiveFilter(TCS3200::ActiveFilter::RED_FILTER_ACTIVE);
                    break;

                default:;

            }

            sensor->setCaptureComplete(true);
        }
        
    }

    NVIC_EnableIRQ(TA0_N_IRQn);
}

extern "C" {

    void TA0_N_IRQHandler(void){
            
            TCS3200::capture_event_handler(_colourSensor);

            TIMER_A0->CCTL[2] &= ~BIT0; // clear interrupt flag
    }
}

extern "C" {

    void PORT1_IRQHandler(void){

            if(P1->IFG & BIT1){ // User SW1 (P1.1)
                _colourSensor->setGreenFilter();

                // Clear interrupt
                P1->IFG &= ~BIT1;
            }
            
            if(P1->IFG & BIT4){ // User SW2 (P1.4)
                
                _colourSensor->setBlueFilter();
                
                // Clear interrupt
                P1->IFG &= ~BIT4;
            }

            if(P1->IFG & BIT6){ // External Swtich connected to P1.6 with internal Pull UP enabled!
                _colourSensor->setRedFilter();

                // Clear Interrupt
                P1->IFG &=~ BIT6;
            }
           
    }
}