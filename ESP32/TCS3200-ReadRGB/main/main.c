#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"

#include "utilities.h"
#include "TCS320.h"

/**
 * TCS3200 Pins
 */
#define S0_GPIO_PIN 16
#define S1_GPIO_PIN 17
#define S2_GPIO_PIN 18
#define S3_GPIO_PIN 19
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<S0_GPIO_PIN) | (1ULL<<S1_GPIO_PIN) | (1ULL<<S2_GPIO_PIN) | (1ULL<<S3_GPIO_PIN))
#define GPIO_CAP0_IN   23    // Set GPIO 23 as  CAP0 (TCS3200 frequency output pin)

#define MCPWM_EN_CAPTURE 1   // Make this 1 to test capture submodule of mcpwm, measure time between rising/falling edge of captured signal
#define MCPWM_GPIO_INIT 0    // select which function to use to initialize gpio signals
#define CAP_SIG_NUM 1        // one capture signal
#define CAP0_INT_EN BIT(27)  // Capture 0 interrupt bit

static TCS320_t colourSensor = {0};

// Prototypes of the TCS320_t function pointers
static int getRedPulseWidth(void);
static int getGreenPulseWidth(void);
static int getBluePulseWidth(void);
static int setFrequencyScaling(TCS320_FreqScaler_t);
static void enable_capture(void);

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

static xQueueHandle cap_queue;
static xQueueHandle pulse_width_queue;
static volatile uint8_t is_capture_enabled = 0;

#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

static void gpio_initialize(void)
{
    printf("initializing gpio...\n");
#if MCPWM_GPIO_INIT
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
#else
    mcpwm_pin_config_t pin_config = {
        .mcpwm_cap0_in_num   = GPIO_CAP0_IN,
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
#endif
    gpio_pulldown_en(GPIO_CAP0_IN);    //Enable pull down on CAP0   signal

    /* Set S0-S3 Pins as outputs */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask the pins to set as output
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(S0_GPIO_PIN, 0);
    gpio_set_level(S1_GPIO_PIN, 0);
    gpio_set_level(S2_GPIO_PIN, 0);
    gpio_set_level(S3_GPIO_PIN, 0);

}

#if MCPWM_EN_CAPTURE
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void* arg)
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status

    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }

    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}
#endif

/**
 * @brief Enable Caputure module
 */
static void enable_capture(void)
{
    // Capture configuration
    // Configure CAP0 signal to start capture counter on rising edge
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second

    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN;  //Enable interrupt on  CAP0
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    is_capture_enabled = 1;
}

static void disable_capture(void)
{
    // disable capture
    mcpwm_capture_disable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);

    // disable interrupt, 
    MCPWM[MCPWM_UNIT_0]->int_ena.val = 0; 

    is_capture_enabled = 0;
}


/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
static void get_captured_signal(void *arg)
{
    
    uint32_t prev_capture = 0;
    uint32_t curr_capture = 0;
    capture evt;

    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);

        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            
            portDISABLE_INTERRUPTS();

            curr_capture = evt.capture_signal - prev_capture;
            
            prev_capture = evt.capture_signal;
                        
            curr_capture = (curr_capture / 10000) * (10000000000 / rtc_clk_apb_freq_get()); 

            colourSensor.rawPulseWidth = (int32_t)curr_capture;

            xQueueSendFromISR(pulse_width_queue, &colourSensor, NULL);
            

            portENABLE_INTERRUPTS();
        }
    }
}

/**
 * @brief configure the GPIO to read the red, green and blue raw values from the sensor
 */
static void tcs3200_read_rgb(void *args)
{
    uint32_t delayTime = 200;

    for(;;){
        
        colourSensor.getRedPulseWidth();
        
        colourSensor.getGreenPulseWidth();
    
        colourSensor.getBluePulseWidth();
       
        if(colourSensor.doCalibration){
            printf("RW: %d GW: %d BW: %d\r\n", colourSensor.redPulseWidth, colourSensor.greenPulseWidth, colourSensor.bluePulseWidth);
            printf("%d, %d, %d\r\n", colourSensor.redValue, colourSensor.greenValue, colourSensor.blueValue);
        }
        else{
            // Send Values to PC Program running classification over USB Serial
            printf("%d, %d, %d\r\n", colourSensor.redValue, colourSensor.greenValue, colourSensor.blueValue);
        }

        vTaskDelay(delayTime/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    printf("Testing TCS3200 Colour Sensor...\n");
    printf("Built: %s %s\r\n\r\n", __DATE__, __TIME__);

    // Create Queues First to avoid xQueueSend using null queues
    cap_queue = xQueueCreate(1, sizeof(capture));
    pulse_width_queue = xQueueCreate(1, sizeof(TCS320_t));

    // gpio initialization
    gpio_initialize();

    // Enable input capture
    enable_capture();

    // Assign function pointers to read RGB Pulse widths and set frequency scaling
    colourSensor.getRedPulseWidth = getRedPulseWidth;
    colourSensor.getGreenPulseWidth = getGreenPulseWidth;
    colourSensor.getBluePulseWidth = getBluePulseWidth;
	colourSensor.setFrequencyScaling = setFrequencyScaling;
    
    // Perform calibration of colours
    colourSensor.doCalibration = 0;

    // Set Frequency Scaling to 2%
    colourSensor.setFrequencyScaling(TCS320_SCALE_20_PCT);
    vTaskDelay(50/portTICK_PERIOD_MS);

    xTaskCreate(get_captured_signal, "mcpwm_config", 4096, NULL, (tskIDLE_PRIORITY + 1), NULL);
    xTaskCreate(tcs3200_read_rgb, "tcs3200_read_rgb", 4096, NULL, (tskIDLE_PRIORITY + 2), NULL);
    
}

static int getRedPulseWidth(void){
	
	// Set Sensor to read Red Only
	gpio_set_level(S2_GPIO_PIN, 0);
    gpio_set_level(S3_GPIO_PIN, 0);
	vTaskDelay(10/portTICK_PERIOD_MS);

    int32_t avg = 0;
    int32_t total_measurements = 128;

    for(int i = 0; i < total_measurements; ++i){

        xQueueReceive(pulse_width_queue, &colourSensor, portMAX_DELAY);

        colourSensor.redPulseWidth = colourSensor.rawPulseWidth; 

        avg += colourSensor.redPulseWidth;

    }
    
    avg  /= total_measurements;
    colourSensor.redPulseWidth = avg;
    colourSensor.redValue = (uint8_t)map(colourSensor.redPulseWidth, TCS320_RED_MIN,
                TCS320_RED_MAX, 255, 0);    

    return 0;
}

static int getGreenPulseWidth(void){

	// Set Sensor to read Green Only
	gpio_set_level(S2_GPIO_PIN, 1);
    gpio_set_level(S3_GPIO_PIN, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);
    
    
    int32_t avg = 0;
    int32_t total_measurements = 128;

    for(int32_t i = 0; i < total_measurements; ++i){

        xQueueReceive(pulse_width_queue, &colourSensor, portMAX_DELAY);

        colourSensor.greenPulseWidth = colourSensor.rawPulseWidth; 

        avg += colourSensor.greenPulseWidth;

    }
    
    avg  /= total_measurements;
    colourSensor.greenPulseWidth = avg;
    colourSensor.greenValue = (uint8_t)map(colourSensor.greenPulseWidth, TCS320_GREEN_MIN,
                TCS320_GREEN_MAX, 255, 0);    

    return 0;
}

static int getBluePulseWidth(void){

	// Set Sensor to read Blue Only
	gpio_set_level(S2_GPIO_PIN, 0);
    gpio_set_level(S3_GPIO_PIN, 1);
	vTaskDelay(10/portTICK_PERIOD_MS);
	
    int32_t avg = 0;
    int32_t total_measurements = 128;

    for(int32_t i = 0; i < total_measurements; ++i){

        xQueueReceive(pulse_width_queue, &colourSensor, portMAX_DELAY);

        colourSensor.bluePulseWidth = colourSensor.rawPulseWidth; 

        avg += colourSensor.bluePulseWidth;

    }
    
    avg  /= total_measurements;
    colourSensor.bluePulseWidth = avg;
    colourSensor.blueValue = (uint8_t)map(colourSensor.bluePulseWidth, TCS320_BLUE_MIN,
                TCS320_BLUE_MAX, 255, 0);    

    return 0;
}
	

static int setFrequencyScaling(TCS320_FreqScaler_t scaleFactor){
	
	switch(scaleFactor){
        
        case TCS320_SCALE_2_PCT:
            /* Reset S0 Pin, Set S1 Pin */
            gpio_set_level(S0_GPIO_PIN, 0);
            gpio_set_level(S1_GPIO_PIN, 1);
            break;

		case TCS320_SCALE_20_PCT:
			/* Set S0 Pin, Reset S1 Pin */
            gpio_set_level(S0_GPIO_PIN, 1);
            gpio_set_level(S1_GPIO_PIN, 0);
			break;

        case TCS320_SCALE_100_PCT:
            /* Set S0 Pin, Set S1 Pin */
            gpio_set_level(S0_GPIO_PIN, 1);
            gpio_set_level(S1_GPIO_PIN, 1);
            break;

        case TCS320_POWER_DOWN:
            gpio_set_level(S0_GPIO_PIN, 0);
            gpio_set_level(S1_GPIO_PIN, 0);
            break;

		default:break;
				
	}
	
	return 0;
}


