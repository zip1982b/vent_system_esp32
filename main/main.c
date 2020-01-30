/* ventilation system progect

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

//#include "esp32/rom/ets_sys.h"



#include "esp_log.h"


/**
 * Brief:
 * 
 * GPIO status:
 * GPIO32: output
 * GPIO34:  input, interrupt from rising edge and falling edge
 * 
 */
 
static const char* TAG = "VentSys";

#define FAN				32
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<FAN)

#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  (1ULL<<ZERO_SENSOR)

#define ESP_INTR_FLAG_DEFAULT 0

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER1_INTERVAL_SWITCH_ON_TRIAC   (0.00005) // for switch on TRIAC (sec) - 50uS
#define SPEED_1   (0.005)   // for firing angle to be 90' for 220V 50Hz AC signal, we need to have a delay of 5 ms
#define WITHOUT_RELOAD   0        
#define WITH_RELOAD      1     

typedef struct {
    uint8_t speed; //0, 1 ,2
} fan_event_t;


portBASE_TYPE xStatus_venting;
portBASE_TYPE xStatus_task_isr_handler_ZS;
portBASE_TYPE xStatus_task_isr_handler_T0;
portBASE_TYPE xStatus_task_isr_handler_T1;


xTaskHandle xVenting_Handle;
xTaskHandle xZS_Handle;
xTaskHandle xT0_Handle;
xTaskHandle xT1_Handle;

xQueueHandle xQueueDIM;
xSemaphoreHandle xBinSemaphoreZS;
xSemaphoreHandle xBinSemaphoreT0;
xSemaphoreHandle xBinSemaphoreT1;

 static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	ESP_LOGI(TAG, "[gpio_isr_handler] inside interrupt handler\n");
	if(gpio_num == ZERO_SENSOR){
		ESP_LOGI(TAG, "[gpio_isr_handler] event ZERO_SENSOR \n");
		xSemaphoreGiveFromISR(xBinSemaphoreZS, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			ESP_LOGI(TAG, "[gpio_isr_handler] xHigherPriorityTaskWoken = TRUE - Yield");
			portYIELD_FROM_ISR();
		}
	}
}




/*
 * Timer group0 ISR handler
 * switch on TRIAC
 * switch off TRIAC
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    //timer_spinlock_take(TIMER_GROUP_0);
    timer_idx_t timer_idx = (timer_idx_t) para;
	ESP_LOGI(TAG, "[timer_group0_isr] para = %d", timer_idx);
	//uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
	timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
	
	
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	ESP_LOGI(TAG, "[timer_group0_isr] in ISR");
	

	if (timer_intr & TIMER_INTR_T0) {
		ESP_LOGI(TAG, "[timer_group0_isr] T0 event");
		timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
		xSemaphoreGiveFromISR(xBinSemaphoreT0, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	} 
	else if (timer_intr & TIMER_INTR_T1) {
		ESP_LOGI(TAG, "[timer_group0_isr] T1 event");
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
		xSemaphoreGiveFromISR(xBinSemaphoreT1, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
    } 
    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
	ESP_LOGI(TAG, "[timer_group0_isr] enable alarm in isr");
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    //timer_spinlock_give(TIMER_GROUP_0);
}





/* very high priority task*/
static void  task_isr_handler_ZS(void* arg)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG, "[task_isr_handler_ZS] Priority get = [%d]",  (uint8_t)uxPriority);
	
	/*
	0 - off
	1 - 50 % = 0.005mS
	2 - 100 % = 0.01mS
	*/
	
	/* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = WITH_RELOAD;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER0_INTERVAL_DELAY * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

	fan_event_t received_data;
	uint64_t alarm_value;
	received_data.speed = 0;
	
	for(;;){
		printf("Timer0 is configured  - Cicle - \n");
		ESP_LOGI(TAG, "[task_isr_handler_ZS] Timer0 is configured  - Cicle -");
		xSemaphoreTake(xBinSemaphoreZS, portMAX_DELAY);
		xQueueReceive(xQueueDIM, &received_data, 0);
		switch(received_data.speed){
		case 0:
			gpio_set_level(FAN, 0); //FAN switch off
			break;
		case 1:
			timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
			alarm_value = (uint64_t) SPEED_1 * TIMER_SCALE; // FAN switch on - 50% speed
			ESP_LOGI(TAG, "[t_isr_handler_ZS] Alarm value [%d]",  (uint32_t)alarm_value);
			//printf('Alarm value [%ld] ',  alarm_value);
			timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarm_value); // а до скольки считать?	
			timer_start(TIMER_GROUP_0, TIMER_0); //T0 start
			break;
		case 2:
			gpio_set_level(FAN, 1); //FAN switch on - 100% speed
			break;
		}
	}
}




/* very high priority task*/
static void  task_isr_handler_T0(void* arg)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG, "[task_isr_handler_T0] Priority get = [%d]",  (uint8_t)uxPriority);
	/*
	1 - gpio_set_level(FAN, 1);
	2 - T1 start;
	*/
	
	/* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = WITH_RELOAD;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    timer_init(TIMER_GROUP_0, TIMER_1, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER0_INTERVAL_DELAY * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_group0_isr, (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);

	uint64_t alarm_value;
	
	for(;;){
		//printf("[t_isr_handler_T0] Timer1 is configured  - Cicle - \n");
		ESP_LOGI(TAG, "[task_isr_handler_T0] Timer1 is configured  - Cicle -");
		xSemaphoreTake(xBinSemaphoreT0, portMAX_DELAY);
		timer_pause(TIMER_GROUP_0, TIMER_0); //T0 pause
		timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL);
		alarm_value = (uint64_t) TIMER1_INTERVAL_SWITCH_ON_TRIAC * TIMER_SCALE; // FAN switch on - 50% speed
		ESP_LOGI(TAG, "[t_isr_handler_T0] Alarm value [%d]",  (uint32_t)alarm_value);
		timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, alarm_value); // а до скольки считать?	
		timer_start(TIMER_GROUP_0, TIMER_1); //T1 start
		gpio_set_level(FAN, 1); //FAN switch on 

	}
}


/* very high priority task*/
static void  task_isr_handler_T1(void* arg)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG, "[task_isr_handler_T1] Priority get = [%d]",  (uint8_t)uxPriority);
	/*
	1 - gpio_set_level(FAN, 0);
	*/
	gpio_set_level(FAN, 0); //FAN switch off
	
	for(;;){
		printf("[t_isr_handler_T1]   - Cicle - \n");
		xSemaphoreTake(xBinSemaphoreT1, portMAX_DELAY);
		timer_pause(TIMER_GROUP_0, TIMER_1); //T1 pause
		gpio_set_level(FAN, 0); //FAN switch off
	}
}





static void venting(void* arg)
{
	fan_event_t send_data;
	
	//printf("[venting task] send_data.timer_counter_value = [%d]\n", send_data.speed);
    send_data.speed = 0;
	uint8_t i = 0;
	
    for(;;) {
		while(i <= 2)
		{
			ESP_LOGI(TAG, "[venting] send_data.timer_counter_value = [%d]\n", send_data.speed);
			vTaskDelay(5000 / portTICK_RATE_MS);
			xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
			send_data.speed = send_data.speed + i;
			i++;
		}
		send_data.speed = 0;
		i = 0;
    }
}


void app_main()
{
	/*** triac control ***/
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; //bit mask of the pins that you want to set,e.g.GPIO32/33
    io_conf.pull_down_en = 0; //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings
	/*********************/
	
	/*** zero sensor ***/
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE; //interrupt ANYEDGE
	io_conf.mode = GPIO_MODE_INPUT; //set as input mode
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins, use GPIO34 here
    io_conf.pull_up_en = 1; //enable pull-up mode
    io_conf.pull_down_en = 0; //disable pull-down_cw mode - отключитли подтяжку к земле
    gpio_config(&io_conf);
	
	/*********************/
	
	
	
	//esp_err_t err;
	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);// err = 
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);
	//ESP_ERROR_CHECK(err);
	


	
	xQueueDIM = xQueueCreate(5, sizeof(fan_event_t));
	
	xStatus_venting = xTaskCreate(venting, "test_fan_work", 1024 * 4,  NULL, 9, NULL);
	if(xStatus_venting == pdPASS)
		ESP_LOGI(TAG, "[app_main] Task [test_fan_work] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [test_fan_work] is not created");
	
	vSemaphoreCreateBinary(xBinSemaphoreZS);
	vSemaphoreCreateBinary(xBinSemaphoreT0);
	vSemaphoreCreateBinary(xBinSemaphoreT1);
	
	xStatus_task_isr_handler_ZS = xTaskCreate(task_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 12, &xZS_Handle);
	if(xStatus_task_isr_handler_ZS == pdPASS)
		ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is not created");
	
	
	xStatus_task_isr_handler_T0 = xTaskCreate(task_isr_handler_T0, "task isr handler T0", 1024 * 4,  NULL, 11, &xT0_Handle);
	if(xStatus_task_isr_handler_T0)
		ESP_LOGI(TAG, "[app_main] Task [task isr handler T0] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [task isr handler T0] is not created");
	
	
	xStatus_task_isr_handler_T1 = xTaskCreate(task_isr_handler_T1, "task isr handler T1", 1024 * 4,  NULL, 10, &xT1_Handle);
	if(xStatus_task_isr_handler_T1)
		ESP_LOGI(TAG, "[app_main] Task [task isr handler T1] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [task isr handler T1] is not created");
}
