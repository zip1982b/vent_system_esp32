/* ventilation system progect

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <unistd.h>
#include "esp_timer.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "esp_log.h"






static void startingTRIAC_timer_callback(void* arg);
static void delay_timer_callback(void* arg);



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
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN))

#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))

#define ESP_INTR_FLAG_DEFAULT 0





typedef struct {
    uint8_t speed; //0, 1 ,2
} fan_event_t;


portBASE_TYPE xStatus_venting;
portBASE_TYPE xStatus_task_isr_handler_ZS;


xTaskHandle xVenting_Handle;
xTaskHandle xZS_Handle;

//static xQueueHandle gpio_evt_queue = NULL;
xQueueHandle xQueueDIM;
xSemaphoreHandle xBinSemaphoreZS;


 static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if(gpio_num == ZERO_SENSOR){
		xSemaphoreGiveFromISR(xBinSemaphoreZS, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
}






/* very high priority task*/
static void  task_isr_handler_ZS(void* arg)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG, "[task_isr_handler_ZS] Priority get = [%d]",  (uint8_t)uxPriority);
	
	/* Create a one-shot timer for starting TRIAC */
	const esp_timer_create_args_t startingTRIAC_timer_args = {
            .callback = &startingTRIAC_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "starting TRIAC"
    };
    esp_timer_handle_t startingTRIAC_timer;
    ESP_ERROR_CHECK(esp_timer_create(&startingTRIAC_timer_args, &startingTRIAC_timer));
	
	
	
	/* Create a one-shot timer for delay RMS */
	const esp_timer_create_args_t delay_timer_args = {
            .callback = &delay_timer_callback,
			.arg = (void*) startingTRIAC_timer,
            .name = "delay timer"
    };
	esp_timer_handle_t delay_timer;
    ESP_ERROR_CHECK(esp_timer_create(&delay_timer_args, &delay_timer));
	
	
	uint32_t io_num;
	fan_event_t received_data;
	uint8_t speed = 0;
	ESP_LOGI(TAG, "[task_isr_handler_ZS]esp_timer is configured");
	
	for(;;){
		//ESP_LOGI(TAG, "[task_isr_handler_ZS] - Cicle -");
		
		xSemaphoreTake(xBinSemaphoreZS, portMAX_DELAY);
		//xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] ionum = %d", io_num);
		
		
		xQueueReceive(xQueueDIM, &received_data, 0);
		speed = received_data.speed;
		
		//ESP_LOGI(TAG, "[task_isr_handler_ZS] speed = %d", speed);
		switch(speed){
		case 0:
			gpio_set_level(FAN, 0); //FAN switch off
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] Fan off");
			break;
		case 1:
			/* Start the one-shot timer */
			esp_timer_start_once(delay_timer, 5000);
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] Started timer, time since boot: %lld us", esp_timer_get_time());
			break;
		case 2:
			gpio_set_level(FAN, 1); //FAN switch on - 100% speed
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] FAN ON speed = 2");
			break;
		}
	}
}



static void venting(void* arg)
{
	fan_event_t send_data;
    //send_data.speed = 1;
	
	
    for(;;) {
		send_data.speed = 0;
		//xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
		//ESP_LOGI(TAG, "[venting] send_data.speed = %d\n", send_data.speed);
		//vTaskDelay(3000 / portTICK_RATE_MS);
		
		for(uint8_t i=0; i<=2; i++){
			send_data.speed = i;
			ESP_LOGI(TAG, "[venting] send_data.speed = %d\n", send_data.speed);
			xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
			vTaskDelay(5000 / portTICK_RATE_MS);
			ESP_LOGI(TAG, "[venting] i = [%d]\n", i);
		}
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
    io_conf.pull_up_en = 0; //enable pull-up mode
    io_conf.pull_down_en = 0; //disable pull-down_cw mode - отключитли подтяжку к земле
    gpio_config(&io_conf);
	/*********************/
	
	
	
	esp_err_t err;
	//install gpio isr service
    err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);//  
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);
	ESP_ERROR_CHECK(err);
	
	
	vSemaphoreCreateBinary(xBinSemaphoreZS);
	xQueueDIM = xQueueCreate(5, sizeof(fan_event_t));
	
	xStatus_venting = xTaskCreate(venting, "test_fan_work", 2048,  NULL, 5, NULL);
	if(xStatus_venting == pdPASS)
		ESP_LOGI(TAG, "[app_main] Task [test_fan_work] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [test_fan_work] is not created");
	
	
	xStatus_task_isr_handler_ZS = xTaskCreate(task_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 8, NULL); //&xZS_Handle
	if(xStatus_task_isr_handler_ZS == pdPASS)
		ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is created");
	else
		ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is not created");

	//xTaskCreate(terminator, "task terminator", 1024,  NULL, 9, NULL);

}






static void delay_timer_callback(void* arg)
{
    //int64_t time_since_boot = esp_timer_get_time();
    esp_timer_handle_t startingTRIAC_timer_handle = (esp_timer_handle_t) arg;
	gpio_set_level(FAN, 1); //FAN switch on 
	//ESP_LOGI(TAG, "[task_isr_handler_ZS] Fan on half");
    /* To start the timer which is running, need to stop it first */
    ESP_ERROR_CHECK(esp_timer_start_once(startingTRIAC_timer_handle, 50));
    //ESP_LOGI(TAG, "[delay_timer_callback] startingTRIAC_timer start once");
}


static void startingTRIAC_timer_callback(void* arg)
{
	gpio_set_level(FAN, 0); //FAN switch off
}
