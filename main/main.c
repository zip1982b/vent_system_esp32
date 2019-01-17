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

#include "rom/ets_sys.h"

/**
 * Brief:
 * 
 * GPIO status:
 * GPIO32: output
 * GPIO33: output
 * GPIO34:  input, interrupt from rising edge and falling edge
 * 
 */
 

#define FAN_IN				32
#define FAN_OUT				33
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN_IN) | (1ULL<<FAN_OUT))

#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))
#define ESP_INTR_FLAG_DEFAULT 0


xQueueHandle xQueueDIM;
 //xQueueHandle xQueueISR;





static void vCHANGE_SPEED(void* arg)
{	
	uint8_t dim = 0;
    for(;;) {
		xQueueReceive(xQueueDIM, &dim, 0);
		printf("delay = %d ms\n", dim);
		gpio_set_level(FAN_IN, 1);
		vTaskDelay(50 / portTICK_RATE_MS);
		gpio_set_level(FAN_IN, 0);
		vTaskDelay(dim / portTICK_RATE_MS);
	}
}

static void ON_OFF_FAN(void* arg)
{	
	uint8_t dim = 0;
    for(;;) {
		dim = 0;
		xQueueSendToBack(xQueueDIM, &dim, 100/portTICK_RATE_MS);
		vTaskDelay(20000 / portTICK_RATE_MS);
		dim = 100;
		xQueueSendToBack(xQueueDIM, &dim, 100/portTICK_RATE_MS);
		vTaskDelay(20000 / portTICK_RATE_MS);
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
    //io_conf.intr_type = GPIO_INTR_ANYEDGE; //interrupt ANYEDGE
    //io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins, use GPIO34 here
    //io_conf.mode = GPIO_MODE_INPUT; //set as input mode
    //io_conf.pull_up_en = 0; //enable pull-up mode
    //io_conf.pull_down_en = 0; //disable pull-down_cw mode - отключитли подтяжку к земле
    //gpio_config(&io_conf);
	
	/*********************/
	
	//install gpio isr service
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);


	
	//xQueueISR = xQueueCreate(5, sizeof(uint32_t));
	
	xQueueDIM = xQueueCreate(10, sizeof(uint8_t));
	
	xTaskCreate(vCHANGE_SPEED, "CHANGE_SPEED", 2048, NULL, 11, NULL);
		
	
    xTaskCreate(ON_OFF_FAN, "on off Fan", 2048, NULL, 10, NULL);
}
