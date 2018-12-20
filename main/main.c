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
#include "driver/gpio.h"

/**
 * Brief:
 * This test code configure gpio and use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */
 

#define FAN_IN				32
#define FAN_OUT				33
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN_IN) | (1ULL<<FAN_OUT))

#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))
#define ESP_INTR_FLAG_DEFAULT 0
 
 xSemaphoreHandle xBinarySemaphore;
 
 
 static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	
	if(gpio_num == ZERO_SENSOR){
		gpio_set_intr_type(ZERO_SENSOR, GPIO_INTR_DISABLE);
		
	}
	
}




static void vCHANGE_SPEED(void* arg)
{
	//portBASE_TYPE xStatus;
    uint32_t io_num;
    for(;;) {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		
    }
}





static void ON_OFF_FAN(void* arg)
{
    
    for(;;) {
        //gpio_set_level(FAN_IN, 0);
		vTaskDelay(5000 / portTICK_RATE_MS);
        //gpio_set_level(FAN_IN, 1);
		vTaskDelay(5000 / portTICK_RATE_MS);
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
    io_conf.intr_type = GPIO_INTR_ANYEDGE; //interrupt ANYEDGE
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins, use GPIO34 here
    io_conf.mode = GPIO_MODE_INPUT; //set as input mode
    io_conf.pull_up_en = 0; //enable pull-up mode
    io_conf.pull_down_en = 0; //disable pull-down_cw mode - отключитли подтяжку к земле
    gpio_config(&io_conf);
	/*********************/
	
	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);




	
	vSemaphoreCreateBinary(xBinarySemaphore);
	if(xBinarySemaphore != NULL){
		xTaskCreate(vCHANGE_SPEED, "CHANGE_SPEED", 1000, NULL, 11, NULL);
		
	}	
    xTaskCreate(ON_OFF_FAN, "on off Fan", 2048, NULL, 10, NULL);
}
