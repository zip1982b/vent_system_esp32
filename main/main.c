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
 
 
 /*
 static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	
	if(gpio_num == ZERO_SENSOR){
		gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_DISABLE);
		
	}
	
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, 0);
}
*/

/*

static void CHANGE_SPEED(void* arg)
{
	
	enum action rotate;
	//portBASE_TYPE xStatus;
    uint32_t io_num;
    for(;;) {
		io_num = 0;
		xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY); // the task is blocked until the data arrives
		if(io_num == GPIO_ENC_CLK)
		{
			xQueueReceive(gpio_evt_queue, &io_num, 100/portTICK_RATE_MS);
			gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_ANYEDGE); //enable
			if (io_num == GPIO_ENC_DT)
			{
				rotate = cr;
				printf("[ENC]clockwise rotation\n");
				xQueueSendToBack(ENC_queue, &rotate, 100/portTICK_RATE_MS); //xStatus = 
				gpio_set_intr_type(GPIO_ENC_DT, GPIO_INTR_ANYEDGE);//enable
				
			}
				
		}	
		else if(io_num == GPIO_ENC_DT)
		{
			//vTaskDelay(10 / portTICK_RATE_MS);
			xQueueReceive(gpio_evt_queue, &io_num, 100/portTICK_RATE_MS);
			gpio_set_intr_type(GPIO_ENC_DT, GPIO_INTR_ANYEDGE);
			
			if (io_num == GPIO_ENC_CLK)
			{
				rotate = ccr;
				//printf("[ENC]rotate = %d\n", rotate);
				printf("[ENC]counter clockwise rotation\n");
				xQueueSendToBack(ENC_queue, &rotate, 100/portTICK_RATE_MS);//xStatus = 
				gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_ANYEDGE);
			}
				
		}
		else if(io_num == GPIO_ENC_SW)
		{
			rotate = bp;
			//printf("[ENC]rotate = %d\n", rotate);
			printf("[ENC]Button is pressed\n");
			vTaskDelay(300 / portTICK_RATE_MS);
			xQueueSendToBack(ENC_queue, &rotate, 100/portTICK_RATE_MS);//xStatus = 
			gpio_set_intr_type(GPIO_ENC_SW, GPIO_INTR_NEGEDGE);//enable
			// 
		}
    }
}


*/


static void ON_OFF_FAN(void* arg)
{
    
    for(;;) {
        gpio_set_level(FAN_IN, 0);
		vTaskDelay(5000 / portTICK_RATE_MS);
        gpio_set_level(FAN_IN, 1);
		vTaskDelay(5000 / portTICK_RATE_MS);
    }
}




void app_main()
{
    gpio_config_t io_conf;
	//disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO32/33
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	
	
	
	
	
    // interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    //bit mask of the pins, use GPIO34/32/33 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
	
    //enable pull-up mode
    io_conf.pull_up_en = 0;
	
	//disable pull-down_cw mode - отключитли подтяжку к земле
    io_conf.pull_down_en = 0;
	
    gpio_config(&io_conf);
	
	//install gpio isr service
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);




	

    //create a queue to handle gpio event from isr
    //gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
	
	
    xTaskCreate(ON_OFF_FAN, "on off Fan", 2048, NULL, 10, NULL);

    
	
}
