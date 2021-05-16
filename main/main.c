/* ventilation system progect

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "esp_wifi.h"
#include "esp_system.h"

#include "nvs_flash.h"

#include "esp_event.h"


#include "esp_netif.h"

#include "esp_protocol_examples_common.h"

#include <unistd.h>
#include "esp_timer.h"

#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "DHT.h"


enum Mode {
	hand,
	avto
};




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
static const char* TAG_dht = "DHT22";

static const char* TAG_reg = "Regulator";

static const char* TAG_mqtt = "MQTT";




#define FAN				32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN))

#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))

#define ESP_INTR_FLAG_DEFAULT 0





typedef struct {
    uint8_t speed; //0, 1 ,2
} fan_event_t;


typedef struct {
	float H;
	float T;
} dht_data_t;




portBASE_TYPE xStatus_venting;
portBASE_TYPE xStatus_task_isr_handler_ZS;


xTaskHandle xVenting_Handle;
xTaskHandle xZS_Handle;

//static xQueueHandle gpio_evt_queue = NULL;
xQueueHandle xQueueDIM;
xQueueHandle xQueueHumi;
xQueueHandle xQueueDHTdata;

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


static void log_error_if_nonzero(const char *message, int error_code)
{
	    if (error_code != 0) {
		    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	    }
}




static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    ESP_LOGD(TAG_mqtt, "event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch((esp_mqtt_event_id_t)event_id){
	    case MQTT_EVENT_CONNECTED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_CONNECTED");
		    break;
	    case MQTT_EVENT_DISCONNECTED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_DISCONNECTED");
		    break;
	    case MQTT_EVENT_SUBSCRIBED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		    break;
	    case MQTT_EVENT_UNSUBSCRIBED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		    break;
	    case MQTT_EVENT_PUBLISHED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		    break;
	    case MQTT_EVENT_DATA:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_DATA");
		    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
		    printf("DATA=%.*s\r\n", event->data_len, event->data);
                    break;
	    case MQTT_EVENT_ERROR:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_ERROR");
		    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
			    log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
			    log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
			    log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
			    ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
		    }
		    break;
	    default:

		    ESP_LOGI(TAG_mqtt, "other event id:%d", event->event_id);
		    break;

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
	
	
//	uint32_t io_num;
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





void DHT_task(void *pvParameter)
{
    setDHTgpio(GPIO_NUM_33);
    ESP_LOGI(TAG_dht, "Starting DHT Task\n\n");
    float H;
    float T;
    dht_data dht22;
    
	
    while (1)
    {
        ESP_LOGI(TAG_dht, "=== Reading DHT ===\n");
        int ret = readDHT();

        errorHandler(ret);
		
	H = getHumidity();
	T = getTemperature();
	ESP_LOGI(TAG_dht, "Hum: %.1f Tmp: %.1f", H, T);	
	dht22.H = H;
	dht22.T = T;	
  	xQueueSendToBack(xQueueHumi, &H, 100 / portTICK_RATE_MS); //send H to regulator
  	xQueueSendToBack(xQueueDHTdata, &dht22, 100 / portTICK_RATE_MS); //send H and T to regulator
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}






void Regulator_task(void *pvParameter)
{
    enum Mode Reg_mode;
    Reg_mode = avto;

    float target_humidity = 45.2;// %
    ESP_LOGI(TAG_dht, "Starting DHT Task\n\n"); 
    float H = 0.0;
    uint8_t delta = 2;//default delta
    uint8_t speed = 2;//default speed


    
while(1){

		xQueueReceive(xQueueHumi, &H, 0);
		
		xQueueReceive(xQueueSpeed, &speed, 0);
		
		xQueueReceive(xQueueHumi, &target_humidity, 0);

    if(Reg_mode){
		
		ESP_LOGI(TAG_reg, "[Regulator task]Recivied humidity = %f", H);

		if(H > target_humidity + delta){
			ESP_LOGI(TAG_reg, "[Regulator_task] Fan speed = %d", speed);
  			xQueueSendToBack(xQueueDIM, &speed, portMAX_DELAY);
                }
   		else if(H < target_humidity - delta){
                         ESP_LOGI(TAG_reg, "[Regulator_task] Fan speed = 0");
			 speed = 0;
                         xQueueSendToBack(xQueueDIM, &speed, portMAX_DELAY);
                }

    }
    else{
	    xQueueSendToBack(xQueueDIM, &speed, portMAX_DELAY);
    }
    vTaskDelay(3000 / portTICK_RATE_MS);
}
}

void MQTT_pub(void *pvParameter)
{
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);
	
	while(1){

	}





} 


















void app_main()
{
    ESP_LOGI(TAG, "[app] startup ...");
    ESP_LOGI(TAG, "[app] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[app] IDF version: %s", esp_get_idf_version());
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
    xQueueHumi = xQueueCreate(5, sizeof(float));
    xQueueDHTdata = xQueueCreate(5, suzeof(dht_data_t));

	
	
	
    xStatus_task_isr_handler_ZS = xTaskCreate(task_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 8, NULL); //&xZS_Handle
    if(xStatus_task_isr_handler_ZS == pdPASS)
	ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is created");
    else
	ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is not created");

	
	 //Initialize NVS
    //esp_err_t ret = nvs_flash_init();
    //if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    //{
        //ESP_ERROR_CHECK(nvs_flash_erase());
        //ret = nvs_flash_init();
    //}
    //ESP_ERROR_CHECK(ret);

    //esp_log_level_set("*", ESP_LOG_INFO);

    xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
    xTaskCreate(&Regulator_task, "Regulator Humi", 2048, NULL, 5, NULL);

    xTaskCreate(&MQTT_pub, "MQTT publish task", 2048, NULL, 5, NULL);

    ESP_ERROR_CHECK(example_connect());


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
