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


#include <unistd.h>
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"


#include "esp_log.h"
#include "mqtt_client.h"
#include "DHT.h"


enum Mode {
	hand,
	avto
};




#define ESP_WIFI_SSID 		CONFIG_ESP_WIFI_SSID 
#define ESP_WIFI_PASS 		CONFIG_ESP_WIFI_PASSWORD 
#define ESP_MAXIMUM_RETRY 	CONFIG_ESP_MAXIMUM_RETRY

/* freertos event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT0




static void startingTRIAC_timer_callback(void* arg);
static void delay_timer_callback(void* arg);

static int s_retry_num = 0;



/**
 * Brief:
 * 
 * GPIO status:
 * GPIO32: output
 * GPIO34:  input, interrupt from rising edge and falling edge
 * 
 */
 
static const char* TAG = "VentSys";
static const char* TAG_reg = "Regulator";
static const char* TAG_mqtt = "MQTT";
static const char* TAG_wifi = "wifi station";


const char topic_DHT22_dataH[] = "/home/bathroom/humi/value";
const char topic_DHT22_dataT[] = "/home/bathroom/temp/value";

const char topic_regulator_vent_speed[] = "/home/bathroom/reg/speed/set";
const char topic_regulator_target_humi[] = "/home/bathroom/reg/target_humi/set";
const char topic_regulator_mode[] = "/home/bathroom/reg/mode/set";









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
xQueueHandle xQueueDHTdata;
xQueueHandle xQueueTargetHumi;
xQueueHandle xQueueMode;
xQueueHandle xQueueSpeed;
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





static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	if(event_base == WIFI_EVENT && 	event_id == WIFI_EVENT_STA_START){
		esp_wifi_connect();
	}
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
		if(s_retry_num < ESP_MAXIMUM_RETRY){
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG_wifi, "retry to connect to the AP");
		}else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG_wifi, "connect to the AP fail");
	}
	else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG_wifi, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}



void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_wifi, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_wifi, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_wifi, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_wifi, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}











static void log_error_if_nonzero(const char *message, int error_code)
{
	    if (error_code != 0) {
		    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	    }
}




static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
	char speed[2];
	char *vent_speed;
	vent_speed = &speed[0];
	uint8_t set_speed;

	char humi[5];
	char *target_humi;
	target_humi = &humi[0];
	float target_humidity;

	char mode[1];
	char *reg_mode;
	reg_mode = &mode[0];
	uint8_t mode_regulator;





    ESP_LOGD(TAG_mqtt, "event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch((esp_mqtt_event_id_t)event_id){
	    case MQTT_EVENT_CONNECTED:
		    ESP_LOGI(TAG_mqtt, "MQTT_EVENT_CONNECTED");
		    esp_mqtt_client_subscribe(client, topic_regulator_vent_speed, 0);
		    esp_mqtt_client_subscribe(client, topic_regulator_target_humi, 0);
		    esp_mqtt_client_subscribe(client, topic_regulator_mode, 0);
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
		    if(strncmp(topic_regulator_vent_speed, event->topic, event->topic_len)==0){
			    ESP_LOGI(TAG_mqtt, "received vent speed set");
			    strncpy(vent_speed, event->data, event->data_len);
			    set_speed = atoi(vent_speed);
			    xQueueSendToBack(xQueueSpeed, &set_speed, 100/portTICK_RATE_MS);
		    }
		    else if(strncmp(topic_regulator_target_humi, event->topic, event->topic_len)==0){
			    ESP_LOGI(TAG_mqtt, "received target humidity");
			    strncpy(target_humi, event->data, event->data_len);
			    target_humidity = atoi(target_humi);
			    xQueueSendToBack(xQueueTargetHumi, &target_humidity, 100/portTICK_RATE_MS);
		    }
		    else if(strncmp(topic_regulator_mode, event->topic, event->topic_len)==0){
			    ESP_LOGI(TAG_mqtt, "received mode");
			    strncpy(reg_mode, event->data, event->data_len);
			    mode_regulator = atoi(reg_mode);
			    xQueueSendToBack(xQueueMode, &mode_regulator, 100/portTICK_RATE_MS);
		    }
		    else ESP_LOGI(TAG_mqtt, "topic strings not equal");
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











void Regulator_task(void *pvParameter)
{
	
    vTaskDelay(3000 / portTICK_RATE_MS);
	
    ESP_LOGI(TAG_reg, "Starting Regulator Task\n\n"); 
    enum Mode Reg_mode;
    Reg_mode = avto;

    setDHTgpio(GPIO_NUM_33);
    float H = 0.0;
    float T;
    dht_data_t dht22;
    int ret;
    
    
    float target_humidity = 15.2;// %
    uint8_t delta = 2;//default delta
    uint8_t Speed = 2;//default speed
    uint8_t sp = 0;

while(1){
        ESP_LOGI(TAG_reg, "=== Reading DHT ===\n");
	ret = readDHT();
	errorHandler(ret);
	H = getHumidity();
	T = getTemperature();
	ESP_LOGI(TAG_reg, "Hum: %.1f Tmp: %.1f", H, T);
	dht22.H = H;
	dht22.T = T;
  	xQueueSendToBack(xQueueDHTdata, &dht22, 100 / portTICK_RATE_MS); //send H and T to MQTT_pub
	xQueueReceive(xQueueSpeed, &Speed, 100 / portTICK_RATE_MS);
	xQueueReceive(xQueueTargetHumi, &target_humidity, 0);
	xQueueReceive(xQueueMode, &Reg_mode, 0);//regulator mode ()

    if(Reg_mode){
		if(H > (target_humidity + delta)){
			/* need low humidity */
			ESP_LOGI(TAG_reg, "[Regulator_task] Fan ON, speed = %d", Speed);
  			xQueueSendToBack(xQueueDIM, &Speed, portMAX_DELAY);
                }
   		else if(H < (target_humidity - delta)){
                         ESP_LOGI(TAG_reg, "[Regulator_task] Fan OFF, speed = 0");
                         xQueueSendToBack(xQueueDIM, &sp, portMAX_DELAY);
                }
    }
    else{
	    xQueueSendToBack(xQueueDIM, &Speed, portMAX_DELAY);
    }
    vTaskDelay(5000 / portTICK_RATE_MS);
}
}











void MQTT_pub(void *pvParameter)
{
	char buf_H[5];
	char buf_T[5];
	portBASE_TYPE xStatus;
        dht_data_t dht22;
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);
	
	while(1){
	    xStatus = xQueueReceive(xQueueDHTdata, &dht22, portMAX_DELAY);
	    if(xStatus == pdPASS){
		sprintf(buf_H, "%1.1f", dht22.H);
		sprintf(buf_T, "%1.1f", dht22.T);
	        esp_mqtt_client_publish(client, topic_DHT22_dataH, buf_H, 0, 0, 0);   
	        esp_mqtt_client_publish(client, topic_DHT22_dataT, buf_T, 0, 0, 0);    
	    }
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
    xQueueDHTdata = xQueueCreate(5, sizeof(dht_data_t));
    xQueueMode = xQueueCreate(5, sizeof(uint8_t));
    xQueueSpeed = xQueueCreate(5, sizeof(uint8_t));
    xQueueTargetHumi = xQueueCreate(5, sizeof(float));
	
	
	
    xStatus_task_isr_handler_ZS = xTaskCreate(task_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 8, NULL); //&xZS_Handle
    if(xStatus_task_isr_handler_ZS == pdPASS)
	ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is created");
    else
	ESP_LOGI(TAG, "[app_main] Task [task isr handler Zero Sensor] is not created");

	
	 //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //esp_log_level_set("*", ESP_LOG_INFO);

    xTaskCreate(&Regulator_task, "Regulator Humi", 2048, NULL, 5, NULL);

    xTaskCreate(&MQTT_pub, "MQTT publish task", 3072, NULL, 5, NULL);




    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_sta();

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
