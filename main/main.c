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





static int s_retry_num = 0;



/* tags */
static const char* TAG = "VentSys";
static const char* TAG_reg = "Regulator";
static const char* TAG_mqtt = "MQTT";
static const char* TAG_wifi = "wifi station";

/* topics for mqtt */
const char topic_DHT22_dataH[] = "/home/bathroom/humi/value";
const char topic_DHT22_dataT[] = "/home/bathroom/temp/value";

const char topic_regulator_vent_speed[] = "/home/bathroom/reg/speed/set";
const char topic_regulator_target_humi[] = "/home/bathroom/reg/target_humi/set";
const char topic_regulator_mode[] = "/home/bathroom/reg/mode/set";

const char topic_regulator_vent_speed_v[] = "/home/bathroom/reg/speed/value";
const char topic_regulator_target_humi_v[] = "/home/bathroom/reg/target_humi/value";
const char topic_regulator_mode_v[] = "/home/bathroom/reg/mode/value";

/* 
 *
 *GPIO STATUS:
 *GPIO32 - output
 *GPIO34 - input, interrupt from rising edge and falling edge
 *
 * */


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

xQueueHandle xQueueDIM;
xQueueHandle xQueueDHTdata;

xQueueHandle xQueueSpeeddata;
xQueueHandle xQueueModedata;
xQueueHandle xQueueTargetHumidata;


xQueueHandle xQueueTargetHumi;
xQueueHandle xQueueMode;
xQueueHandle xQueueSpeed;
xSemaphoreHandle xBinSemaphoreZS;






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
	uint8_t target_humidity;

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

























void Regulator_task(void *pvParameter)
{
    vTaskDelay(5000 / portTICK_RATE_MS);

    portBASE_TYPE xStatus;
//    ESP_LOGI(TAG_reg, "Starting Regulator Task\n\n"); 
    enum Mode Reg_mode;
    Reg_mode = avto;

    setDHTgpio(GPIO_NUM_33);
    float H = 0.0;
    float T;
    dht_data_t dht22;
    int ret;
    
    
    uint8_t target_humidity = 55;/* % */
    uint8_t delta = 2;//default delta
    uint8_t fan_on = 0;//default speed
  //  uint8_t sp = 0;

while(1){
 //       ESP_LOGI(TAG_reg, "=== Reading DHT ===\n");
	ret = readDHT();
	errorHandler(ret);
	if (ret == 0){
		H = getHumidity();
		T = getTemperature();
//		ESP_LOGI(TAG_reg, "Hum: %.1f Tmp: %.1f", H, T);
		dht22.H = H;
		dht22.T = T;
	}
	xQueueReceive(xQueueSpeed, &fan_on, 0);
	xStatus = xQueueReceive(xQueueTargetHumi, &target_humidity, 0);
	if(xStatus == pdPASS){
		ESP_LOGI(TAG_reg, "[Regulator_task] target_humidity = %d", target_humidity);
	}
	xQueueReceive(xQueueMode, &Reg_mode, 0);//regulator mode (auto, hand)

  	xQueueSendToBack(xQueueDHTdata, &dht22, 0); //send H and T to MQTT_pub


    if(Reg_mode){
	if(H >= (target_humidity + delta)){
	/* need low humidity */
		ESP_LOGI(TAG_reg, "[Regulator_task] Avto mode - Fan ON");
		gpio_set_level(FAN, 1); //FAN switch on - 100% speed
  		//xQueueSendToBack(xQueueDIM, &Speed, 0);
        }
   	else if(H < (target_humidity - delta)){
	
                ESP_LOGI(TAG_reg, "[Regulator_task] Avto mode - Fan OFF");
		gpio_set_level(FAN, 0); //FAN switch off
        }
    }
    else{
	    if(fan_on){

                ESP_LOGI(TAG_reg, "[Regulator_task] Hand mode  - Fan on");
		gpio_set_level(FAN, 1); //FAN switch on - 100% speed
	    }
	    else {
                ESP_LOGI(TAG_reg, "[Regulator_task] Hand mode  - Fan off");
		gpio_set_level(FAN, 0); //FAN switch off
	    }
    }

    xQueueSendToBack(xQueueTargetHumidata, &target_humidity, 0); //send Terget Humi to MQTT_pub
    xQueueSendToBack(xQueueModedata, &Reg_mode, 0); //send Mode to MQTT_pub
    vTaskDelay(5000 / portTICK_RATE_MS);
}
}











void MQTT_pub(void *pvParameter)
{
	char buf_H[5];
	char buf_T[5];
	portBASE_TYPE xStatus;

	portBASE_TYPE xStatus1;
	char buf_speed[5];
	uint8_t speed = 0;


	portBASE_TYPE xStatus2;
	char buf_target_h[5];
	uint8_t target_humi = 0;


	portBASE_TYPE xStatus3;
	char buf_mode[5];
	uint8_t mode = 0;


        dht_data_t dht22;

	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);
	
	while(1){
	    xStatus = xQueueReceive(xQueueDHTdata, &dht22, 0);
	    if(xStatus == pdPASS){
		sprintf(buf_H, "%1.1f", dht22.H);
		sprintf(buf_T, "%1.1f", dht22.T);
	        esp_mqtt_client_publish(client, topic_DHT22_dataH, buf_H, 0, 0, 0);   
	        esp_mqtt_client_publish(client, topic_DHT22_dataT, buf_T, 0, 0, 0);    
	    }

	    xStatus1 = xQueueReceive(xQueueSpeeddata, &speed, 0);
	    if(xStatus1 == pdPASS){
		sprintf(buf_speed, "%d", speed);
	        esp_mqtt_client_publish(client, topic_regulator_vent_speed_v, buf_speed, 0, 0, 0);   
	    }

	    xStatus2 = xQueueReceive(xQueueTargetHumidata, &target_humi, 0);
	    if(xStatus2 == pdPASS){
		sprintf(buf_target_h, "%d", target_humi);
	        esp_mqtt_client_publish(client, topic_regulator_target_humi_v, buf_target_h, 0, 0, 0);   
	    }


	    xStatus3 = xQueueReceive(xQueueModedata, &mode, 0);
	    if(xStatus3 == pdPASS){
		sprintf(buf_mode, "%d", mode);
	        esp_mqtt_client_publish(client, topic_regulator_mode_v, buf_mode, 0, 0, 0);   
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
	
	
	
	
    vSemaphoreCreateBinary(xBinSemaphoreZS);
    xQueueDIM = xQueueCreate(5, sizeof(fan_event_t));
    xQueueDHTdata = xQueueCreate(5, sizeof(dht_data_t));
    xQueueMode = xQueueCreate(5, sizeof(uint8_t));
    xQueueSpeed = xQueueCreate(5, sizeof(uint8_t));
    xQueueTargetHumi = xQueueCreate(5, sizeof(float));
	
    xQueueSpeeddata = xQueueCreate(5, sizeof(uint8_t));
    xQueueModedata = xQueueCreate(5, sizeof(uint8_t));
    xQueueTargetHumidata = xQueueCreate(5, sizeof(uint8_t));



    
	
	 //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_sta();


    vTaskDelay(5000 / portTICK_RATE_MS);


    xTaskCreate(&Regulator_task, "Regulator Humi", 2048, NULL, 6, NULL);

    xTaskCreate(&MQTT_pub, "MQTT publish task", 3072, NULL, 5, NULL);
}



