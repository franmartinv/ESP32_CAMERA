/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* 		ESP32-CAM main project
* 		----------------------
*
* 		In this project you can configure the ESP32-CAM and work with it. The driver is based on the official ESP32-CAM with the OV2640 sensor.
*
* 		Change the OV2640 sensor config with the 'camera_config_t' struct.
*
* 		Modify the MQTT broker IP using the 'idf.py menuconfig' as well as the Wifi config.
*
* 		This program is made to use with NodeRed. You can see the diagram flow in the current folder of this repository. Open 'NodeRed Flow' folder!!.
* 		The capture images will appear on: 			C:\tfg-sensors-data\photos\
*
* 		NOTE: There is a principal parameter that is used to change the delay between two photo captures. You can change it modifying the parameter: "CAM_DELAY".
*
*
* 		Francisco Martín Villegas
* 		Electronic Industry and Automatic Engineering
* 		University of Almería, 2022
*
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_spiffs.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "unity.h"
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <mbedtls/base64.h>
#include <hal/ledc_types.h>

#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

#define					MAC_LEN				7

#define					number_samples			5
#define					MAX_DELAY			3600000		// 3600000 	milliseconds = 3600 seconds = 1 hour
#define					MIN_DELAY			5000		// 5000		milliseconds = 5	seconds
#define					DELAY_SIZE			20
#define					TIMEZONE			2		// timezone-zone for spain: UTC +2
#define					YEAR_OFFSET			1900
#define					MONTH_OFFSET			1

static const char 			*TAG = "ESP_CAM_MQTT";
static 	int				CAM_DELAY		=	1200000;	// Actual delay time between cams: 20 minutes

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// ESP32Cam (AiThinker) PIN Map
#define CAM_PIN_PWDN 		32
#define CAM_PIN_RESET 		-1 //software reset will be performed
#define CAM_PIN_XCLK 		0
#define CAM_PIN_SIOD 		26
#define CAM_PIN_SIOC 		27

#define CAM_PIN_D7 		35
#define CAM_PIN_D6 		34
#define CAM_PIN_D5 		39
#define CAM_PIN_D4 		36
#define CAM_PIN_D3 		21
#define CAM_PIN_D2 		19
#define CAM_PIN_D1 		18
#define CAM_PIN_D0 		5
#define CAM_PIN_VSYNC 		25
#define CAM_PIN_HREF 		23
#define CAM_PIN_PCLK 		22


static camera_config_t camera_config = {
		.pin_pwdn  = CAM_PIN_PWDN,
		.pin_reset = CAM_PIN_RESET,
		.pin_xclk = CAM_PIN_XCLK,
		.pin_sscb_sda = CAM_PIN_SIOD,
		.pin_sscb_scl = CAM_PIN_SIOC,

		.pin_d7 = CAM_PIN_D7,
		.pin_d6 = CAM_PIN_D6,
		.pin_d5 = CAM_PIN_D5,
		.pin_d4 = CAM_PIN_D4,
		.pin_d3 = CAM_PIN_D3,
		.pin_d2 = CAM_PIN_D2,
		.pin_d1 = CAM_PIN_D1,
		.pin_d0 = CAM_PIN_D0,
		.pin_vsync = CAM_PIN_VSYNC,
		.pin_href = CAM_PIN_HREF,
		.pin_pclk = CAM_PIN_PCLK,

		.xclk_freq_hz = 20000000,			
		.ledc_timer = LEDC_TIMER_0,
		.ledc_channel = LEDC_CHANNEL_0,

		.pixel_format = PIXFORMAT_JPEG,			//YUV422,GRAYSCALE,RGB565,JPEG
		.frame_size = FRAMESIZE_UXGA,			//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

		.jpeg_quality = 63, 				//0-63 lower number means higher quality
		.fb_count = 1, 					//if more than one, i2s runs in continuous mode. Use only with JPEG
		.grab_mode = CAMERA_GRAB_WHEN_EMPTY		//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};


#define	FRAMESIZE_STRING			"0"
#define ENABLE_EXTERNAL_FLASH_STORAGE		0

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


/**
 * @brief	Function to compare mac_base string with ID of ESP32-CAM
 *
 * @param[in]	*mac_base	:	(char) pointer to string that contains the mac_base address of the device
 *
 */
uint8_t get_id_from_mac(char *mac_base) {
	uint8_t		MAC_1[6] = {244, 207, 162, 152, 237, 96};
	uint8_t		MAC_2[6] = {244, 207, 162, 154, 18, 232};
	char		TOPIC_ID_1[6], TOPIC_ID_2[6], MAC_copy[6];

	memcpy(TOPIC_ID_1, MAC_1, MAC_LEN-1);
	memcpy(TOPIC_ID_2, MAC_2, MAC_LEN-1);
	memcpy(MAC_copy, mac_base, MAC_LEN-1);

	if (memcmp(TOPIC_ID_1, MAC_copy, MAC_LEN-2) == 0) return 1; //memcmp ( TOPIC_ID_1, mac_base,MAC_LEN )

	else if (memcmp(TOPIC_ID_2, MAC_copy, MAC_LEN-2) == 0) return 2;

	else return 0;
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief 	Event handler registered to receive MQTT events
 *
 *  		This function is called by the MQTT client event loop.
 *
 * @param[in] handler_args	: 	user data registered to the event.
 * @param[in] base		:	Event base for the handler(always MQTT Base in this example).
 * @param[in] event_id 		:	The id for the received event.
 * @param[in] event_data 	:	The data for the event, esp_mqtt_event_handle_t.
 *
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	int				i;
	char			read_topic[50], read_data[50];
	esp_err_t		ret;

	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "ESP_control/run", 1);
        ESP_LOGI(TAG, "sent subscribe to ESP_control/run successful!!, msg_id = %d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "ESP_control/delay", 1);
		ESP_LOGI(TAG, "sent subscribe to ESP_control/delay successful!!, msg_id = %d", msg_id);

		msg_id = esp_mqtt_client_subscribe(client, "ESP_CAM/photo", 1);
		ESP_LOGI(TAG, "sent subscribe to ESP_CAM/photo successful!!, msg_id = %d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        for(i = 0; i <= event->topic_len; i++) {
        	if(i == event->topic_len) read_topic[i] = '\0';
        	else read_topic[i] = event->topic[i];
        }

        for(i = 0; i <= event->data_len; i++) {
        	if(i == event->data_len) read_data[i] = '\0';
        	else read_data[i] = event->data[i];
        }

        if(strcmp(read_topic, "ESP_control/run") == 0) {
        	if(strcmp(read_data, "STOP") == 0) {
				ret = esp_mqtt_client_publish(client, "ESP_control", "MQTT SERVICE STOPPED", 0, 1, 0);
					if(ret == -1) {
						printf("ERROR sending 'MQTT SERVICE STOPPED' to topic 'ESP_control'\n");
					}
				ESP_LOGI(TAG, "MQTT SERVICE STOPPED received message");

				ret = esp_mqtt_client_disconnect(client);
				if(ret != ESP_OK) printf("ERROR disconnecting MQTT client: %x\n", ret);
				else ESP_LOGI(TAG, "MQTT client succesful disconnected !!");
			}
			else if (strcmp(read_data, "START") == 0) {
				ret = esp_mqtt_client_start(client);
				if(ret != ESP_OK) {
					ret = esp_mqtt_client_reconnect(client);
					if(ret != ESP_OK) {
						ESP_LOGE(TAG, "ERROR reconnecting MQTT client!!!");
					}
				}

				ret = esp_mqtt_client_publish(client, "ESP_control", "MQTT SERVICE STARTED", 0, 1, 0);
					if(ret == -1) {
						printf("ERROR sending 'MQTT SERVICE STARTED' to topic 'ESP_control'\n");
					}
				ESP_LOGI(TAG, "MQTT SERVICE STARTED");
			}
        }

        else if(strcmp(read_topic, "ESP_CAM/photo") == 0) {
        	if(strcmp(read_data, "capture") == 0) {
				// Function to take a photo
				camera_fb_t * fb = esp_camera_fb_get();
				if (!fb) {
					ESP_LOGE(TAG, "Camera Capture Failed");
				}
				else ESP_LOGI(TAG, "Picture taken! Its size was: %d bytes", fb->len);

				printf("Captured image's parameters:\nSize: %d\nWidth: %d\nHeight: %d\n\n", fb->len, fb->width, fb->height);

				ret = esp_mqtt_client_publish(client, "ESP_CAM/photo", (char *)fb->buf, fb->len, 1, 0);
				if(ret < 0) {
					ESP_LOGE(TAG, "FAIL sending photo data!!");
				}
				else ESP_LOGI(TAG, "Photo data sent successful");

				//return the frame buffer back to the driver
				esp_camera_fb_return(fb);
        	}
        }

        else if(strcmp(read_topic, "ESP_CAM/delay") == 0) {
        	CAM_DELAY = atoi(event->data);	//	convert char* to float variable

			if(CAM_DELAY > MAX_DELAY) {
				CAM_DELAY = MAX_DELAY;
			}
			else if(CAM_DELAY < MIN_DELAY) {
				CAM_DELAY = MIN_DELAY;
			}

			ESP_LOGI(TAG, "DELAY SELECTED: %d", CAM_DELAY);
        }


        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


/**
 * @brief	Camera initialition
 *
 */
esp_err_t	init_camera()
{
	esp_err_t		ret;

	// Initialize the camera
	ret = esp_camera_init(&camera_config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Camera Init Failed: %x", ret);
		return ret;
	}
	else {
		ESP_LOGI(TAG, "Camera successfull initialited with resolution: %s", FRAMESIZE_STRING);
	}

	return ret;
}


/**
 * @brief	Function for enable SPIFFS external storage. This enables the storage in a external flash.
 *
 */
esp_err_t SPIFFS_external_storage(char * partition_label, char * base_path)
{
	ESP_LOGI(TAG, "Initializing SPIFFS file system");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = base_path,
		.partition_label = partition_label,
		.max_files = 5,
		.format_if_mount_failed = true
	};

	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem!!");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition!!");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS!!: %x", ret);
		}
		return ret;
	}

	size_t 	total;
	size_t	used;

	total = 0;
	used = 0;

	ret = esp_spiffs_info(partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information!!: %x", ret);
	} else {
		ESP_LOGI(TAG, "Partition size -> total: %d used: %d", total, used);
	}
	ESP_LOGI(TAG, "SPIFFS filesystem mounted!!");
	return ret;
}


void app_main(void)
{
	wifi_ap_record_t			ap_info;
	esp_err_t				ret;
	char					mac_address[MAC_LEN];
	uint8_t	 				mac_base[MAC_LEN] = {0};
	uint8_t 				mac_local_base[MAC_LEN];
	uint8_t 				mac_uni_base[MAC_LEN] = {0};

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if(ENABLE_EXTERNAL_FLASH_STORAGE == 1) {
		char *partition_label = "storage";
		char *base_path = "/spiffs";

		ret = SPIFFS_external_storage(partition_label, base_path);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "mountSPIFFS fail");
			while(1) { vTaskDelay(1); }
		}
	}

	#if CONFIG_FRAMESIZE_240X240
		int framesize = FRAMESIZE_240X240;
		#define	FRAMESIZE_STRING "240x240"
	#elif CONFIG_FRAMESIZE_QVGA
		int framesize = FRAMESIZE_QVGA;
		#define	FRAMESIZE_STRING "320x240"
	#elif CONFIG_FRAMESIZE_HVGA
		int framesize = FRAMESIZE_HVGA;
		#define	FRAMESIZE_STRING "480x320"
	#elif CONFIG_FRAMESIZE_VGA
		int framesize = FRAMESIZE_VGA;
		#define	FRAMESIZE_STRING "640x480"
	#elif CONFIG_FRAMESIZE_SVGA
		int framesize = FRAMESIZE_SVGA;
		#define	FRAMESIZE_STRING "800x600"
	#elif CONFIG_FRAMESIZE_XGA
		int framesize = FRAMESIZE_XGA;
		#define	FRAMESIZE_STRING "1024x768"
	#elif CONFIG_FRAMESIZE_HD
		int framesize = FRAMESIZE_HD;
		#define	FRAMESIZE_STRING "1280x720"
	#elif CONFIG_FRAMESIZE_SXGA
		int framesize = FRAMESIZE_SXGA;
		#define	FRAMESIZE_STRING "1280x1024"
	#elif CONFIG_FRAMESIZE_UXGA
		int framesize = FRAMESIZE_UXGA;
		#define	FRAMESIZE_STRING "1600x1200"
	#endif

	vTaskDelay(100/portTICK_RATE_MS);

	init_camera();

	vTaskDelay(100/portTICK_RATE_MS);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Initialition of mqtt

    esp_mqtt_client_config_t mqtt_cfg = {
			.uri = CONFIG_BROKER_URL,
			.out_buffer_size = 4096,
		};

	#if CONFIG_BROKER_URL_FROM_STDIN
		char line[128];

		if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
			int count = 0;
			printf("Please enter url of mqtt broker\n");
			while (count < 128) {
				int c = fgetc(stdin);
				if (c == '\n') {
					line[count] = '\0';
					break;
				} else if (c > 0 && c < 127) {
					line[count] = c;
					++count;
				}
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			mqtt_cfg.uri = line;
			printf("Broker url: %s\n", line);
		} else {
			ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
			abort();
		}
	#endif /* CONFIG_BROKER_URL_FROM_STDIN */

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	/* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);

    // End initialition of mqtt

    vTaskDelay(1000/portTICK_RATE_MS);

    while(1) {
		if(ap_info.rssi == 0) {
			do {
				ESP_ERROR_CHECK(example_connect());
				vTaskDelay(4000/portTICK_RATE_MS);
			} while(ap_info.rssi == 0);
		}

		printf("Connecting power: %i [dB]\n\n", ap_info.rssi);

    	// Functions to obtain MAC address
    	esp_efuse_mac_get_default(mac_base);
		esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
		esp_derive_local_mac(mac_local_base, mac_uni_base);

		memcpy(mac_address, mac_base, MAC_LEN-1);

		// Uncomment to show the MAC address of the device
		//printf("MAC address device: %d-%d-%d-%d-%d-%d\n", mac_address[0],mac_address[1],mac_address[2],mac_address[3],mac_address[4],mac_address[5]);

		uint8_t topic_id = get_id_from_mac(&mac_address[MAC_LEN]);

		char *num;
		char buffer[100];

		if(asprintf(&num, "%d", topic_id) == -1) {
			perror("asprintf");
		} else {
			strcat(strcpy(buffer, "ESP_CAM/photo/"), num);
			printf("%s\n", buffer);
		}

    	// Function to take a photo
		camera_fb_t * fb = esp_camera_fb_get();
		if (!fb) {
			ESP_LOGE(TAG, "Camera Capture Failed");
		}
		else ESP_LOGI(TAG, "Picture taken! Its size was: %d bytes", fb->len);

		printf("Captured image's parameters:\nSize: %d\nWidth: %d\nHeight: %d\n\n", fb->len, fb->width, fb->height);

		ret = esp_mqtt_client_publish(client, buffer, (char *)fb->buf, fb->len, 1, 0);
		if(ret < 0) {
			ESP_LOGE(TAG, "FAIL sending photo data!!");
		}
		else ESP_LOGI(TAG, "Photo data sent successful");

		//return the frame buffer back to the driver for reuse
		esp_camera_fb_return(fb);

		vTaskDelay(100/portTICK_RATE_MS);

		// End of taking photo

		ret = esp_wifi_sta_get_ap_info(&ap_info);
		if(ret != ESP_OK) {
			printf("ERROR reading wifi info....\n");
		}

		vTaskDelay(CAM_DELAY/portTICK_RATE_MS);

    }
}


#ifdef __cplusplus
}
#endif //__cplusplus
