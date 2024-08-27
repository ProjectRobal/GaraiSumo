#include <stdio.h>
#include <string.h>

#include <esp_wifi.h>
#include <esp_netif_net_stack.h>
#include <esp_netif.h>
#include <nvs_flash.h>

#include <esp_log.h>

#include <esp_littlefs.h>

#include "WiFiManager.h"
#include "HttpService.h"

#include <freertos/event_groups.h>

#include "config.hpp"

static EventGroupHandle_t xWiFiStatus;

static TaskHandle_t xWifiHandle = NULL;

static StackType_t* xWifiStack = NULL;

static StaticTask_t xWifiTask;

static uint8_t ConnectionAttempts = 0;

void wifi_manager_wifi_event_hadler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
   
   if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
        break;
        // wifi scan done
        case WIFI_EVENT_SCAN_DONE:

        break;

        // device connected to wi-fi
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("MAIN","ESP connected to network");
            xEventGroupClearBits(xWiFiStatus,EVENT_BITS_WIFI_DISCONNECTED);
        break;

        // device disconnected to wi-fi
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI("MAIN","ESP disconnected to network, preparing for scanning");
            xEventGroupSetBits(xWiFiStatus,EVENT_BITS_WIFI_DISCONNECTED);
        break;
        }

    }
    else if(event_base == IP_EVENT)
    {

            switch(event_id)
        {
            // wifi scan done, wich means we have succesfully connected to network!
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI("MAIN","ESP got IP from Access Point!");
            break;
        }

    }

}


void wifi_manager_connect_to_ssid(const char* ssid,const char* pass)
{
    ConnectionAttempts=0;
    esp_wifi_disconnect();

    // function that will set ssid and password for station
    wifi_config_t wifi_sta_config={
        .sta={
            .scan_method=WIFI_FAST_SCAN,
            .failure_retry_cnt=5,
            .sort_method=WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi=-127,
            .threshold.authmode=WIFI_AUTH_OPEN,
        }
    };

    strncpy((char*)wifi_sta_config.sta.ssid,ssid,32);
    strncpy((char*)wifi_sta_config.sta.password,pass,64);

    ESP_LOGI("MAIN","Got a new SSID: %s",ssid);
    ESP_LOGI("MAIN","Got a new password: %s",pass);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&wifi_sta_config));
}

void wifi_manager_new_ssid_set()
{
    xEventGroupSetBits(xWiFiStatus,EVENT_BITS_NEW_SSID);
}

void wifi_manager_loop(void* arg)
{
    EventBits_t uxBits;

    while(1)
    {
        ESP_LOGI("MAIN","WiFi manager task!");

        uxBits=xEventGroupGetBits(xWiFiStatus);

        if(((uxBits & EVENT_BITS_WIFI_DISCONNECTED)||(uxBits & EVENT_BITS_NEW_SSID )) && (ConnectionAttempts < WIFI_CONNECTION_ATTEMPTS))
        {

            esp_wifi_connect();

            ConnectionAttempts++;

            xEventGroupClearBits(xWiFiStatus,EVENT_BITS_NEW_SSID);

        }

        // call for each 20 seconds
        vTaskDelay(20000/portTICK_PERIOD_MS);

    }
}

void wifi_manager_init_wifi(void)
{

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_config;

    strcpy((char*)ap_config.ap.ssid,AP_SSID);

    ap_config.ap.ssid_len=strlen(AP_SSID);

    strcpy((char*)ap_config.ap.password,AP_PSK);

    ap_config.ap.authmode=WIFI_AUTH_WPA2_PSK;

    //hide SSID
    ap_config.ap.ssid_hidden=1;

    ap_config.ap.max_connection=1;

    ap_config.ap.channel=1;

    ap_config.ap.pmf_cfg.required=false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap_config));

  
}

void wifi_manager_init()
{
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());

    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_manager_init_wifi();

    xWiFiStatus=xEventGroupCreate();

    xWifiStack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);

    if( ( xWifiHandle = xTaskCreateStaticPinnedToCore(wifi_manager_loop,"WiFiManager",MIN_TASK_STACK_SIZE,NULL,configMAX_PRIORITIES-1,xWifiStack,&xWifiTask,WIFIM_TASK_CORE_ID) ) == NULL )
    {
        ESP_LOGE("MAIN","Cannot start wifi manager task!");
    }

    // esp_netif_create_default_wifi_sta(); 

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
    ESP_EVENT_ANY_ID,&wifi_manager_wifi_event_hadler,NULL,NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
    ESP_EVENT_ANY_ID,&wifi_manager_wifi_event_hadler,NULL,NULL));

    ESP_ERROR_CHECK(esp_wifi_start());

}

