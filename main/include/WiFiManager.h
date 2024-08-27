#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>


// because why not

#define WIFIM_TASK_CORE_ID 1

#define WIFI_TASK_STACK_SIZE 32768

#define WIFI_CONNECTION_ATTEMPTS 3

// events bits definition

#define EVENT_BITS_WIFI_DISCONNECTED (1<<0)

#define EVENT_BITS_NEW_SSID (1<<1)


// --- WIFI Event handler

void wifi_manager_wifi_event_hadler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);

// --- General WiFi Functions

void wifi_manager_connect_to_ssid(const char* ssid,const char* pass);

void wifi_manager_init_wifi(void);

void wifi_manager_new_ssid_set();

// function that initlialize manager
void wifi_manager_init();
