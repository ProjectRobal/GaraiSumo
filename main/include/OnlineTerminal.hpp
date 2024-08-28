/*
    Class that handles websocket server for remote debuging and testing.
    It would handle websocket for reading sensor data,
    websocket for motor control,
    http endpoint for setting IMU offsets
    http endpoint for setting motors PID

*/

#pragma once

#define WS_MAX_PAYLOAD_SIZE 8192

#include <ctime>
#include <cstdlib>
#include <cJSON.h>
#include <esp_http_server.h>
#include <esp_log.h>

#include <esp_ota_ops.h>

#include "ConfigLoader.hpp"

#include "shared.hpp"



class OnlineTerminal
{

    httpd_handle_t server;

    const httpd_uri_t ws;
    const httpd_uri_t ws_ota;
    const httpd_uri_t ws_motors;
    const httpd_uri_t pid;
    const httpd_uri_t pid_post;
    const httpd_uri_t imu;
    const httpd_uri_t imu_post;
    const httpd_uri_t imu_calibr;
    const httpd_uri_t imu_calibr_post;
    const httpd_uri_t mag_cfg;
    const httpd_uri_t mag_cfg_post;

    const httpd_uri_t esp_rst_post;

    const httpd_uri_t accelfilter;
    const httpd_uri_t accelfilter_post;

    const httpd_uri_t rotorfilter;
    const httpd_uri_t rotorfilter_post;

    char buffer[WS_MAX_PAYLOAD_SIZE];

    clock_t ota_timeout_counter;

    size_t ota_sectors_left;

    esp_ota_handle_t ota_handle;

    enum OTAStage
    {
        OTA_BEGIN=0,
        OTA_FLASHING=1
    };

    void clear_buf()
    {
        memset(buffer,0,WS_MAX_PAYLOAD_SIZE*sizeof(char));
    }


    void send_ws_error(httpd_ws_frame_t* ws_packet,httpd_req_t *req,const char* msg)
    {
        ws_packet->type = HTTPD_WS_TYPE_TEXT;
        ws_packet->payload=(uint8_t*)msg;
        ws_packet->len=strlen(msg);

        ws_packet->final=true;

        httpd_ws_send_frame(req,ws_packet);

    }

    void send_ws_json(httpd_ws_frame_t* ws_packet,httpd_req_t *req,cJSON *json)
    {
        this->clear_buf();
        ws_packet->type = HTTPD_WS_TYPE_TEXT;
        ws_packet->payload=(uint8_t*)this->buffer;

        cJSON_PrintPreallocated(json,this->buffer,4096,true);

        ws_packet->len=strlen(this->buffer);

        httpd_ws_send_frame(req,ws_packet);
    }
    esp_err_t ws_sensors_reading(httpd_req_t *req);

    esp_err_t ws_motor_control(httpd_req_t *req);

    esp_err_t set_imu_config(httpd_req_t *req);

    esp_err_t set_motor_pid(httpd_req_t *req);

    esp_err_t calibr_imu(httpd_req_t *req);

    esp_err_t set_position_filter(httpd_req_t *req);

    esp_err_t set_rotor_filter(httpd_req_t *req);

    esp_err_t set_mag_config(httpd_req_t *req);

    esp_err_t ws_ota_handler(httpd_req_t *req);

    esp_err_t reset_handler(httpd_req_t *req);

    void start();

    void stop();

    static esp_err_t ws_wrapper(httpd_req_t *req);

    static esp_err_t ota_wrapper(httpd_req_t *req);

    static esp_err_t motor_wrapper(httpd_req_t *req);

    static esp_err_t imu_wrapper(httpd_req_t *req);

    static esp_err_t pid_wrapper(httpd_req_t *req);

    static esp_err_t calibr_wrapper(httpd_req_t *req);

    static esp_err_t mag_wrapper(httpd_req_t *req);

    static esp_err_t posfilter_wrapper(httpd_req_t *req);

    static esp_err_t rototrfilter_wrapper(httpd_req_t *req);

    static esp_err_t reset_wrapper(httpd_req_t *req);

    static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
    {
        ESP_LOGI("OnlineTerminal","Client connected! Starting Online terminal!");
        ((OnlineTerminal*)arg)->start();
    }

    static void disconnect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
    {
        ESP_LOGI("OnlineTerminal","Client disconnected! Stoping Online terminal!");
        ((OnlineTerminal*)arg)->stop();
    }

    public:

    OnlineTerminal();

    void init();

};

