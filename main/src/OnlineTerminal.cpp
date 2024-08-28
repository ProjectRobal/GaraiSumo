#include <ctime>

#include <esp_wifi.h>
#include <esp_event.h>

#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_flash_partitions.h>
#include <esp_partition.h>

#include "OnlineTerminal.hpp"

extern "C"
{
    #include "HttpService.c"
}

using shared::mods;

esp_err_t OnlineTerminal::ws_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_sensors_reading(req);
}

esp_err_t OnlineTerminal::ota_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_ota_handler(req);
}

esp_err_t OnlineTerminal::motor_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_motor_control(req);
}

esp_err_t OnlineTerminal::imu_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_imu_config(req);
}

esp_err_t OnlineTerminal::pid_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_motor_pid(req);
}

esp_err_t OnlineTerminal::calibr_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->calibr_imu(req);
}

esp_err_t OnlineTerminal::mag_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_mag_config(req);
}

esp_err_t OnlineTerminal::posfilter_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_position_filter(req);
}

esp_err_t OnlineTerminal::rototrfilter_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_rotor_filter(req);
}

esp_err_t OnlineTerminal::reset_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->reset_handler(req);
}

OnlineTerminal::OnlineTerminal()
:ws(
    {
        .uri="/readings",
        .method=HTTP_GET,
        .handler=this->ws_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
ws_ota(
    {
        .uri="/ota",
        .method=HTTP_GET,
        .handler=this->ota_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
ws_motors(
    {
        .uri="/motors",
        .method=HTTP_GET,
        .handler=this->motor_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
pid(
    {
        .uri="/pid",
        .method=HTTP_GET,
        .handler=this->pid_wrapper,
        .user_ctx=this
    }
),
pid_post(
    {
        .uri="/pid",
        .method=HTTP_POST,
        .handler=this->pid_wrapper,
        .user_ctx=this
    }
),
imu(
    {
        .uri="/imu",
        .method=HTTP_GET,
        .handler=this->imu_wrapper,
        .user_ctx=this
    }
),
imu_post(
    {
        .uri="/imu",
        .method=HTTP_POST,
        .handler=this->imu_wrapper,
        .user_ctx=this
    }
),
imu_calibr(
    {
        .uri="/calibr",
        .method=HTTP_GET,
        .handler=this->calibr_wrapper,
        .user_ctx=this
    }
),
imu_calibr_post(
    {
        .uri="/calibr",
        .method=HTTP_POST,
        .handler=this->calibr_wrapper,
        .user_ctx=this
    }
),
mag_cfg(
    {
        .uri="/mag",
        .method=HTTP_GET,
        .handler=this->mag_wrapper,
        .user_ctx=this
    }
),
mag_cfg_post(
    {
        .uri="/mag",
        .method=HTTP_POST,
        .handler=this->mag_wrapper,
        .user_ctx=this
    }
),
esp_rst_post(
    {
        .uri="/reset",
        .method=HTTP_POST,
        .handler=this->reset_wrapper,
        .user_ctx=this
    }
),
accelfilter(
    {
        .uri="/posfilter",
        .method=HTTP_GET,
        .handler=this->posfilter_wrapper,
        .user_ctx=this
    }
),
accelfilter_post(
    {
        .uri="/posfilter",
        .method=HTTP_POST,
        .handler=this->posfilter_wrapper,
        .user_ctx=this
    }
),
rotorfilter(
    {
        .uri="/rotorfilter",
        .method=HTTP_GET,
        .handler=this->rototrfilter_wrapper,
        .user_ctx=this
    }
),
rotorfilter_post(
    {
        .uri="/rotorfilter",
        .method=HTTP_POST,
        .handler=this->rototrfilter_wrapper,
        .user_ctx=this
    }
)
{
    this->ota_handle=NULL;
    this->server=NULL;
}

void OnlineTerminal::init()
{

    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &this->connect_handler, this));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &this->disconnect_handler, this));

    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &this->connect_handler, this));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &this->disconnect_handler, this));

    this->start();
}

esp_err_t OnlineTerminal::ws_sensors_reading(httpd_req_t *req)
{
    
        if(req->method == HTTP_GET)
        {
            //handshake
            ESP_LOGI("OnlineTerminal", "Handshake done, the new connection was opened for sensors data");

            return ESP_OK;
        }

        mods.sensors->Lock();

        httpd_ws_frame_t ws_packet={0};
        ws_packet.type=HTTPD_WS_TYPE_TEXT;

        esp_err_t ret =httpd_ws_recv_frame(req,&ws_packet,0);

        if(ret != ESP_OK)
        {
            return ret;
        }

        // send sensors reading

        cJSON* json;

        json=cJSON_CreateObject();

        cJSON_AddNumberToObject(json,"T",static_cast<double>(clock())/CLOCKS_PER_SEC);

        cJSON_AddBoolToObject(json,"IMUOnly",mods.sensors->read().IMUOnlyReading);

        int _distances[18];

        for(uint8_t i=0;i<18;++i)
        {
            _distances[i]=mods.sensors->read().distances[i];
        }

        cJSON* intarray=cJSON_CreateIntArray(_distances,18);

        cJSON_AddItemToObject(json,"distances",intarray);

        cJSON* raw_gyroscope=cJSON_AddObjectToObject(json,"raw_gyroscope");

        Vec3Df raw_gyro=mods.sensors->getGyroscopeBuffer().mean();

        cJSON_AddNumberToObject(raw_gyroscope,"x",raw_gyro.x);
        cJSON_AddNumberToObject(raw_gyroscope,"y",raw_gyro.y);
        cJSON_AddNumberToObject(raw_gyroscope,"z",raw_gyro.z);

        cJSON* raw_accelerometer=cJSON_AddObjectToObject(json,"raw_accelerometer");

        Vec3Df raw_accel=mods.sensors->getAccelerometerBuffer().mean();

        cJSON_AddNumberToObject(raw_accelerometer,"x",raw_accel.x);
        cJSON_AddNumberToObject(raw_accelerometer,"y",raw_accel.y);
        cJSON_AddNumberToObject(raw_accelerometer,"z",raw_accel.z);

        cJSON* raw_magnetrometer=cJSON_AddObjectToObject(json,"raw_magentrometer");

        const Vec3Df& raw_mag=mods.sensors->read().magReading;

        cJSON_AddNumberToObject(raw_magnetrometer,"x",raw_mag.x);
        cJSON_AddNumberToObject(raw_magnetrometer,"y",raw_mag.y);
        cJSON_AddNumberToObject(raw_magnetrometer,"z",raw_mag.z);


        cJSON_AddNumberToObject(json,"left_motor_speed",mods.sensors->read().motorSpeed[0]);

        cJSON_AddNumberToObject(json,"right_motor_speed",mods.sensors->read().motorSpeed[0]);
        
        cJSON_AddNumberToObject(json,"yaw",mods.sensors->read().yaw);

        cJSON_AddBoolToObject(json,"stopped",mods.sensors->read().stoped);

        cJSON* ktir_arr=cJSON_AddArrayToObject(json,"ktirs");

        for(const bool& ktir : mods.sensors->read().floor_sensors)
        {
            cJSON* ktir_bool=cJSON_CreateBool(ktir);
            cJSON_AddItemToArray(ktir_arr,ktir_bool);
        }

        cJSON* coords=cJSON_AddObjectToObject(json,"position");
        
        cJSON* x=cJSON_CreateNumber(mods.sensors->read().position.x);
        cJSON* y=cJSON_CreateNumber(mods.sensors->read().position.y);

        cJSON_AddItemToObject(coords,"x",x);
        cJSON_AddItemToObject(coords,"y",y);

        this->send_ws_json(&ws_packet,req,json);

        cJSON_Delete(json);

        mods.sensors->Unlock();

        return ESP_OK;
}

esp_err_t OnlineTerminal::ws_motor_control(httpd_req_t *req)
{

    if(req->method == HTTP_GET)
    {
        //handshake
        ESP_LOGI("OnlineTerminal", "Handshake done, the new connection was opened for motor control");

        return ESP_OK;
    }

    httpd_ws_frame_t ws_packet={0};
    ws_packet.type=HTTPD_WS_TYPE_TEXT;

    esp_err_t ret=httpd_ws_recv_frame(req,&ws_packet,0);

    if(ret != ESP_OK)
    {
        return ret;
    }

    if(ws_packet.len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }
    
    if(ws_packet.len==0)
    {
        this->send_ws_error(&ws_packet,req,"Body is empty!");

        return ESP_OK;
    }

    this->clear_buf();

    ws_packet.payload=(uint8_t*)this->buffer;

    ret=httpd_ws_recv_frame(req,&ws_packet,ws_packet.len);

    cJSON *json;

    json=cJSON_ParseWithLength(this->buffer,ws_packet.len);

    if( json == NULL)
    {
        const char *msg=cJSON_GetErrorPtr();
        if(msg)
        {
            this->send_ws_error(&ws_packet,req,msg);
        }
        else
        {
            this->send_ws_error(&ws_packet,req,"Cannot parse JSON body");
        }

        return ESP_OK;
    }

    // read motors settings

    cJSON* motorA=cJSON_GetObjectItemCaseSensitive(json,"mA");

    cJSON* motorB=cJSON_GetObjectItemCaseSensitive(json,"mB");

    if((cJSON_IsNumber(motorA)) && (cJSON_IsNumber(motorB)))
    {
        int32_t motorA_pwr=motorA->valueint;
        int32_t motorB_pwr=motorB->valueint;

        ESP_LOGI("OnlineTerminal","Manual override, disabling automatic mode!");

        mods.driver->setAutomaticMode(false);

        mods.driver->set_channelA(motorA_pwr);
        mods.driver->set_channelB(motorB_pwr);

        this->clear_buf();

        sprintf(this->buffer,"A: %ld B: %ld",motorA_pwr,motorB_pwr);

        ws_packet.len=strlen(this->buffer);
        
    }
    else
    {

        this->clear_buf();

        sprintf(this->buffer,"No data");

        ws_packet.len=strlen(this->buffer);   
        
        ws_packet.final=true;

    }

    cJSON_Delete(json);

    httpd_ws_send_frame(req,&ws_packet);

    return ESP_OK;
}

// handle ota update
esp_err_t OnlineTerminal::ws_ota_handler(httpd_req_t *req)
{

    if(req->method == HTTP_GET)
    {
        //handshake
        ESP_LOGI("OnlineTerminal", "Handshake done, the new connection was opened for motor control");

        return ESP_OK;
    }

    httpd_ws_frame_t ws_packet={0};
    ws_packet.type=HTTPD_WS_TYPE_TEXT;

    esp_err_t ret=httpd_ws_recv_frame(req,&ws_packet,0);

    if(ret != ESP_OK)
    {
        return ret;
    }

    if(ws_packet.len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }
    
    if(ws_packet.len==0)
    {
        this->send_ws_error(&ws_packet,req,"Body is empty!");

        return ESP_OK;
    }

    ws_packet.payload=(uint8_t*)this->buffer;

    // We are going to use frames with format:
    // < 1 bytes - id >< 4 bytes - length >< length bytes - data >

    const esp_partition_t* running_partition = esp_ota_get_running_partition();

    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(running_partition);

    ret=httpd_ws_recv_frame(req,&ws_packet,ws_packet.len);

    switch(this->buffer[0])
    {
        case OnlineTerminal::OTAStage::OTA_BEGIN:
        {

            memmove((uint8_t*)&this->ota_sectors_left,this->buffer+1,4);

            ESP_LOGI("MAIN","Starting OTA update");

            this->clear_buf();

            if( update_partition == NULL )
            {
                ESP_LOGE("MAIN","Cannot select partition!");

                sprintf(this->buffer,"ERR");

                ws_packet.len=strlen(this->buffer);

                ws_packet.final = true;
            }
            else
            {

                if( esp_ota_begin(update_partition,OTA_SIZE_UNKNOWN,&ota_handle) != ESP_OK )
                {
                    ESP_LOGE("MAIN","Cannot start OTA!");

                    sprintf(this->buffer,"ERR");

                    ws_packet.len=strlen(this->buffer);

                    ws_packet.final = true;
                }
                else
                {
                    ESP_LOGI("MAIN","OTA started!");
                    ESP_LOGI("OTA","Sectors to write: %u",this->ota_sectors_left);
                    sprintf(this->buffer,"OK");

                    ws_packet.len=strlen(this->buffer);

                    mods.driver->stop();
                }
            }
            
        }
        break;

        case OnlineTerminal::OTAStage::OTA_FLASHING:
        {
            if( this->ota_handle == 0 )
            {
                ESP_LOGE("MAIN","OTA not started!");

                sprintf(this->buffer,"ERR");

                ws_packet.len=strlen(this->buffer);

                break;
            }

            if(ws_packet.len<4096)
            {
                ESP_LOGE("MAIN","Packet to small!");

                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort(this->ota_handle));

                this->ota_handle = 0;

                sprintf(this->buffer,"ERR");

                ws_packet.len=strlen(this->buffer);

                break;

            }

            size_t offset;

            memcpy((uint8_t*)&offset,this->buffer+1,4);

            ESP_LOGI("OTA","Writing sector at offset: %u",offset);

            // image header checking
            if( offset == 0 )
            {
                esp_app_desc_t new_app_info;

                memcpy(&new_app_info, this->buffer+5+sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), sizeof(esp_app_desc_t));
                ESP_LOGI("MAIN", "New firmware version: %s", new_app_info.version);

                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &running_app_info) == ESP_OK) {
                    ESP_LOGI("MAIN", "Running firmware version: %s", running_app_info.version);
                }

                if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
                    ESP_LOGE("MAIN","The incoming version is the same as current version, abort");

                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort(this->ota_handle));

                    this->ota_handle = 0;

                    sprintf(this->buffer,"ERR");

                    ws_packet.len=strlen(this->buffer);

                    break;
                }
            }

            if( esp_ota_write_with_offset(this->ota_handle,this->buffer+5,4096,offset*4096) == ESP_OK )
            {
                this->clear_buf();


                if(this->ota_sectors_left==0)
                {
                    ESP_LOGI("MAIN","Image has been written!");

                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_end(this->ota_handle));

                    this->ota_handle = 0;

                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_set_boot_partition(update_partition));

                    sprintf(this->buffer,"END");

                    ws_packet.len=strlen(this->buffer);
                }
                else
                {
                    sprintf(this->buffer,"OK");

                    ws_packet.len=strlen(this->buffer);
                }

                this->ota_sectors_left--;

            }
            else
            {
                ESP_LOGE("MAIN","Cannot write sector %u",offset*4096);

                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort(this->ota_handle));

                this->ota_handle = 0;

                sprintf(this->buffer,"ERR");

                ws_packet.len=strlen(this->buffer);

                ws_packet.final = true;
            }
        }
        break;

        default:

            this->clear_buf();

            sprintf(this->buffer,"Wrong options");

            ws_packet.len=strlen(this->buffer);

        break;
    }

    httpd_ws_send_frame(req,&ws_packet);

    return ESP_OK;
}

esp_err_t OnlineTerminal::set_imu_config(httpd_req_t *req)
{
    if(req->content_len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }

    switch(req->method)
    {
        case HTTP_GET:
        {
            this->clear_buf();

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.sensors->dump_cfg()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
            
        }
        break;
        case HTTP_POST:
        {

            this->clear_buf();

            httpd_req_recv(req,this->buffer,req->content_len);

            config::SensorConfig config;

            if(!config::ConfigLoader::fromBuffer(this->buffer,config))
            {
                ESP_LOGE("OnlineTerminal","Failed to parse IMU offsets from body");
                httpd_resp_send_500(req);   
                return ESP_OK;
            }

            mods.sensors->update_cfg(config);

            config::ConfigLoader::save(config);

            httpd_resp_sendstr(req,"OK");

        }
        break;
    }
    return ESP_OK;
}

esp_err_t OnlineTerminal::reset_handler(httpd_req_t *req)
{
    if(req->content_len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }

    if( req->method != HTTP_POST )
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI("MAIN","Reseting device...");

    esp_restart();

    return ESP_OK;
}

esp_err_t OnlineTerminal::set_mag_config(httpd_req_t *req)
{
    if(req->content_len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }

    switch(req->method)
    {
        case HTTP_GET:
        {
            this->clear_buf();

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.sensors->dump_mag_cfg()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
            
        }
        break;
        case HTTP_POST:
        {

            this->clear_buf();

            httpd_req_recv(req,this->buffer,req->content_len);

            config::MagConfig config;

            if(!config::ConfigLoader::fromBuffer(this->buffer,config))
            {
                ESP_LOGE("OnlineTerminal","Failed to parse MAG offsets from body");
                httpd_resp_send_500(req);   
                return ESP_OK;
            }

            mods.sensors->update_mag_cfg(config);

            config::ConfigLoader::save(config);

            httpd_resp_sendstr(req,"OK");

        }
        break;
    }
    return ESP_OK;
}

esp_err_t OnlineTerminal::set_motor_pid(httpd_req_t *req)
{
    if(req->content_len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }

    switch(req->method)
    {
        case HTTP_GET:
        {

            this->clear_buf();

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.driver->MotorConfig()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
            
        }
        break;
        case HTTP_POST:
        {
            
            httpd_req_recv(req,this->buffer,req->content_len);

            config::MotorCFG cfg;

            if(!config::ConfigLoader::fromBuffer(this->buffer,cfg))
            {
                ESP_LOGE("OnlineTerminal","Cannot get PID value from body");
                httpd_resp_send_500(req);   
                return ESP_OK;  
            }

            mods.driver->setMotorConfig(cfg);

            config::ConfigLoader::save(cfg);

            httpd_resp_sendstr(req,"OK");

        }
        break;
    }
    return ESP_OK;
}

esp_err_t OnlineTerminal::calibr_imu(httpd_req_t *req)
{

    switch(req->method)
    {
        case HTTP_POST:
        {

            size_t N = 10;

            httpd_req_recv(req,this->buffer,req->content_len);

            cJSON* json = cJSON_Parse(this->buffer);

            if( json != NULL )
            {
                cJSON *steps=cJSON_GetObjectItem(json,"steps");

                if( steps != NULL )
                {
                    if(cJSON_IsNumber(steps))
                    {
                        N = steps->valueint;
                    }
                }
            }

            mods.driver->stop();
            mods.sensors->DoIMUCalibration(N);

            httpd_resp_sendstr(req,"OK");

        }
        break;

        case HTTP_GET:

            cJSON*json;

            json=cJSON_CreateObject();

            cJSON_AddBoolToObject(json,"done",mods.sensors->IMUCalibrationDone());
            cJSON_AddNumberToObject(json,"steps_left",mods.sensors->StepsLeftUntilCalibration());

            this->clear_buf();

            if(!cJSON_PrintPreallocated(json,this->buffer,WS_MAX_PAYLOAD_SIZE,true))
            {
                httpd_resp_send_500(req);
                return ESP_OK;
            }

            httpd_resp_sendstr(req,this->buffer);

            cJSON_Delete(json);

        break;
    }

    return ESP_OK;

}

esp_err_t OnlineTerminal::set_position_filter(httpd_req_t *req)
{

    if(req->content_len>WS_MAX_PAYLOAD_SIZE)
    {   
        ESP_LOGE("OnlineTerminal","Incoming body is too big!");
        return ESP_FAIL;
    }

    size_t query_len=httpd_req_get_url_query_len(req)+1;

    if(query_len==1)
    {
        ESP_LOGE("OnlineTerminal","No query string passed!");
        return ESP_FAIL;
    }

    char query[query_len];

    httpd_req_get_url_query_str(req,query,query_len);

    char param[255];

    if( httpd_query_key_value(query,"axis",param,255) != ESP_OK)
    {
        ESP_LOGE("OnlineTerminal","No query string passed!");
        return ESP_FAIL;
    }

    switch(req->method)
    {
        case HTTP_GET:
        {

            this->clear_buf();

            switch(param[0])
            {

                case 'y':

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.sensors->getPositionFilterYCFG()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
                break;

                case 'x':

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.sensors->getPositionFilterXCFG()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
                break;

            }
            
        }
        break;
        case HTTP_POST:
        {
            switch(param[0])
            {

                case 'x':

            httpd_req_recv(req,this->buffer,req->content_len);

            config::PositionFilterCFG cfg;

            if(!config::ConfigLoader::fromBuffer(this->buffer,cfg))
            {
                ESP_LOGE("OnlineTerminal","Cannot get position filter config from body");
                httpd_resp_send_500(req);   
                return ESP_OK;  
            }

            mods.sensors->updatePositionFilterXCFG(cfg);

            config::ConfigLoader::save(cfg,POS_Y_CFG_FILE);

            httpd_resp_sendstr(req,"OK");

                break;

                case 'y':

            httpd_req_recv(req,this->buffer,req->content_len);

            if(!config::ConfigLoader::fromBuffer(this->buffer,cfg))
            {
                ESP_LOGE("OnlineTerminal","Cannot get position filter config from body");
                httpd_resp_send_500(req);   
                return ESP_OK;  
            }

            mods.sensors->updatePositionFilterYCFG(cfg);

            config::ConfigLoader::save(cfg,POS_Y_CFG_FILE);

            httpd_resp_sendstr(req,"OK");

                break;
            }

        }
        break;
    }
    return ESP_OK;

}

esp_err_t OnlineTerminal::set_rotor_filter(httpd_req_t *req)
{


    switch(req->method)
    {
        case HTTP_GET:
        {

            this->clear_buf();

            if(config::ConfigLoader::toBuffer(this->buffer,WS_MAX_PAYLOAD_SIZE,mods.sensors->getRoationFilterCFG()))
            {
                httpd_resp_set_type(req,"application/json");

                httpd_resp_send(req,this->buffer,strlen(this->buffer));
            }
            else
            {
                httpd_resp_send_500(req);
            }
            
        }
        break;
        case HTTP_POST:
        {

                case 'x':

            httpd_req_recv(req,this->buffer,req->content_len);

            config::RotationFilterCFG cfg;

            if(!config::ConfigLoader::fromBuffer(this->buffer,cfg))
            {
                ESP_LOGE("OnlineTerminal","Cannot get rotation config data from body");
                httpd_resp_send_500(req);   
                return ESP_OK;  
            }

            mods.sensors->updateRoationFilterCFG(cfg);

            config::ConfigLoader::save(cfg);

            httpd_resp_sendstr(req,"OK");
        }
        break;
    }
    return ESP_OK;

}


void OnlineTerminal::start()
{

    if( this->server != NULL )
    {
        return;
    }

    // ESP_LOGI("MAIN","Starting HTTP server");

    httpd_config_t config= HTTPD_DEFAULT_CONFIG();

    config.max_uri_handlers = 19;


    if(httpd_start(&this->server,&config) == ESP_OK)
    {

        ESP_LOGI("OnlineTerminal","Starting online terminal");

        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->ws)); 
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->ws_ota));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->ws_motors));   
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->pid));   
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->imu));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->pid_post));   
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->imu_post)); 
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->imu_calibr));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->imu_calibr_post));   
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->mag_cfg));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->mag_cfg_post)); 
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->esp_rst_post)); 

        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&set_ssid_cfg));  
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&home_cfg));       
        
        // this crash online terminal:
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->accelfilter));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->accelfilter_post)); 
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->rotorfilter));
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->rotorfilter_post));
    }
    else
    {
        ESP_LOGE("OnlineTerminal","Error starting server");
    }


}

void OnlineTerminal::stop()
{
    if(this->server)
    {
        httpd_stop(this->server);
        this->server=NULL;
    }

}