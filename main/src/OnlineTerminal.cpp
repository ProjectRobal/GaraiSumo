#include <ctime>

#include <esp_wifi.h>
#include <esp_event.h>

#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_flash_partitions.h>
#include <esp_partition.h>

#include "OnlineTerminal.hpp"

#include "starter.hpp"

extern "C"
{
    #include "HttpService.c"
}

using shared::mods;

esp_err_t ws_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_sensors_reading(req);
}

esp_err_t ota_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_ota_handler(req);
}

esp_err_t motor_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->ws_motor_control(req);
}

esp_err_t imu_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_imu_config(req);
}

esp_err_t pid_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_motor_pid(req);
}

esp_err_t calibr_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->calibr_imu(req);
}

esp_err_t mag_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_mag_config(req);
}

esp_err_t posfilter_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_position_filter(req);
}

esp_err_t rototrfilter_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->set_rotor_filter(req);
}

esp_err_t reset_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->reset_handler(req);
}

esp_err_t starter_wrapper(httpd_req_t *req)
{
    return ((OnlineTerminal*)req->user_ctx)->starter_handler(req);
}


OnlineTerminal::OnlineTerminal()
:ws(
    {
        .uri="/readings",
        .method=HTTP_GET,
        .handler=ws_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
ws_ota(
    {
        .uri="/ota",
        .method=HTTP_GET,
        .handler=ota_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
ws_motors(
    {
        .uri="/motors",
        .method=HTTP_GET,
        .handler=motor_wrapper,
        .user_ctx=this,
        .is_websocket=true
    }
),
pid(
    {
        .uri="/pid",
        .method=HTTP_GET,
        .handler=pid_wrapper,
        .user_ctx=this
    }
),
pid_post(
    {
        .uri="/pid",
        .method=HTTP_POST,
        .handler=pid_wrapper,
        .user_ctx=this
    }
),
imu(
    {
        .uri="/imu",
        .method=HTTP_GET,
        .handler=imu_wrapper,
        .user_ctx=this
    }
),
imu_post(
    {
        .uri="/imu",
        .method=HTTP_POST,
        .handler=imu_wrapper,
        .user_ctx=this
    }
),
imu_calibr(
    {
        .uri="/calibr",
        .method=HTTP_GET,
        .handler=calibr_wrapper,
        .user_ctx=this
    }
),
imu_calibr_post(
    {
        .uri="/calibr",
        .method=HTTP_POST,
        .handler=calibr_wrapper,
        .user_ctx=this
    }
),
mag_cfg(
    {
        .uri="/mag",
        .method=HTTP_GET,
        .handler=mag_wrapper,
        .user_ctx=this
    }
),
mag_cfg_post(
    {
        .uri="/mag",
        .method=HTTP_POST,
        .handler=mag_wrapper,
        .user_ctx=this
    }
),
esp_rst_post(
    {
        .uri="/reset",
        .method=HTTP_POST,
        .handler=reset_wrapper,
        .user_ctx=this
    }
),
esp_starter_override(
    {
        .uri="/start",
        .method=HTTP_POST,
        .handler=starter_wrapper,
        .user_ctx=this
    }
),
accelfilter(
    {
        .uri="/posfilter",
        .method=HTTP_GET,
        .handler=posfilter_wrapper,
        .user_ctx=this
    }
),
accelfilter_post(
    {
        .uri="/posfilter",
        .method=HTTP_POST,
        .handler=posfilter_wrapper,
        .user_ctx=this
    }
),
rotorfilter(
    {
        .uri="/rotorfilter",
        .method=HTTP_GET,
        .handler=rototrfilter_wrapper,
        .user_ctx=this
    }
),
rotorfilter_post(
    {
        .uri="/rotorfilter",
        .method=HTTP_POST,
        .handler=rototrfilter_wrapper,
        .user_ctx=this
    }
)
{
    this->ota_handle=NULL;
    this->server=NULL;
}

void OnlineTerminal::init()
{
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

        ESP_LOGI("MAIN","Sending sensor's readings!");

        esp_err_t ret =httpd_ws_recv_frame(req,&ws_packet,0);

        if(ret != ESP_OK)
        {
            mods.sensors->Unlock();
            return ret;
        }

        // send sensors reading

        cJSON* json;

        json=cJSON_CreateObject();

        cJSON_AddNumberToObject(json,"T",static_cast<double>(clock())/CLOCKS_PER_SEC);

        cJSON_AddNumberToObject(json,"battery",mods.sensors->read().battery_voltage);

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

        cJSON* shock = cJSON_AddObjectToObject(json,"shock");

        cJSON_AddNumberToObject(shock,"angle",mods.sensors->read().shock_angel);

        cJSON_AddNumberToObject(shock,"value",mods.sensors->read().shock_value);


        cJSON_AddNumberToObject(json,"left_motor_speed",mods.sensors->read().motorSpeed[0]);

        cJSON_AddNumberToObject(json,"right_motor_speed",mods.sensors->read().motorSpeed[1]);
        
        cJSON_AddNumberToObject(json,"yaw",mods.sensors->read().yaw);

        cJSON_AddBoolToObject(json,"stopped",mods.sensors->read().stoped);

        cJSON* ktir_arr=cJSON_AddArrayToObject(json,"ktirs");

        for(bool ktir : mods.sensors->read().floor_sensors)
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

    bool open = true;

    // bool if set true it will set raw engine speed and if not it will set motor speed
    cJSON* raw_or_speed=cJSON_GetObjectItemCaseSensitive(json,"open");

    if(cJSON_IsBool(raw_or_speed))
    {
        open = cJSON_IsTrue(raw_or_speed);
    }

    if((cJSON_IsNumber(motorA)) && (cJSON_IsNumber(motorB)))
    {

        ESP_LOGI("OnlineTerminal","Manual override, disabling automatic mode!");

        mods.driver->setAutomaticMode(!open);

        if( open )
        {
            int32_t motorA_pwr=motorA->valueint;
            int32_t motorB_pwr=motorB->valueint;
            
            mods.sensors->Lock();
            // hello
            if(( mods.sensors->read().stoped )||( !starter_state() ))
            {
                mods.driver->stop();                
            }
            else
            {

                mods.driver->set_channelA(motorA_pwr);
                mods.driver->set_channelB(motorB_pwr);

            }
            mods.sensors->Unlock();

            this->clear_buf();

            sprintf(this->buffer,"A: %ld B: %ld",motorA_pwr,motorB_pwr);
        }
        else
        {
            float speed = motorA->valuedouble;
            float angel = motorB->valuedouble;
            mods.driver->setTargetSpeed(speed);
            mods.driver->setTargetAngel(angel);

            this->clear_buf();

            sprintf(this->buffer,"Speed: %f Yaw: %f",speed,angel);
        }

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

                // Cannot select partition
                sprintf(this->buffer,"PRT");

                ws_packet.len=strlen(this->buffer);

                ws_packet.final = true;
            }
            else
            {

                if( esp_ota_begin(update_partition,OTA_SIZE_UNKNOWN,&ota_handle) != ESP_OK )
                {
                    ESP_LOGE("MAIN","Cannot start OTA!");

                    // Cannot start OTA
                    sprintf(this->buffer,"OTA");

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

                    ota_timeout_counter = clock();
                }
            }
            
        }
        break;

        case OnlineTerminal::OTAStage::OTA_FLASHING:
        {
            if( this->ota_handle == 0 )
            {
                ESP_LOGE("MAIN","OTA not started!");
                
                this->clear_buf();
                // OTA not started
                sprintf(this->buffer,"QTA");

                ws_packet.len=strlen(this->buffer);

                break;
            }

            if( ( ( clock() - ota_timeout_counter )/CLOCKS_PER_SEC ) >= OTA_WS_TIMEOUT )
            {
                ESP_LOGE("OTA","Websocket timeout!");

                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort(this->ota_handle));

                this->clear_buf();

                this->ota_handle = 0;

                this->ota_timeout_counter=0;
            }

            if(ws_packet.len<5)
            {
                ESP_LOGE("MAIN","Packet to small!");

                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_abort(this->ota_handle));

                this->clear_buf();

                this->ota_handle = 0;
                // Packet to small
                sprintf(this->buffer,"TSL");

                ws_packet.len=strlen(this->buffer);

                break;

            }

            size_t offset;

            memcpy((uint8_t*)&offset,this->buffer+1,4);

            ota_timeout_counter = clock();

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

                    this->clear_buf();

                    this->ota_handle = 0;
                    // Incoming verision is the same as on the microcontroller
                    sprintf(this->buffer,"OUT");

                    ws_packet.len=strlen(this->buffer);

                    break;
                }
            }

            if( esp_ota_write_with_offset(this->ota_handle,this->buffer+5,ws_packet.len-5,offset*4096) == ESP_OK )
            {
                this->clear_buf();

                if(this->ota_sectors_left==0)
                {
                    ESP_LOGI("MAIN","Image has been written!");

                    esp_err_t err = esp_ota_end(this->ota_handle);

                    if( err != ESP_OK )
                    {
                        ESP_LOGE("OTA","Cannot end OTA %s",esp_err_to_name(err));
                    }

                    this->ota_handle = 0;

                    err = esp_ota_set_boot_partition(update_partition);

                    if( err!= ESP_OK )
                    {
                        ESP_LOGE("OTA","Cannot select new boot partition %s",esp_err_to_name(err));
                    }

                    if( err == ESP_OK)
                    {
                        ESP_LOGI("OTA","Image flashed successfully");
                        sprintf(this->buffer,"END");

                        ws_packet.len=strlen(this->buffer);
                    }
                    else
                    {
                        sprintf(this->buffer,"FUK");

                        ws_packet.len=strlen(this->buffer);
                    }
                }
                else
                {
                    this->ota_sectors_left--;

                    sprintf(this->buffer,"OK");

                    ws_packet.len=strlen(this->buffer);
                }

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
            // Wrong option
            sprintf(this->buffer,"OPT");

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

    httpd_resp_sendstr(req,"RST");

    esp_restart();

    return ESP_OK;
}

esp_err_t OnlineTerminal::starter_handler(httpd_req_t *req)
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

    httpd_req_recv(req,this->buffer,req->content_len);

    cJSON* json = cJSON_Parse(this->buffer);

    if( json != NULL )
    {
        cJSON *start=cJSON_GetObjectItem(json,"start");

        if( start != NULL )
        {
            if(cJSON_IsBool(start))
            {
                starter_override(cJSON_IsTrue(start));   
            }
            else
            {
                httpd_resp_send_500(req);
                cJSON_Delete(json);
                return ESP_FAIL;
            }
        }
        else
        {
            httpd_resp_send_500(req);
            cJSON_Delete(json);
            return ESP_FAIL;
        }
    }
    else
    {
        httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    cJSON_Delete(json);

    if(starter_state())
    {
        httpd_resp_sendstr(req,"Unlocked");
    }
    else
    {
        httpd_resp_sendstr(req,"Locked");
    }

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

            config::MagConfig config = mods.sensors->dump_mag_cfg();

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

            starter_override(false);
            mods.sensors->DoIMUCalibration(N);

            if(json!=NULL)
            {
                cJSON_Delete(json);
            }

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

    config.max_uri_handlers = 20;
    config.stack_size = 4096*2;


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
        ESP_ERROR_CHECK(httpd_register_uri_handler(this->server,&this->esp_starter_override)); 

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