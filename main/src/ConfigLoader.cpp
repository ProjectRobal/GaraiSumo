#include "ConfigLoader.hpp"

namespace config
{

esp_vfs_littlefs_conf_t ConfigLoader::fs={0};

void ConfigLoader::print_filesystem_info()
{

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGD("Config","This is %s chip with %d CPU cores, WiFi%s%s, ",
               CONFIG_IDF_TARGET,
               chip_info.cores,
               (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
               (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGD("Config","silicon revision %d, ", chip_info.revision);

    uint32_t size_flash_chip = 0;
    esp_flash_get_size(NULL, &size_flash_chip);
    ESP_LOGD("Config","%uMB %s flash\n", (unsigned int)size_flash_chip >> 20,
               (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGD("Config","Free heap: %u\n", (unsigned int) esp_get_free_heap_size());

}

void ConfigLoader::init_filesystem()
{
    ConfigLoader::fs = {
            .base_path = "/littlefs",
            .partition_label = "littlefs",
            .format_if_mount_failed = true,
            .dont_mount = false
        };

    esp_err_t ret= esp_vfs_littlefs_register(&ConfigLoader::fs);

    switch(ret)
    {
        case ESP_OK:
        {

            ESP_LOGI("Config","LittleFS initialized!");

            char dirPath[255]={0};

            struct stat st = {0};

            sprintf(dirPath,"/littlefs/%s",CONFIG_FOLDER);

            if(stat(dirPath,&st)==-1)
            {

            mkdir(dirPath,0777);

            }
        }

        break;

        case ESP_ERR_NO_MEM:

            ESP_LOGE("Config","Not enough memorty for LittleFS");

        break;

        case ESP_ERR_NOT_FOUND:
            
            ESP_LOGE("Config","Couldn't find partition with correct name");

        break;

        case ESP_FAIL:

            ESP_LOGE("Config","Couldn't mount littlefs partition");
            //esp_littlefs_format("littlefs");
            //esp_vfs_littlefs_register(&this->fs);

        break;
    }

    PartitionInfo info=ConfigLoader::info();

    ESP_LOGI("Config", "Partition size: total: %d kB, used: %d kB", info.total/1000, info.used/1000);

}


PartitionInfo ConfigLoader::info()
{
    PartitionInfo info;

    esp_littlefs_info(ConfigLoader::fs.partition_label,&info.total,&info.used);

    return info;
}

void ConfigLoader::init()
{
    init_filesystem();
    print_filesystem_info();
}

bool ConfigLoader::fromBuffer(const char* buffer,SensorConfig& cfg)
{

    cJSON* json=cJSON_Parse(buffer);

    if(!json)
    {
        ESP_LOGE("Config","Cannot parse sensor config json: %s",cJSON_GetErrorPtr());
        return false;
    }

    if(!ConfigLoader::json_to_vector3d<int32_t>(cJSON_GetObjectItemCaseSensitive(json,"accel"),cfg.calibr.accelerometer))
    {
        cJSON_Delete(json);
        return false;
    }   

    if(!ConfigLoader::json_to_vector3d<int32_t>(cJSON_GetObjectItemCaseSensitive(json,"gyro"),cfg.calibr.gyroscope))
    {
        cJSON_Delete(json);
        return false;
    }

    cJSON *beta=cJSON_GetObjectItemCaseSensitive(json,"beta");

    if(!cJSON_IsNumber(beta))
    {
        cJSON_Delete(json);
        return false;
    }
    else
    {
        cfg.beta=beta->valuedouble;
    }

    cJSON_Delete(json);

    return true;

}

bool ConfigLoader::toBuffer(char* buffer,size_t size,const SensorConfig& cfg)
{

    cJSON *json= cJSON_CreateObject();

    cJSON* gyroOffset=ConfigLoader::vector3d_to_json<int32_t>(cfg.calibr.gyroscope);

    cJSON* accelOffset=ConfigLoader::vector3d_to_json<int32_t>(cfg.calibr.accelerometer); 

    cJSON_AddItemToObject(json,"gyro",gyroOffset);
    cJSON_AddItemToObject(json,"accel",accelOffset);

    cJSON_AddNumberToObject(json,"beta",cfg.beta);

    if(!cJSON_PrintPreallocated(json,buffer,size,true))
    {
        cJSON_Delete(json);
        return false;
    } 

    cJSON_Delete(json);
    return true;

}


bool ConfigLoader::fromBuffer(const char* buffer,MagConfig& cfg)
{

    cJSON* json=cJSON_Parse(buffer);

    if(!json)
    {
        ESP_LOGE("Config","Cannot parse sensor config json: %s",cJSON_GetErrorPtr());
        return false;
    }

    if(!ConfigLoader::json_to_vector3d<float>(cJSON_GetObjectItemCaseSensitive(json,"offsets"),cfg.offsets))
    {
        cJSON_Delete(json);
        return false;
    }   


    cJSON *matrix=cJSON_GetObjectItemCaseSensitive(json,"matrix");

    if(!cJSON_IsArray(matrix))
    {
        cJSON_Delete(json);
        return false;
    }
    else
    {
        for(uint8_t i=0;i<9;++i)
        {
            cJSON* number = cJSON_GetArrayItem(matrix,i);

            if(!cJSON_IsNumber(number))
            {
                cJSON_Delete(json);
                return false;
            }

            cfg.c_matrix[i] = number->valuedouble;
        }   
    }

    cJSON_Delete(json);

    return true;

}

bool ConfigLoader::toBuffer(char* buffer,size_t size,const MagConfig& cfg)
{

    cJSON *json= cJSON_CreateObject();

    cJSON* offsets=ConfigLoader::vector3d_to_json<float>(cfg.offsets);

    cJSON_AddItemToObject(json,"offsets",offsets);

    cJSON* c_matrix = cJSON_CreateDoubleArray(cfg.c_matrix,9);

    cJSON_AddItemToObject(json,"matrix",c_matrix);

    if(!cJSON_PrintPreallocated(json,buffer,size,true))
    {
        cJSON_Delete(json);
        return false;
    } 

    cJSON_Delete(json);
    return true;

}

bool ConfigLoader::fromBuffer(const char* buffer,MotorCFG& cfg)
{

    cJSON* json=cJSON_Parse(buffer);

    if(!json)
    {
        ESP_LOGE("Config","Cannot parse sensor config json: %s",cJSON_GetErrorPtr());
        return false;
    }

    if(!ConfigLoader::json_to_pid(cJSON_GetObjectItemCaseSensitive(json,"motor1"),cfg.motor1))
    {
        cJSON_Delete(json);
        return false;
    }   

    if(!ConfigLoader::json_to_pid(cJSON_GetObjectItemCaseSensitive(json,"motor2"),cfg.motor2))
    {
        cJSON_Delete(json);
        return false;
    } 

    if(!ConfigLoader::json_to_pid(cJSON_GetObjectItemCaseSensitive(json,"motor3"),cfg.motor3))
    {
        cJSON_Delete(json);
        return false;
    }

    cJSON_Delete(json);

    return true;

}

bool ConfigLoader::toBuffer(char* buffer,size_t size,const MotorCFG& cfg)
{
    cJSON *json= cJSON_CreateObject();

    cJSON* pid1=ConfigLoader::pid_to_json(cfg.motor1);    

    cJSON* pid2=ConfigLoader::pid_to_json(cfg.motor2);

    cJSON* pid3=ConfigLoader::pid_to_json(cfg.motor3);

    cJSON_AddItemToObject(json,"motor1",pid1);
    cJSON_AddItemToObject(json,"motor2",pid2);
    cJSON_AddItemToObject(json,"motor2",pid3);

    if(!cJSON_PrintPreallocated(json,buffer,512,true))
    {
        cJSON_Delete(json);
        return false;
    } 

    return true;
}

bool ConfigLoader::fromBuffer(const char* buffer,PositionFilterCFG& cfg)
{
    cJSON* json=cJSON_Parse(buffer);

    if(!json)
    {
        ESP_LOGE("Config","Cannot parse sensor config json: %s",cJSON_GetErrorPtr());
        return false;
    }
    
    cJSON *obj=cJSON_GetObjectItemCaseSensitive(json,"Ex1");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Ex1=obj->valuedouble;


    obj=cJSON_GetObjectItemCaseSensitive(json,"Ex2");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Ex2=obj->valuedouble;
    

    obj=cJSON_GetObjectItemCaseSensitive(json,"R");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.R=obj->valuedouble;

    cJSON_Delete(json);

    return true;
}

bool ConfigLoader::toBuffer(char* buffer,size_t size,const PositionFilterCFG& cfg)
{
    cJSON *json= cJSON_CreateObject();

    cJSON_AddNumberToObject(json,"Ex1",cfg.Ex1);
    cJSON_AddNumberToObject(json,"Ex2",cfg.Ex2);
    cJSON_AddNumberToObject(json,"R",cfg.R);

    if(!cJSON_PrintPreallocated(json,buffer,512,true))
    {
        cJSON_Delete(json);
        return false;
    }

    cJSON_Delete(json);

    return true;
}

bool ConfigLoader::fromBuffer(const char* buffer,RotationFilterCFG& cfg)
{
    cJSON* json=cJSON_Parse(buffer);

    if(!json)
    {
        ESP_LOGE("Config","Cannot parse rotation filter config json: %s",cJSON_GetErrorPtr());
        return false;
    }

    cJSON* obj=cJSON_GetObjectItemCaseSensitive(json,"Ex1");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Ex1=obj->valuedouble;

    obj=cJSON_GetObjectItemCaseSensitive(json,"Ex2");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Ex2=obj->valuedouble;

    obj=cJSON_GetObjectItemCaseSensitive(json,"R");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.R=obj->valuedouble;

    obj=cJSON_GetObjectItemCaseSensitive(json,"Q0_1");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Q0_1=obj->valuedouble;

    obj=cJSON_GetObjectItemCaseSensitive(json,"Q0_2");

    if(!cJSON_IsNumber(obj))
    {
        cJSON_Delete(json);
        return false;
    }

    cfg.Q0_2=obj->valuedouble;

    cJSON_Delete(json);

    return true;

}

bool ConfigLoader::toBuffer(char* buffer,size_t size,const RotationFilterCFG& cfg)
{

    cJSON* json=cJSON_CreateObject();

    cJSON_AddNumberToObject(json,"Ex1",cfg.Ex1);
    cJSON_AddNumberToObject(json,"Ex2",cfg.Ex2);

    cJSON_AddNumberToObject(json,"R",cfg.R);

    cJSON_AddNumberToObject(json,"Q0_1",cfg.Q0_1);
    cJSON_AddNumberToObject(json,"Q0_2",cfg.Q0_2);
    
    bool res=cJSON_PrintPreallocated(json,buffer,size,true);

    cJSON_Delete(json);

    return res;
}

bool ConfigLoader::save(const SensorConfig& cfg)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,SENSOR_CFG_FILENAME);

    FILE *file= fopen(filename,"w");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open sensor config for saving!");
        return false;
    }

    char buffer[512]={0};

    if(!ConfigLoader::toBuffer(buffer,512,cfg))
    {
        fclose(file);
        return false;
    }

    fwrite(buffer,sizeof(char),512,file);

    fclose(file);

    return true;
}

bool ConfigLoader::load(SensorConfig& cfg)
{
    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,SENSOR_CFG_FILENAME);

    FILE *file= fopen(filename,"r");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open sensor config for loading!");
        return false;
    }

    char buffer[512]={0};

    fread(buffer,sizeof(char),512,file);

    fclose(file);

    return ConfigLoader::fromBuffer(buffer,cfg);
}

bool ConfigLoader::save(const MagConfig& cfg)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,MAG_CFG_FILENAME);

    FILE *file= fopen(filename,"w");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open sensor config for saving!");
        return false;
    }

    char buffer[512]={0};

    if(!ConfigLoader::toBuffer(buffer,512,cfg))
    {
        fclose(file);
        return false;
    }

    fwrite(buffer,sizeof(char),512,file);

    fclose(file);

    return true;
}

bool ConfigLoader::load(MagConfig& cfg)
{
    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,MAG_CFG_FILENAME);

    FILE *file= fopen(filename,"r");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open sensor config for loading!");
        return false;
    }

    char buffer[512]={0};

    fread(buffer,sizeof(char),512,file);

    fclose(file);

    return ConfigLoader::fromBuffer(buffer,cfg);
}

bool ConfigLoader::save(const MotorCFG& cfg)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,MOTOR_PID_FILENAME);

    FILE *file= fopen(filename,"w");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open motors pid config for saving!");
        return false;
    }

    char buffer[2048]={0};

    if(!ConfigLoader::toBuffer(buffer,2048,cfg))
    {
        fclose(file);
        return false;
    }

    fwrite(buffer,sizeof(char),2048,file);

    fclose(file);

    return true;
}

bool ConfigLoader::load(MotorCFG& cfg)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,MOTOR_PID_FILENAME);

    FILE *file= fopen(filename,"r");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open motor config for loading!");
        return false;
    }

    char buffer[2048]={0};

    fread(buffer,sizeof(char),2048,file);

    fclose(file);


    return ConfigLoader::fromBuffer(buffer,cfg);
}


bool ConfigLoader::save(const PositionFilterCFG& cfg,const char* _filename)
{
    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,_filename);

    FILE *file= fopen(filename,"w");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open position filter config file for saving!");
        return false;
    }

    char buffer[2048]={0};

    if(!ConfigLoader::toBuffer(buffer,2048,cfg))
    {
        fclose(file);
        return false;   
    }

    fwrite(buffer,sizeof(char),2048,file);

    fclose(file);
    return true;
}

bool ConfigLoader::load(PositionFilterCFG& cfg,const char* _filename)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,_filename);

    FILE *file= fopen(filename,"r");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open position filter config file! %s",_filename);
        return false;
    }

    char buffer[2048]={0};

    fread(buffer,sizeof(char),2048,file);

    fclose(file);

    
    return ConfigLoader::fromBuffer(buffer,cfg);
}

bool ConfigLoader::save(const RotationFilterCFG& cfg)
{

    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,ROTATION_FILTER_FILENAME);

    FILE *file= fopen(filename,"w");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open rotation filter config for saving!");
        return false;
    }

    char buffer[2048]={0};

    if(!ConfigLoader::toBuffer(buffer,2048,cfg))
    {
        fclose(file);
        return false;
    }

    fwrite(buffer,sizeof(char),2048,file);

    fclose(file);

    return true;

}

bool ConfigLoader::load(RotationFilterCFG& cfg)
{
    char filename[255]={0};

    sprintf(filename,"/littlefs/%s/%s",CONFIG_FOLDER,ROTATION_FILTER_FILENAME);

    FILE *file= fopen(filename,"r");

    if(!file)
    {
        ESP_LOGE("Config","Cannot open rotation filter config file!");
        return false;
    }

    char buffer[2048]={0};

    fread(buffer,sizeof(char),2048,file);

    fclose(file);

    
    return ConfigLoader::fromBuffer(buffer,cfg);
}

};