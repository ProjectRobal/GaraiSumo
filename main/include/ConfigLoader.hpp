/*
    A class for configuratiom loading.

*/

#pragma once

#include <cstring>

extern "C"
{

#include <cJSON.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_log.h>

#include <esp_idf_version.h>
#include <esp_flash.h>
#include <esp_chip_info.h>
#include <spi_flash_mmap.h>

#include "esp_littlefs.h"
#include "esp_spiffs.h"
#include <sys/stat.h>

}

#include "Vec.hpp"
#include "PID.hpp"

#include "MPU6050.hpp"

#define SENSOR_CFG_FILENAME "littlefs/sensor.json"

#define MAG_CFG_FILENAME "littlefs/mag.json"

#define MOTOR_PID_FILENAME "littlefs/motor_pid.json"

#define ROTATION_FILTER_FILENAME "littlefs/rotor.json"

#define CONFIG_FOLDER "cfg"


namespace config
{

    struct SensorConfig
    {
        MPU6050::calibr_data_t calibr;
        float beta;
    };

    struct MagConfig
    {
        Vec3Df offsets;
        double c_matrix[9];

        double xV;
        double xW;
        double yV;
        double yW;
        double zV;
        double zW;
    };

    struct MotorPID
    {
        float P;
        float I;
        float D;
    };

    struct MotorCFG
    {
        MotorPID motor1;
        MotorPID motor2;
        MotorPID motor3;
    };

    struct PositionFilterCFG
    {
        float Ex1;
        float Ex2;
        float R;

    };

    struct RotationFilterCFG
    {
        float Ex1;
        float Ex2;
        float R;
    };

    struct Kalman1DCFG
    {
        float W;
        float V;
        float Q0;
    };

    struct PartitionInfo
    {
        size_t total;
        size_t used;
    };

    class ConfigLoader
    {
        public:

        static MagConfig mag_default()
        {
            return {
                .offsets = Vec3Df(0,0,0),
                .c_matrix = {1.f,0.f,0.f,0.f,1.f,0.f,0.f,0.f,1.f},
                .xV = 0.00006,
                .xW = 0.0000089,
                .yV = 0.00006,
                .yW = 0.0000089,
                .zV = 0.00006,
                .zW = 0.000011
            };
        }

        static void init_filesystem();

        static void print_filesystem_info();

        static esp_vfs_littlefs_conf_t fs;

        template<typename T>
        static cJSON* vector3d_to_json(const Vec3D<T>& vec)
        {
            cJSON* obj= cJSON_CreateObject();

            cJSON* x= cJSON_CreateNumber(vec.x);
            cJSON* y= cJSON_CreateNumber(vec.y);
            cJSON* z= cJSON_CreateNumber(vec.z);

            cJSON_AddItemToObject(obj,"x",x);
            cJSON_AddItemToObject(obj,"y",y);
            cJSON_AddItemToObject(obj,"z",z);

            return obj;
        }

        template<typename T>
        static bool json_to_vector3d(cJSON* obj,Vec3D<T>& vec)
        {
            const cJSON* x=cJSON_GetObjectItemCaseSensitive(obj,"x");

            if(cJSON_IsNumber(x))
            {
                vec.x=x->valuedouble;
            }
            else
            {
                return false;
            }

            const cJSON* y=cJSON_GetObjectItemCaseSensitive(obj,"y");

            if(cJSON_IsNumber(y))
            {
                vec.y=y->valuedouble;
            }
            else
            {
                return false;
            }

            const cJSON* z=cJSON_GetObjectItemCaseSensitive(obj,"z");

            if(cJSON_IsNumber(z))
            {
                vec.z=z->valuedouble;
            }
            else
            {
                return false;
            }

            return true;
        }

        static cJSON* pid_to_json(const MotorPID& pid)
        {
            cJSON* obj= cJSON_CreateObject();

            cJSON* p= cJSON_CreateNumber(pid.P);
            cJSON* i= cJSON_CreateNumber(pid.I);
            cJSON* d= cJSON_CreateNumber(pid.D);

            cJSON_AddItemToObject(obj,"P",p);
            cJSON_AddItemToObject(obj,"I",i);
            cJSON_AddItemToObject(obj,"D",d);

            return obj;
        }

        static bool json_to_pid(cJSON* obj,MotorPID& pid)
        {
            const cJSON* p=cJSON_GetObjectItemCaseSensitive(obj,"P");

            if(cJSON_IsNumber(p))
            {
                pid.P=p->valuedouble;
            }
            else
            {
                return false;
            }

            const cJSON* i=cJSON_GetObjectItemCaseSensitive(obj,"I");

            if(cJSON_IsNumber(i))
            {
                pid.I=i->valuedouble;
            }
            else
            {
                return false;
            }

            const cJSON* d=cJSON_GetObjectItemCaseSensitive(obj,"D");

            if(cJSON_IsNumber(d))
            {
                pid.D=d->valuedouble;
            }
            else
            {
                return false;
            }

            return true;

        }

        static bool from_json_to_array(cJSON* arr,float*out)
        {
            if(cJSON_IsArray(arr))
            {
                uint8_t i=0;
                cJSON* element;
                cJSON_ArrayForEach(element,arr)
                {
                    if(!cJSON_IsNumber(element))
                    {
                        return false;
                    }

                    out[i++]=element->valuedouble;

                }
            }

            return true;
        }

        static PartitionInfo info();

        static void init();

        static bool fromBuffer(const char* buffer,SensorConfig& cfg);

        static bool toBuffer(char* buffer,size_t size,const SensorConfig& cfg);

        static bool fromBuffer(const char* buffer,MagConfig& cfg);

        static bool toBuffer(char* buffer,size_t size,const MagConfig& cfg);

        static bool fromBuffer(const char* buffer,MotorCFG& cfg);

        static bool toBuffer(char* buffer,size_t size,const MotorCFG& cfg);

        static bool fromBuffer(const char* buffer,PositionFilterCFG& cfg);

        static bool toBuffer(char* buffer,size_t size,const PositionFilterCFG& cfg);

        static bool fromBuffer(const char* buffer,RotationFilterCFG& cfg);

        static bool toBuffer(char* buffer,size_t size,const RotationFilterCFG& cfg);


        static bool save(const SensorConfig& cfg);

        static bool load(SensorConfig& cfg);

        static bool save(const MagConfig& cfg);

        static bool load(MagConfig& cfg);

        static bool save(const MotorCFG& cfg);

        static bool load(MotorCFG& cfg);

        static bool save(const PositionFilterCFG& cfg,const char* filename);

        static bool load(PositionFilterCFG& cfg,const char* filename);

        static bool save(const RotationFilterCFG& cfg);

        static bool load(RotationFilterCFG& cfg);

    };

}

