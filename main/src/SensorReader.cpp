#include <cstring>
#include <cmath>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "MotorDriver.hpp"
#include "SensorReader.hpp"

#include "shared.hpp"

extern "C"
{
    #include "MadgwickAHRS.h"
}

namespace sensors
{

static void IRAM_ATTR gpio_isr_imu_handler(void* arg)
{
    SensorReader* reader = (SensorReader*)arg;

    portYIELD_FROM_ISR( reader->imu_gpio_interupt() );
}

// it will have 1kHz sampling frequency
static void IMU_task(void* arg)
{
    SensorReader* reader = (SensorReader*)arg;

    while(true)
    {
        // wait for interrupt on pin

        reader->read_imu();

    }
}

static void Encoders_task(void* arg)
{
    SensorReader* reader = (SensorReader*)arg;

    while(true)
    {
        reader->read_encoders();
        
        vTaskDelay(ENCODER_UPDATE_TIME_MS/portTICK_PERIOD_MS);
    }
}

static void TOFs_task(void* arg)
{
    SensorReader* reader = (SensorReader*)arg;

    while(true)
    {
        reader->tofs_read();        
        
        vTaskDelay(VL_SAMPLE_TIME_MS/portTICK_PERIOD_MS);
    }
}

static void Fusion_task(void* arg)
{
    SensorReader* reader = (SensorReader*)arg;

    while(true)
    {

        reader->fusion();        
        
        vTaskDelay(SAMPLE_TIME_MS/portTICK_PERIOD_MS);
    }
}

void SensorReader::init_peripherials()
{
//I2C for sensors
    i2c_config_t conf{};

    conf.mode=I2C_MODE_MASTER;
    conf.sda_io_num=SENSOR_SDA;
    conf.scl_io_num=SENSOR_SCL;
    conf.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf.master.clk_speed=I2C_MASTER_FREQ_HZ;
    //conf.clk_flags=0;

    esp_err_t err=ESP_OK;
    
    err=i2c_param_config(I2C_SENSOR_PORT,&conf);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_SENSOR_PORT,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("MAIN","%s",esp_err_to_name(err));
    }

    //----------------------------
    // ADC initialization

    this->install_adc();

    //-----------------------------
    // PCINT initialization

    ch1.init(PCNT_CH1);
    ch2.init(PCNT_CH2);

    //----------------------------

    }

void SensorReader::init_sensors()
{

    this->imuEvent = xEventGroupCreate();

    gpio_set_direction(MPU6050_INT_PIN,GPIO_MODE_INPUT);

    gpio_set_intr_type(MPU6050_INT_PIN,GPIO_INTR_NEGEDGE);

    gpio_isr_handler_add(MPU6050_INT_PIN, gpio_isr_imu_handler, this);

    this->vl=new MultiVL(I2C_SENSOR_PORT);

    for(uint8_t sensor : SensorList)
    {
        esp_err_t err=this->vl->SwitchSensor(sensor);
        if(err==ESP_OK)
        {
            if(!this->vl->init())
            {
                ESP_LOGE("Sensors","Cannot initialize VL with id=%u",sensor);
            }
            else
            {
            // other configuration like measure mode, max distance etc.

                ESP_LOGI("Sensors","VL with id=%u has been initialized!",sensor);
                this->vl->setTimingBudget(20000);
                this->vl->setSignalRateLimit(0.5);
                this->vl->setVcselPulsePeriodPre(12);
                this->vl->setVcselPulsePeriodFinal(8);

                #ifdef TOF_CONTINOUS
                this->vl->setContinousMode();
                #endif
            }
        }
        else
        {
            ESP_LOGE("MAIN","Cannot switch to VL with id=%u",sensor);
            ESP_LOGE("MAIN","%s",esp_err_to_name(err));
            // SensorsFaulty=true;
        }
    }
    

    if(!this->mpu.init(true))
    {
        ESP_LOGE("MAIN","MPU initialization failed!");
        SensorsFaulty=true;
    }

    this->mpu.setGyroscopeRange(MPU6050::GyroscopeRange::G500);
    this->mpu.setAccelerometerRange(MPU6050::AccelerometerRange::G4);

    this->mpu.setDLPFMode(MPU6050::DPLF_MODE::_5);

    vTaskDelay(100/portTICK_PERIOD_MS);

    this->init_mag();

    this->ch1.start();
    this->ch2.start();
}


void SensorReader::install_adc()
{
   // we are going to use one shoot mode

   adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &this->hmd));

    adc_oneshot_chan_cfg_t config;

    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_11;

    for(uint8_t channel : KTIRChannel)
    {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(this->hmd, static_cast<adc_channel_t>(channel), &config));
    }

}

void SensorReader::read_adc()
{
    ESP_LOGD("MAIN","ADC reading!!");
    uint8_t i=0;

   for(uint8_t channel : KTIRChannel)
    {
        int result;

        ESP_ERROR_CHECK(adc_oneshot_read(this->hmd,static_cast<adc_channel_t>(channel), &result));

        this->reads.floor_sensors[i++]= result>=this->KtirThreshold;

        //ESP_LOGI("Sensors","ADC %x : %d",i,result);
    }

    // read a battery voltage

    int reading;

    ESP_ERROR_CHECK(adc_oneshot_read(this->hmd,BATTERY_ADC_CHANNEL, &reading));

    this->reads.battery_voltage = (reading*168)/34750;

    // if( this->reads.battery_voltage <= 12.8 )
    // {
    //     ESP_LOGI("MAIN","Low battery voltage!!");

    //     // do something with it
    //     //esp_deep_sleep_start();
    // }
}

uint8_t SensorReader::from_angel_to_sensor_index(const float& angel) const
{
    // we want index 8 to represent 0 radians
    return (8+(static_cast<uint8_t>(angel/SENSOR_SPACING)))%18;
}

void SensorReader::init_mag()
{
    HMC5883 *hmc=new HMC5883(I2C_SENSOR_PORT);

    if(hmc->checkDevice())
    {
        ESP_LOGD("MAIN","Found HMC 5883 magnetrometer!");

        hmc->setGain(HMC5883::Gain::LSB_660);
        hmc->setDataOutputRate(HMC5883::DataOutputRate::_7500);
        hmc->setMeasurmentMode(HMC5883::MeasurmentMode::Normal);
        hmc->setMode(HMC5883::Mode::Continouse);
        hmc->setSampleAveraging(HMC5883::SampleAverage::_4);

        this->mag=hmc;
        return;
    }

    delete hmc;

    QMC5883 *qmc=new QMC5883(I2C_SENSOR_PORT);

    if(qmc->checkChipID())
    {
        ESP_LOGD("MAIN","Found QMC 5883 magnetrometer!");

        qmc->setFullScale(QMC5883::FullScale::G2);
        qmc->setOutputDataRate(QMC5883::OutputDataRate::_50Hz);
        qmc->setMode(QMC5883::Mode::Continuous);
        qmc->setIntEnable(true);

        this->mag=qmc;
        return;
    }

    delete qmc;

    this->mag=NULL;

    this->reads.IMUOnlyReading = true;

    ESP_LOGE("MAIN","No magnetrometer found!");
}

void SensorReader::read_mag()
{
    this->reads.magReading = this->mag->read();
}

SensorReader::SensorReader()
{
    this->semp = xSemaphoreCreateMutex();
    SensorsFaulty=false;
    KtirThreshold=KTIR_THRESHOLD;
    this->CalibrateIMU=false;
    this->CalibrationCounter=0;

    yaw_error_tolerance=0;
    distance_error_tolerance=0;
    
}


void SensorReader::init_tasks()
{
    this->encoder_stack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);
    
    if( xTaskCreateStaticPinnedToCore(Encoders_task,"Encoders",MIN_TASK_STACK_SIZE,this,tskIDLE_PRIORITY+1,this->encoder_stack,&this->encoder_task,xPortGetCoreID()) == NULL )
    {
        ESP_LOGE("MAIN","Cannot create encoder task");
    }

    this->tof_stack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);

    if( xTaskCreateStaticPinnedToCore(TOFs_task,"TOFs",MIN_TASK_STACK_SIZE,this,tskIDLE_PRIORITY+2,this->tof_stack,&this->tof_task,xPortGetCoreID()) == NULL )
    {
        ESP_LOGE("MAIN","Cannot create TOFs task");
    }

    if( !SensorsFaulty )
    {
        this->imu_stack = (StackType_t*)malloc(MAIN_TASK_STACK_SIZE);

        if( xTaskCreateStaticPinnedToCore(IMU_task,"IMU",MAIN_TASK_STACK_SIZE,this,configMAX_PRIORITIES-1,this->imu_stack,&this->imu_task,xPortGetCoreID()) == NULL )
        {
            ESP_LOGE("MAIN","Cannot create IMU task");
        }

    }

    this->fusion_stack = (StackType_t*)malloc(MAIN_TASK_STACK_SIZE);

    if( xTaskCreateStaticPinnedToCore(Fusion_task,"Fusion",MAIN_TASK_STACK_SIZE,this,configMAX_PRIORITIES-1,this->fusion_stack,&this->fusion_task,xPortGetCoreID()) == NULL )
    {
        ESP_LOGE("MAIN","Cannot create Fusion task");
    }

}

void SensorReader::init(const config::SensorConfig& _config)
{

    this->update_cfg(_config);

    this->init_peripherials();

    this->init_sensors();

    this->init_tasks();

}

void SensorReader::read_encoders()
{
    ESP_LOGD("MAIN","Encoder reading!!");
    this->Lock();

    int32_t step_ch1 = ch1.get();
    int32_t step_ch2 = ch2.get();

    step_ch1 = shared::mods.driver->channelADirection() ? step_ch1 : -step_ch1;
    step_ch2 = shared::mods.driver->channelBDirection() ? step_ch2 : -step_ch2;

    float dl = step_ch1/PULSE_TO_DISTANCE;
    float dr = step_ch2/PULSE_TO_DISTANCE;
    
    //distance
    float dx = (dl+dr)/2.f;

    //change of rotation
    float d0 = (dl-dr)/(2*D_WHEELS);

    this->reads.eyaw += d0;

    ESP_LOGD("Sensors","PCINT 1 steps: %ld",step_ch1);
    ESP_LOGD("Sensors","PCINT 2 steps: %ld",step_ch2);

    ch1.clear();
    ch2.clear();

    this->reads.motorSpeed[0] = dl/ENCODER_UPDATE_TIME;
    this->reads.motorSpeed[1] = dr/ENCODER_UPDATE_TIME;

    this->reads.epostion.x += dx*cos(this->reads.eyaw);
    this->reads.epostion.y += dx*sin(this->reads.eyaw);


    //KTIR floor sensor reading

    this->read_adc();

    this->Unlock();

}

void SensorReader::fusion()
{
    ESP_LOGD("MAIN","Fusion task!!");

    this->Lock();

    if( this->CalibrateIMU )
    {
        MadgwickReset();
        this->Unlock();
        return;
    } 

    Vec3Df _gyroMean=this->gyroMean.mean();
    Vec3Df _accelMean=this->accelMean.mean();

    // we only care about 2D projection from top view:
    // so only yaw axis from IMU
    if( !this->reads.IMUOnlyReading )
    {
        MadgwickAHRSupdate(_gyroMean.x,_gyroMean.y,_gyroMean.z,_accelMean.x,_accelMean.y,_accelMean.z,this->reads.magReading.x,this->reads.magReading.y,this->reads.magReading.z);
    }
    else
    {
        MadgwickAHRSupdateIMU(_gyroMean.x,_gyroMean.y,_gyroMean.z,_accelMean.x,_accelMean.y,_accelMean.z);
    }

    float _roll=0.f;
    float _pitch=0.f;
    float _yaw=0.f;

    MadgwickQuaterionToEuler(&_roll,&_pitch,&_yaw);

    // to do:
    this->reads.yaw=this->rotor.step(this->reads.eyaw,_yaw);

    this->reads.position.x=this->posfilter_x.step(this->reads.epostion.x,_accelMean.x);
    this->reads.position.y=this->posfilter_y.step(this->reads.epostion.y,_accelMean.y);

    // leave it this way for now
    this->reads.yaw = _yaw;

    // leave it this way for now
    this->reads.position.x = this->reads.epostion.x;
    this->reads.position.y = this->reads.epostion.y;

    ESP_LOGD("Sensors","Yaw: %f",this->reads.yaw);
    ESP_LOGD("Sensors","X: %f Y: %f",this->reads.position.x,this->reads.position.y);

    this->Unlock();
}

void SensorReader::tofs_read()
{
    ESP_LOGD("MAIN","TOF task!!");

    this->Lock();
    // distance sensor reading
    uint16_t distance=0;
    float angel=0.0;

    for(uint16_t& dis : this->reads.distances)
    {
        // out of sight
        dis=8190;
    }

    for(uint8_t i=0;i<NUM_OF_SENSORS;++i)
    {
        this->vl->SwitchSensor(SensorList[i]);

        #ifdef TOF_CONTINOUS
            distance=this->vl->readContinous();
        #else
            distance=this->vl->read();
        #endif

        angel=SensorAngleOffset[i]+this->reads.yaw;

        this->reads.distances[this->from_angel_to_sensor_index(angel)]=distance;

    }

    this->Unlock();
}

BaseType_t IRAM_ATTR SensorReader::imu_gpio_interupt()
{
    BaseType_t pxHigherPriorityTaskWoken;

    xEventGroupSetBitsFromISR(this->imuEvent,1<<0,&pxHigherPriorityTaskWoken);

    return pxHigherPriorityTaskWoken;
}

void SensorReader::read_imu()
{

    xEventGroupWaitBits(this->imuEvent,1<<0,true,true,portMAX_DELAY);

    ESP_LOGD("MAIN","IMU task!!");

    Vec3Di _raw_gryo = this->mpu.readRawGyroscope();
    Vec3Di _raw_accel = this->mpu.readRawAccelerometer();

    Vec3Df _gyro=this->mpu.from_raw_gyro(_raw_gryo);
    Vec3Df _accel=this->mpu.from_raw_accel(_raw_accel);

    this->gyroMean.push(_gyro);
    this->accelMean.push(_accel);

    if( this->CalibrateIMU )
    {   
        this->gyroCalibrMean.push(_raw_gryo);
        this->accelCalibrMean.push(_raw_accel);

        if( this->CalibrationCounter > this->CalbirationSteps )
        {
            this->CalibrationCounter = 0;

            Vec3Df gyro_calibr = this->gyroMean.mean();
            Vec3Df accel_calibr = this->accelMean.mean();

            float gyro_variance = gyro_calibr.variance();
            float accel_mean = accel_calibr.mean();

            if(( gyro_variance < 0.01f ) && ( abs(accel_mean) <= 0.375f ))
            {
                this->CalibrateIMU = false;

                ESP_LOGI("MAIN","IMU calibration finished!");  

            }
            else
            {          

                this->mpu.setGyroOffsets(this->mpu.getGyroOffsets()+this->gyroCalibrMean.mean());

                // we have to take into account a g force!!!

                // find two axis with the biggest values:

                Vec3Di accel_offset = this->accelCalibrMean.mean();

                accel_offset.z -= this->mpu.accel_to_raw(1.f);

                accel_offset += this->mpu.getAccelOffsets();              

                this->mpu.setAccelOffsets(accel_offset);

                this->mpu.printOffsets();

                this->gyroMean.reset();
                this->accelMean.reset();

                this->gyroCalibrMean.reset();
                this->accelCalibrMean.reset();

            }
        }

        this->CalibrationCounter++;

    }


    if( this->mag != NULL )
    {
        if(this->mag->checkDataReady())
        {
           this->read_mag();
        }
    }

}

SensorReader::~SensorReader()
{
    delete this->vl;
}

};