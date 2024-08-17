#include <cstring>
#include <cmath>


#include "MotorDriver.hpp"
#include "SensorReader.hpp"

extern "C"
{
    #include "MadgwickAHRS.h"
}

namespace sensors
{

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
        ESP_LOGE("Sensors","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_SENSOR_PORT,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("Sensors","%s",esp_err_to_name(err));
    }

    /*

    // OLED MENU I2C initialization

    i2c_config_t conf1{};

    conf1.mode=I2C_MODE_MASTER;
    conf1.sda_io_num=MENU_SDA;
    conf1.scl_io_num=MENU_SCL;
    conf1.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed=I2C_MASTER_FREQ_HZ;
    //conf.clk_flags=0;

    esp_err_t err=ESP_OK;
    
    err=i2c_param_config(I2C_MENU_PORT,&conf1);

    if(err!=ESP_OK)
    {
        ESP_LOGE("Sensors","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_MENU_PORT,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("Sensors","%s",esp_err_to_name(err));
    }

    */
    //----------------------------
    // ADC initialization

    this->install_adc();

    //-----------------------------
    // PCINT initialization

    ch1.init(PCNT_CH1);
    ch2.init(PCNT_CH2);

    //----------------------------
    // Timer initialization

    this->install_timer(SAMPLE_FREQ);


    }

void SensorReader::init_sensors()
{

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
            ESP_LOGE("Sensors","Cannot switch to VL with id=%u",sensor);
            ESP_LOGE("Sensors","%s",esp_err_to_name(err));
            SensorsFaulty=true;
        }
    }
    

    if(!this->mpu.init(true))
    {
        ESP_LOGE("Sensors","MPU initialization failed!");
        SensorsFaulty=true;
    }

    this->mpu.setGyroscopeRange(MPU6050::GyroscopeRange::G500);
    this->mpu.setAccelerometerRange(MPU6050::AccelerometerRange::G4);

    this->mpu.setDLPFMode(MPU6050::DPLF_MODE::_5);

    vTaskDelay(100/portTICK_PERIOD_MS);

    this->mpu.DoGyroCalibration();

    this->mpu.DoAccelCalibration();

    this->mpu.printOffsets();

    this->init_mag();

    this->ch1.start();
    this->ch2.start();

    this->start_timer();
}


void SensorReader::install_timer(uint32_t _dt)
{
    timer_config_t config{};

    config.alarm_en=TIMER_ALARM_DIS;
    config.counter_en=TIMER_PAUSE;
    config.intr_type=TIMER_INTR_LEVEL;
    config.counter_dir=TIMER_COUNT_UP;
    config.auto_reload=TIMER_AUTORELOAD_EN;
    // Timer count with 1Mhz frequency
    config.divider=80;
    // 80Mhz clock
    config.clk_src=TIMER_SRC_CLK_APB;


    timer_init(TIMER_GROUP_0,TIMER_0,&config);
    timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,1000000/_dt);
    timer_set_alarm(TIMER_GROUP_0,TIMER_0,TIMER_ALARM_EN);
    timer_enable_intr(TIMER_GROUP_0,TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0,TIMER_0,this->_timer_callback_wrapper,this,0);
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
    uint8_t i=0;

   for(uint8_t channel : KTIRChannel)
    {
        int result;

        ESP_ERROR_CHECK(adc_oneshot_read(this->hmd,static_cast<adc_channel_t>(channel), &result));

        this->reads.floor_sensors[i++]= result>=this->KtirThreshold;

        //ESP_LOGI("Sensors","ADC %x : %d",i,result);
    }

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
        ESP_LOGD("Sensors","Found HMC 5883 magnetrometer!");

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
        ESP_LOGD("Sensors","Found QMC 5883 magnetrometer!");

        qmc->setFullScale(QMC5883::FullScale::G2);
        qmc->setOutputDataRate(QMC5883::OutputDataRate::_50Hz);
        qmc->setMode(QMC5883::Mode::Continuous);
        qmc->setIntEnable(true);

        this->mag=qmc;
        return;
    }

    delete qmc;

    this->mag=NULL;

    ESP_LOGE("Sensors","No magnetrometer found!");
}

void SensorReader::read_mag()
{
    this->magReading.x=this->mag->readX();
    this->magReading.y=this->mag->readY();
    this->magReading.z=this->mag->readZ();
}

SensorReader::SensorReader()
{
    SensorsFaulty=false;
    xCycleTask=false;
    KtirThreshold=KTIR_THRESHOLD;
    this->CalibrateIMU=false;

    yaw_error_tolerance=0;
    distance_error_tolerance=0;
    
}

void SensorReader::init(const config::SensorConfig& _config)
{

    this->update_cfg(_config);

    this->init_peripherials();

    this->init_sensors();

}

void SensorReader::step()
{
    if(SensorsFaulty)
    {
        ESP_LOGE("Sensors","Sensors general fault!");
        this->reads.stoped=true;
        return;
    }

    if(CalibrateIMU)
    {   
        this->reads.position=Vec2Df(0.f);
        this->reads.yaw=0.f;
        ESP_LOGI("Sensors","Starting IMU calibration");
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_LOGI("Sensors","Performing IMU calibration");
        this->mpu.DoGyroCalibration();
        this->mpu.DoAccelCalibration();
        ESP_LOGI("Sensors","IMU calibration has finished!");

        CalibrateIMU=false;
    }

    //-------------------------------------
        // do every dt time

    if(mpu.IntStatus()&(1<<0))
    {

        Vec3Df _gyro=this->mpu.readGyroscope();
        Vec3Df _accel=this->mpu.readAccelerometer();

        gyroMean.push(_gyro);

        accelMean.push(_accel);

    }

    if(this->mag!=NULL)
    {
        if(this->mag->checkDataReady())
        {
            this->read_mag();
        }
    }


    // Cycle function
    if(this->xCycleTask)
    {
        this->reads.IMUOnlyReading=false;

        int32_t step_ch1=ch1.get();
        int32_t step_ch2=ch2.get();

        step_ch1 = MotorDriver::channelADirection() ? step_ch1 : -step_ch1;
        step_ch2 = MotorDriver::channelBDirection() ? step_ch2 : -step_ch2;

        float dl=step_ch1/PULSE_TO_DISTANCE;
        float dr=step_ch2/PULSE_TO_DISTANCE;
        
        //distance
        float dx=(dl+dr)/2.f;

        //change of rotation
        float d0=(dl-dr)/(2*D_WHEELS);

        //ESP_LOGI("Sensors","PCINT 1 steps: %ld",step_ch1);
        //ESP_LOGI("Sensors","PCINT 2 steps: %ld",step_ch2);

        ch1.clear();
        ch2.clear();

        // integral from gyroscope readings

        Vec3Df _gyroMean=this->gyroMean.mean();
        Vec3Df _accelMean=this->accelMean.mean();

        // we only care about 2D projection from top view:
        // so only yaw axis from IMU

        MadgwickAHRSupdate(_gyroMean.x,_gyroMean.y,_gyroMean.z,_accelMean.x,_accelMean.y,_accelMean.z,this->magReading.x,this->magReading.y,this->magReading.z);

        float _roll=0.f;
        float _pitch=0.f;
        float _yaw=0.f;

        MadgwickQuaterionToEuler(&_roll,&_pitch,&_yaw);

        this->reads.yaw=this->rotor.step(d0,_yaw);

        this->reads.position.x=this->posfilter_x.step(dx*cos(this->reads.yaw),_accelMean.x);
        this->reads.position.y=this->posfilter_y.step(dx*sin(this->reads.yaw),_accelMean.y);

        this->xCycleTask=false;

        ESP_LOGD("Sensors","Yaw: %f",this->reads.yaw);
        ESP_LOGD("Sensors","X: %f Y: %f",this->reads.position.x,this->reads.position.y);
    }

    //-------------------------------------
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
    //-------------------------------------
    //KTIR floor sensor reading

    this->read_adc();

}

SensorReader::~SensorReader()
{
    delete this->vl;
}

};