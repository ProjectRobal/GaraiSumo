#include <stdio.h>
#include <esp_wifi.h>
#include <esp_netif_net_stack.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <driver/gpio.h>

#include "ConfigLoader.hpp"

#include "config.h"

#include "OnlineTerminal.hpp"
#include "OLED.hpp"
#include "oledModes.hpp"

#include "shared.hpp"

//esp_vfs_littlefs_conf_t config::ConfigLoader::fs=esp_vfs_littlefs_conf_t();

extern "C"
{

using shared::mods;

const uint8_t rides_of_valkyrya[]={30,30,0,0,30,30,0,0,0,30,30,30,30,30,30,0,0,0,27,27,27,27,27,27,27,27,0,0,30,30,30,30,30,30,30,30,30,30,0,0,0,0,
30,30,0,0,30,30,0,0,0,30,30,30,30,30,30,0,0,0,27,27,27,27,27,27,27,27,0,0,30,30,30,30,30,30,30,30,30,30,0,0,0,0,
33,33,0,36,36,36,36,36,36,0,36,36,36,36,36,36,0,0,0,33,33,33,33,33,33,0,0,0,30,30,0,30,30,0,30,0,0,33,33,33,33,0,33,33,33,33,0,0,0,0,0,0,0};

void wifi_init();

void oled_loop(void *arg);

void main_loop(void *arg);

void app_main()
{
    esp_log_level_set("*",ESP_LOG_DEBUG);
    esp_log_level_set("MAIN", ESP_LOG_DEBUG);
    esp_log_level_set("Sensors", ESP_LOG_INFO);

    ESP_LOGI("MAIN","Hello World!");

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);


    // for testing purpose
//    oled_loop(NULL);

    mods.audio_play=new RuraPlayer(rides_of_valkyrya,sizeof(rides_of_valkyrya)/sizeof(uint8_t),25);

    //mods->audio_play->play();

    config::SensorConfig sensor_cfg; 

    config::MotorCFG motor_cfg;

    config::PositionFilterCFG posX;

    config::PositionFilterCFG posY;

    config::RotationFilterCFG rotor;

    mods.sensors=new sensors::SensorReader();

    mods.driver=new MotorDriver();

    OnlineTerminal* terminal=new OnlineTerminal();

    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    config::ConfigLoader::init();

    //load default data in case of failure in loading data
    // it cannot sace configuration
    if(!config::ConfigLoader::load(sensor_cfg))
    {
        sensor_cfg.calibr.accelerometer=Vec3Di(0);
        sensor_cfg.calibr.gyroscope=Vec3Di(0);  
        sensor_cfg.distance_error_tolerance=DISTANCE_TOLERANCE;
        sensor_cfg.yaw_error_tolerance=YAW_TOLERANCE; 

        config::ConfigLoader::save(sensor_cfg);

    }

    if(!config::ConfigLoader::load(motor_cfg))
    {
        motor_cfg.motor1={
            .P=1.0,
            .I=0.0,
            .D=0.0
        };
        motor_cfg.motor2={
            .P=-1.0,
            .I=0.0,
            .D=0.0
        };

        config::ConfigLoader::save(motor_cfg);

    }

    if(!config::ConfigLoader::load(posX,POS_X_CFG_FILE))
    {
        posX={
            .Ex1=1.f,
            .Ex2=1.f,
            .R=1.f
        };

        config::ConfigLoader::save(posX,POS_X_CFG_FILE);

    }

    if(!config::ConfigLoader::load(posY,POS_Y_CFG_FILE))
    {
        posY={
            .Ex1=1.f,
            .Ex2=1.f,
            .R=1.f
        };

        config::ConfigLoader::save(posY,POS_Y_CFG_FILE);

    }

    if(!config::ConfigLoader::load(rotor))
    {
        rotor={
            .Ex1=1.f,
            .Ex2=1.f,
            .R=1.f,
            .Q0_1=0.f,
            .Q0_2=0.f
        };

        config::ConfigLoader::save(rotor);

    }


    wifi_init();

    terminal->init();

    mods.driver->setMotorConfig(motor_cfg);

    mods.driver->init();

    mods.sensors->init(sensor_cfg);

    mods.sensors->updatePositionFilterXCFG(posX);

    mods.sensors->updatePositionFilterYCFG(posY);

    mods.sensors->updateRoationFilterCFG(rotor);

    // init watchdog

    // init screen task


}


void oled_loop(void *arg)
{

    bool fun_button_state;
    bool mode_button_state;

    gpio_set_direction(FUN_BUTTON,GPIO_MODE_INPUT);
    gpio_set_direction(MODE_BUTTON,GPIO_MODE_INPUT);
    gpio_pullup_en(FUN_BUTTON);
    gpio_pullup_en(MODE_BUTTON);

    fun_button_state=gpio_get_level(FUN_BUTTON);
    mode_button_state=gpio_get_level(MODE_BUTTON);

    // OLED MENU I2C initialization
    i2c_config_t conf1{};

    conf1.mode=I2C_MODE_MASTER;
    conf1.sda_io_num=MENU_SDA;
    conf1.scl_io_num=MENU_SCL;
    conf1.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed=400000;
    //conf.clk_flags=0;

    esp_err_t err=ESP_OK;
    
    err=i2c_param_config(I2C_MENU_PORT,&conf1);

    if(err!=ESP_OK)
    {
        ESP_LOGE("OLED","%s",esp_err_to_name(err));
    }

    err=i2c_driver_install(I2C_MENU_PORT,I2C_MODE_MASTER,0,0,0);

    if(err!=ESP_OK)
    {
        ESP_LOGE("OLED","%s",esp_err_to_name(err));
    }

    uint8_t page_id=0;

    oled::OLED screen(I2C_MENU_PORT);

    screen.init(128,64);

    screen.setFont(ssd1306xled_font6x8);

    //screen.drawPixel(64,32,1);

    screen.drawText(25,15,1,"  ABC  ");

    screen.drawRect(0,0,25,25,1);

    screen.setFont(ssd1306xled_font8x16);

    //screen.drawPixel(64,32,1);>

    screen.drawText(25,45,1,"!Hello KoNaR!");

    //screen.drawLine(0,63,127,0,1);

    screen.draw();

    while(true)
    {

        vTaskDelay(200/portTICK_PERIOD_MS);
    }

    while(true)
    {
        // execute oled loop here

        screen.clear();

        uint8_t ret=oled_modes::modes[page_id](screen,&mods,fun_button_state,mode_button_state);

        if(ret!=0)
        {

            page_id=ret-1;

            if(page_id>OLED_MODES_COUNT)
            {
                page_id=0;
            }

        }
       
        screen.draw();

        fun_button_state=gpio_get_level(FUN_BUTTON);
        mode_button_state=gpio_get_level(MODE_BUTTON);

        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}



void wifi_init()
{

    wifi_init_config_t w_config=WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&w_config));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

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

    ESP_ERROR_CHECK(esp_wifi_start());
}

// split it to other tasks
void main_loop(void *arg)
{
    ESP_LOGI("MAIN","Entering main loop!");

    /*
        So something inside these functions cause panic to ariase:

    */
    
    while(true)
    {
        // sensor read step
        /*mods->sensors->step();

        if(!mods->sensors->read().stoped)
        {

            // program block step

            
            // motor control step
            mods->driver->loop(mods->sensors->read());

        }
        else
        {
            mods->driver->stop();
        }*/
        vTaskDelay(1000/portTICK_PERIOD_MS);

    }
}


}