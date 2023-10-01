#pragma once

extern "C"
{

#include "i2c.h"

class VL53L0X
{
    protected:

    typedef enum
    {
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
    } calibration_type_t;

    uint32_t measurement_timing_budget_us;
    uint8_t i2c_port;
    uint8_t stop_variable;
    uint8_t address;

    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

    bool perform_single_ref_calibration(calibration_type_t calib_type);

    bool read_strobe();

    bool set_sequence_steps_enabled(uint8_t sequence_step);

    bool configure_interrupt();

    bool load_default_tuning_settings();

    bool get_spad_info_from_nvm(uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6]);

    bool init_config();

    bool data_init();

    bool static_init();

    bool perform_ref_calibration();

    bool set_spads_from_nvm();

    uint16_t decodeTimeout(uint16_t reg_val);

    uint16_t encodeTimeout(uint32_t timeout_mclks);

    public:

    VL53L0X(i2c_port_t i2c_port = I2C_NUM_0, gpio_num_t gpio_xshut = GPIO_NUM_MAX,
          gpio_num_t gpio_gpio1 = GPIO_NUM_MAX);

    bool init();

    bool setContinousMode();

    bool stopContinuousMode();

    uint16_t readContinous();

    uint16_t read();

    bool setTimingBudget(uint32_t timing);

    uint32_t getTimingBudget();

    bool setSignalRateLimit(float limit);

    float getSignalRateLimit();

    bool setVcselPulsePeriodPre(uint8_t period);

    bool setVcselPulsePeriodFinal(uint8_t period);

    bool setAdress(uint8_t address);

    const uint8_t& getAddress() const
    {
        return this->address;
    }
};

class MultiVL : public VL53L0X
{
    uint8_t multiplexer_address;

    uint8_t current_sensor;

    esp_err_t write_to_multi(uint8_t byte)
    {
        esp_err_t err=ESP_OK;

        i2c_cmd_handle_t cmd=i2c_cmd_link_create();

        i2c_master_start(cmd);

        i2c_master_write_byte(cmd,this->multiplexer_address<<1 | I2C_MASTER_WRITE,I2C_MASTER_ACK);

        i2c_master_write_byte(cmd,byte,I2C_MASTER_NACK);

        i2c_master_stop(cmd);

        err=i2c_master_cmd_begin((i2c_port_t)this->i2c_port,cmd,10/portTICK_PERIOD_MS);

        i2c_cmd_link_delete(cmd);

        return err;

    }

    esp_err_t clear_multiplexer()
    {
        return write_to_multi(0x00);
    }

    public:


    MultiVL(i2c_port_t i2c_port = I2C_NUM_0, uint8_t _multiplexer_address=0x70, gpio_num_t gpio_xshut = GPIO_NUM_MAX,
          gpio_num_t gpio_gpio1 = GPIO_NUM_MAX)
    : VL53L0X(i2c_port,gpio_xshut,gpio_gpio1),
    multiplexer_address(_multiplexer_address)
    {
        current_sensor=0;
    }

    esp_err_t SwitchSensor(uint8_t id)
    {
        esp_err_t err=write_to_multi(1<<id);
        if(err==ESP_OK)
        {
            this->current_sensor=id;
            return ESP_OK;
        }
        return err;
    }

    const uint8_t& SensorSelected() const
    {
        return this->current_sensor;
    }

};

}