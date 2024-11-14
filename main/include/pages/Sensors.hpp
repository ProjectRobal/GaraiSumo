#pragma once

#include <cstring>
#include <cstdlib>

#include "utils.hpp"

#include "sprites.hpp"

#include "oledPage.hpp"

#include "OLED.hpp"

namespace oled_modes
{
    // page for basic sensor readings
    class Sensors : public Page
    {
        public:

        Sensors()
        {
            
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {
            uint16_t tof_readings[5];

            float motor_speed[2];

            float yaw;

            // read sensors data 
            shared::mods.sensors->Lock();

            const sensors::Readings& readings = shared::mods.sensors->read();

            for(uint8_t i=0;i<5;++i)
            {
                tof_readings[i] = readings.distances[i];
            }

            motor_speed[0] = readings.motorSpeed[0];
            motor_speed[1] = readings.motorSpeed[1];

            yaw = readings.yaw;

            shared::mods.sensors->Unlock();

            // display data

            screen.setFont(ssd1306xled_font6x8);

            // display distance data
            char msg_buff[32];

            itoa(tof_readings[0],msg_buff,10);

            screen.drawText(0,24,1,msg_buff);

            memset(msg_buff,0,32);

            // itoa(tof_readings[1],msg_buff,10);

            // screen.drawText(8,16,1,msg_buff);

            // memset(msg_buff,0,32);

            // itoa(tof_readings[2],msg_buff,10);

            // screen.drawText(16,8,1,msg_buff);

            // memset(msg_buff,0,32);

            // itoa(tof_readings[3],msg_buff,10);

            // screen.drawText(24,16,1,msg_buff);

            // memset(msg_buff,0,32);

            // itoa(tof_readings[4],msg_buff,10);

            // screen.drawText(32,20,1,msg_buff);

            // memset(msg_buff,0,32);



            // ftoa(yaw,msg_buff,1);

            // memset(msg_buff,0,32);

            if( mode )
            {
                return 1;
            }

            return 0;
        }

    };

}