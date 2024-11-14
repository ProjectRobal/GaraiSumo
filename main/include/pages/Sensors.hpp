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

        uint8_t subpage;

        void clear()
        {
            this->subpage = 0;
        }

        void tof_page(oled::OLED& screen)
        {

            uint16_t tof_readings[5];

            // read sensors data 
            shared::mods.sensors->Lock();

            const sensors::Readings& readings = shared::mods.sensors->read();

            for(uint8_t i=0;i<5;++i)
            {
                tof_readings[i] = readings.distances[i];
            }

            shared::mods.sensors->Unlock();

            // display data

            screen.setFont(ssd1306xled_font6x8);

            screen.drawText(48,8,1,"TOFs");

            // display distance data
            char msg_buff[32];

            itoa(tof_readings[0],msg_buff,10);

            screen.drawText(0,24,1,msg_buff);

            memset(msg_buff,0,32);

            itoa(tof_readings[1],msg_buff,10);

            screen.drawText(24,16,1,msg_buff);

            memset(msg_buff,0,32);

            itoa(tof_readings[2],msg_buff,10);

            screen.drawText(48,8,1,msg_buff);

            memset(msg_buff,0,32);

            itoa(tof_readings[3],msg_buff,10);

            screen.drawText(72,16,1,msg_buff);

            memset(msg_buff,0,32);

            itoa(tof_readings[4],msg_buff,10);

            screen.drawText(96,24,1,msg_buff);

            memset(msg_buff,0,32);


        }

        void move_page(oled::OLED& screen)
        {

            float motor_speed[2];

            float yaw;

            // read sensors data 
            shared::mods.sensors->Lock();

            const sensors::Readings& readings = shared::mods.sensors->read();

            motor_speed[0] = readings.motorSpeed[0];
            motor_speed[1] = readings.motorSpeed[1];

            yaw = readings.yaw;

            shared::mods.sensors->Unlock();

            // display yaw value
            char msg_buff[32];

            screen.drawText(0,24,1,"Yaw: ");

            itoa(yaw,msg_buff,10);

            screen.drawText(0,24,1,msg_buff);

            memset(msg_buff,0,32);

            // display speed values

            screen.drawText(12,16,1,"Motor Speed: ");

            screen.drawText(6,8,1,"Left: ");

            itoa(motor_speed[0],msg_buff,10);

            screen.drawText(6,0,1,msg_buff);

            memset(msg_buff,0,32);

            screen.drawText(36,8,1,"Right: ");

            itoa(motor_speed[1],msg_buff,10);

            screen.drawText(36,0,1,msg_buff);

            memset(msg_buff,0,32);
        }

        public:

        Sensors()
        {
            this->subpage = 0;
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {
            
            if( this->subpage )
            {
                this->tof_page(screen);
            }
            else
            {
                this->move_page(screen);   
            }
            
            if( fun )
            {
                this->subpage ++ ;

                this->subpage = this->subpage - static_cast<uint8_t>(this->subpage/2) * 2;
            }

            if( mode )
            {
                this->clear();

                return 1;
            }

            return 0;
        }

    };

}