#pragma once

#include "Tactics.hpp"

#include "Tactics_type.hpp"
#include "SensorReader.hpp"

#include "shared.hpp"

namespace tactics
{
    class FullSpeed : public Tactics
    {
        float target_angel;

        bool sweep;


        public:

        FullSpeed()
        : Tactics()
        {

            this->target_angel = 0.f;
            this->sweep = false;

        }

        void loop(){

            float speed = 0.f;

            shared::mods.sensors->Lock();

            const sensors::Readings& readings = shared::mods.sensors->read();

            uint8_t id = 0;

            uint8_t target_id = 100;

            uint16_t min_dist = 8190;

            for( uint16_t distance : readings.distances )
            {
                if( ( distance < min_dist ) && ( distance < 90 ) )
                {
                    min_dist = distance;

                    target_id = id;
                }

                id++;
            }

            if( target_id < 5 )
            {
                // if( angle < 0.f )
                // {
                //     angle = 360.f + angle;
                // }

                this->target_angel = readings.yaw + ( target_id - 2 )*20.f;
            }
            else
            {
                speed = 0.f;

                if( this->sweep )
                {
                    this->target_angel = 360.f;
                }
                else
                {
                    this->target_angel = 0.f;
                }

                if( abs( readings.yaw - this->sweep*360.f ) <= 3.f )
                {

                    this->sweep = !this->sweep;
                
                }


            }

            shared::mods.driver->setTargetSpeed(speed);
            shared::mods.driver->setTargetAngel(this->target_angel);

            shared::mods.sensors->Unlock();

        }

        Tactics_type type()
        {
            return Tactics_type::FullSpeed;
        }

        ~FullSpeed()
        {}
    };
}