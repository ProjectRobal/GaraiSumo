#include "Tactics_list.hpp"

#include "DoNothing.hpp"
#include "FullSpeed.hpp"


tactics::Tactics* select_tactics(Tactics_type tactic)
{
    switch(tactic)
    {
        case Tactics_type::FullSpeed:

            return new tactics::FullSpeed();

        break;

        case Tactics_type::DoNothing:
        default:

            return new tactics::DoNothing();
    }
}