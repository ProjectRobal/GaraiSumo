#include "Tactics_list.hpp"

#include "DoNothing.hpp"
#include "FullSpeed.hpp"


tactics::Tactics* tactics_list[TACTICS_COUNT] = {
    new tactics::DoNothing(),
    new tactics::FullSpeed()
};
