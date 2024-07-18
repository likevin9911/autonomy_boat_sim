#include "HThrustSM.hpp"

namespace usv_vrx {

HThrustSM::HThrustSM(float reconfig_duration): reconfig_duration_(fabs(reconfig_duration))
{
  HThrustSM::resetStateMachine();
}

void HThrustSM::resetStateMachine()
{
  reconfig_time_target_   = 0;
  state_                  = THRUST_TRAVERSE;
}

void HThrustSM::checkTimers()
{
  if (state_ == THRUST_RECONFIG_STRAIGHT && ros::Time::now().toSec() > reconfig_time_target_)
  {
    state_ = THRUST_TRAVERSE; // Normal hull thruster movement
  }
}

void HThrustSM::reconfigure(HThrustSM::THRUST_STATE reconfiguration)
{
  if (reconfiguration == THRUST_RECONFIG_STRAIGHT)
  {
    state_ = reconfiguration; // Signal to begin angling thrusters
    reconfig_time_target_ = ros::Time::now().toSec() + reconfig_duration_;
  }
}

void HThrustSM::updateState(bool keep_station)
{
  switch (state_) 
  {
    case THRUST_TRAVERSE:
      if (keep_station)
        state_ = THRUST_STATION;
      break;

    case THRUST_STATION:
      if (!keep_station)
        HThrustSM::reconfigure(THRUST_RECONFIG_STRAIGHT);
      break;
  }

  HThrustSM::checkTimers();
}

HThrustSM::THRUST_STATE HThrustSM::getState()
{
  HThrustSM::checkTimers();
  return state_;
}

}
