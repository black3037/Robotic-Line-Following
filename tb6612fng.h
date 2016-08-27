#ifndef TB6612FNG_H_INCLUDED
#define TB6612FNG_H_INCLUDED

// Includes
#include "digital_out.h"
#include "pwm_out_advanced_timer.h"

// Driver for HBridge chip for controlling motors.
class TB6612FNG
{
  public: // methods

    // Constructor
    TB6612FNG(void);

    // Set duty cycle for each motor from -1 to 1.
    void setDutyA(float duty);
    void setDutyB(float duty);

  private: // fields

    PwmOutAdvancedTimer pwm_timer_;

};

#endif
