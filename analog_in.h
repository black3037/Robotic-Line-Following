#ifndef ANALOG_IN_H_INCLUDED
#define ANALOG_IN_H_INCLUDED

// Includes
#include <cstdint>

// Setup ADC1 to scan the signals on pins below continuously using DMA to write the
// values to the a buffer.  The channels and the order in which they are scanned
// and placed in the buffer are listed below.
// Runs at 8130Hz for each 9 channel scan
// PIN  EEVA-SIGNAL       ADCCHANNEL
// PA1    QTR_1            ADC1_IN1
// PA2    QTR_2            ADC1_IN2
// PA3    QTR_3            ADC1_IN3
// PA4    QTR_4            ADC1_IN4
// PA5    QTR_5            ADC1_IN5
// PA6    QTR_6            ADC1_IN6
// PA7    QTR_7            ADC1_IN7
// PC4    QTR_8            ADC1_IN14
// PC5    BATT_SENS        ADC1_IN15
class AnalogIn
{
  public: // methods

    // Constructor - this sets up the ADC hardware
    AnalogIn(void);

    // Fill the input array with the most recent voltages read in from the ADCs.
    void getVoltages(float voltages[9]);

  private: // methods

    // Not usually called.  Just used for testing speed.
    void setupInterrupt(void);

  private: // fields

    // The raw (i.e. unscaled) data returned from the ADC's.
    volatile uint16_t adc_raw_values_[9];
};

#endif
