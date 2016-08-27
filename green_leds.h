#ifndef GREEN_LEDS_H_INCLUDED
#define GREEN_LEDS_H_INCLUDED

// Includes
#include <cstdint>

// Interface to the eight green LEDs on the EEVA board
class GreenLeds
{
  public: // methods

    // Constructor - sets up green LED output pins
    explicit GreenLeds(void);

    // Update all 8 LEDs to the specified bit pattern.
    // For example a pattern of 10001001 would turn on 3 LEDs.
    void set(uint8_t pattern);

};

#endif
