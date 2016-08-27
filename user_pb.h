#ifndef USER_PB_H_INCLUDED
#define USER_PB_H_INCLUDED

#include <cstdint>

typedef uint8_t user_button_id_t;
enum
{
    USER_PB_TOP,    // button that is closer to bluetooth
    USER_PB_BOTTOM  // button that is closer to USB port
};

class UserPushButton
{
  public: // methods

    // Constructor
    UserPushButton(user_button_id_t button_id);

    // Return true if button is being pressed.
    bool read(void);

    // Return true if button was released since last call to activated().
    // If this method is called again before the button is pressed/released
    // then it will return false.
    bool activated(void);

  private: // fields

    user_button_id_t button_id_;

    // True if button was pressed during last call to activate().
    bool button_was_pressed_;

};

#endif
