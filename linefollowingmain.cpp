//***************************************************************************************
// File: main.cpp
//***************************************************************************************

//----- INCLUDES ------------------------------------------------------------------------
#include "stm32f4xx.h"
#include "analog_in.h"
#include "battery_leds.h"
#include "robot_settings.h"
#include "systick_timer.h"
#include "tb6612fng.h"
#include "user_pb.h"
#include "green_leds.h"
#include "derivative_filter.h"
#include "encoder.h"

//----- GLOBAL OBJECTS ------------------------------------------------------------------
// the global object "systick_timer" is defined in systick_timer.cpp
TB6612FNG hbridge;

//----- GLOBAL VARIABLES ----------------------------------------------------------------
float left_motor_current, right_motor_current;
float left_duty = 0.0f, right_duty = 0.0f;
float left_distance = 0.0f, right_distance = 0.0f;
float left_speed, right_speed, speed_command;
static float error = 0.0;
float qtr_volts[9];

enum {
    forward,
    reverse,
} drive_mode;

float kp = 3.8;
float kd = 0.01;
float ki = 0.4;
float x_dist = 0.0f;
float d_speed;
float differential_speed = 0.1f;
uint8_t stop = 0;

//----- INTERNAL FUNCTION PROTOTYPES ----------------------------------------------------
void handle_battery_leds(float batt_voltage, double time);
float right_PID(float rerror, float DT);
float left_PID(float rerror, float DT);
void leds_light(float qtr_volts[9]);

//***************************************************************************************
//***************************************************************************************
int main(void)
{
    float avg_distance;
    const float QTR_SENSOR_DISTANCE = 0.009525f;
    speed_command = 0.0f;
    hbridge.setUpdateDuty(0.95);             // These two things are needed so that the
    TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);  // timer updates and current sensing happens

    systick_timer.start();

    UserPushButton top_push_button(USER_PB_TOP);
    drive_mode = forward;
    while (!top_push_button.activated());

    while(true)
    {
        speed_command=1.0f;
        if (stop > 0)
        {
            speed_command = 0.0f;
            while(!top_push_button.activated())
            {
                stop = 0;
            }
        }

        }
    }


//***************************************************************************************
// This function is called in an interrupt context at a freq = systick_timer.frequency()
//***************************************************************************************
void systick_callback(void)
{
    const float DT = systick_timer.period();
    static DerivativeFilter left_derivative(DT, 50.0, 0.707);
    static DerivativeFilter right_derivative(DT, 50.0, 0707);
    static DerivativeFilter line_derivative(DT,50.0, 0.707);
    static Encoder left_encoder(EncoderA);
    static Encoder right_encoder(EncoderB);
    static AnalogIn analog_in;
    static GreenLeds gleds_light;
    analog_in.getVoltages(qtr_volts);
    double current_time = systick_timer.ticks()*systick_timer.period();
    float raw_voltages[9];

    analog_in.getVoltages(raw_voltages);
    float batt_voltage = BATTERY_SCALE * raw_voltages[8] + BATTERY_OFFSET;

    handle_battery_leds(batt_voltage, current_time);
    leds_light(qtr_volts);

    left_motor_current = hbridge.getMotorACurrent()*MOTOR_SIGNS[0];
    right_motor_current = hbridge.getMotorBCurrent()*MOTOR_SIGNS[1];
    left_speed = left_derivative.calculate(left_distance);
    right_speed = right_derivative.calculate(right_distance);
    left_duty = left_PID(speed_command - left_speed, DT);
    right_duty = right_PID(speed_command - right_speed, DT);


    hbridge.setDutyA(left_duty*MOTOR_SIGNS[0]);
    hbridge.setDutyB(right_duty*MOTOR_SIGNS[1]);

}


//***************************************************************************************
//***************************************************************************************
void handle_battery_leds(float batt_voltage, double time)
{
    static BatteryLeds battery_leds;
    static double last_time_set = 0;
    static bool been_cleared = false;

    if (((time - last_time_set) >= 0.5) & !been_cleared)
    {
        been_cleared = true;

        battery_leds.clear(BATTERY_LED_GREEN);
        battery_leds.clear(BATTERY_LED_YELLOW);
        battery_leds.clear(BATTERY_LED_ORANGE);
        battery_leds.clear(BATTERY_LED_RED);
    }
    if ((time - last_time_set) >= 1.0)
    {
        last_time_set = time;
        been_cleared = false;

        if (batt_voltage > 7.4f)
        {
            battery_leds.set(BATTERY_LED_GREEN);
            battery_leds.set(BATTERY_LED_YELLOW);
            battery_leds.set(BATTERY_LED_ORANGE);
            battery_leds.set(BATTERY_LED_RED);
        }
        else if (batt_voltage > 6.8f)
        {
            battery_leds.clear(BATTERY_LED_GREEN);
            battery_leds.set(BATTERY_LED_YELLOW);
            battery_leds.set(BATTERY_LED_ORANGE);
            battery_leds.set(BATTERY_LED_RED);
        }
        else if (batt_voltage > 6.3f)
        {
            battery_leds.clear(BATTERY_LED_GREEN);
            battery_leds.clear(BATTERY_LED_YELLOW);
            battery_leds.set(BATTERY_LED_ORANGE);
            battery_leds.set(BATTERY_LED_RED);
        }
        else
        {
            battery_leds.clear(BATTERY_LED_GREEN);
            battery_leds.clear(BATTERY_LED_YELLOW);
            battery_leds.clear(BATTERY_LED_ORANGE);
            battery_leds.set(BATTERY_LED_RED);
        }
    }
}

float left_PID(float lerror, float DT)
{
    static float lerror_old = 0.0;
    static float ui = 0.0;
    float up, ud;
    float uimax = 0.3;
    up = kp*lerror;
    ud = kd*(lerror - lerror_old)/DT;
    ui += ki*DT*lerror;

    if (ui > uimax)
    {
        ui = uimax;
    }
    if (ui < -uimax)
    {
        ui = -uimax;
    }

    float u = up + ui + ud;
    lerror_old = lerror;

    return u;
}

float right_PID(float rerror, float DT)
{
    static float rerror_old = 0.0;
    static float ui = 0.0;
    float up, ud;
    float uimax = 0.3;
    up = kp*rerror;
    ud = kd*(rerror - rerror_old)/DT;
    ui += ki*DT*rerror;

    if (ui > uimax)
    {
        ui = uimax;
    }
    if (ui < -uimax)
    {
        ui = -uimax;
    }

    float u = up + ui + ud;
    rerror_old = rerror;

    return u;
}
void leds_light(float qtr_volts[9])
{
    uint16_t LEDS[8] = {GPIO_Pin_12,
    GPIO_Pin_13,
    GPIO_Pin_14,
    GPIO_Pin_15,
    GPIO_Pin_8,
    GPIO_Pin_9,
    GPIO_Pin_10,
    GPIO_Pin_11};
    // GREEN LED loop. Iterates through the QTR array to determine its state and the LEDs state.
    for (int i = 0; i < 9; ++i)
    {
        if ((qtr_volts[i] > 2.2f) && (i < 4) )
        {
            GPIOB->BSRRH = LEDS[i];
        }
        else if ((qtr_volts[i] > 2.2f) && (i >= 4) )
        {
            GPIOD->BSRRH = LEDS[i];
        }
        if ((qtr_volts[i] < 2.2f) && (i < 4))
        {
            GPIOB->BSRRL = LEDS[i];
        }
        else if ((qtr_volts[i] < 2.2f) && (i >= 4) )
        {
            GPIOD->BSRRL = LEDS[i];
        }
    }
}
