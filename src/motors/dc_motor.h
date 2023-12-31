#include <avr/io.h>

#define MOTOR34_8KHZ _BV(CS01)           // divide by 8
#define DC_MOTOR_PWM_RATE   MOTOR34_8KHZ // PWM rate for DC motors

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Size of shift register for motor pins
#define SHIFT_REG_SZ 8

class dc_motor
{
public:
  enum state
  {
    RELEASE = 0,
    FORWARD = 1,
    REVERSE = 2
  };

  dc_motor(uint8_t motor_num);
  void change_state(state new_state);
  void set_speed(uint8_t new_speed);

private:
  uint8_t motor_num;
  state current_state;
};
