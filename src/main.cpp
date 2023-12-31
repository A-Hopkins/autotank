#include <avr/io.h>
#include <util/delay.h>

#include "motors/dc_motor.h"

int main(void)
{
  int i;
  dc_motor motor_1(1);
  dc_motor motor_2(2);

  while (true)
  {
    motor_1.change_state(dc_motor::FORWARD);
    motor_2.change_state(dc_motor::FORWARD);

    // Ramp up speed on both motors
    for (i=190; i < 256; i++)
    {
      motor_1.set_speed(i);
      motor_2.set_speed(i);
      _delay_ms(100);
    }
    // Decrease speed on both motors
    for (i=256; i > 0; i--)
    {
      motor_1.set_speed(i);
      motor_2.set_speed(i);
      _delay_ms(100);
    }

    _delay_ms(1000);
    motor_1.change_state(dc_motor::REVERSE);
    motor_2.change_state(dc_motor::REVERSE);
    // Ramp up speed on both motors
    for (i=190; i < 256; i++)
    {
      motor_1.set_speed(i);
      motor_2.set_speed(i);
      _delay_ms(100);
    }
    // Decrease speed on both motors
    for (i=256; i > 0; i--)
    {
      motor_1.set_speed(i);
      motor_2.set_speed(i);
      _delay_ms(100);
    }

    motor_1.change_state(dc_motor::RELEASE);
    motor_2.change_state(dc_motor::RELEASE);
  }

  return 0;
}
