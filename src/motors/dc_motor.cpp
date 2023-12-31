#include "dc_motor.h"

class motor_controller
{
public:
  motor_controller(void) { }
  static uint8_t latch_state;

  void latch_tx(void)
  {
    uint8_t i;

    // latch_tx
    PORTB &= ~_BV(PB4); // Pin 12 MOTORLATCH
    PORTB &= ~_BV(PB0); // Pin 8 MOTORDATA

    // Loop through shift register
    // Q0 Q1 Q2 Q3 Q4 Q5 Q6 Q7
    // 4a 2a 1a 1b 2b 3a 4b 3a

    for (i = 0; i < SHIFT_REG_SZ; i++)
    {
        PORTD &= ~_BV(PD4); // Pin 4 MOTORCLK

        if (latch_state & _BV((SHIFT_REG_SZ - 1) - i))
        {
            PORTB |= _BV(PB0); // Pin 8 MOTORDATA
        }
        else
        {
            PORTB &= ~_BV(PB0); // Pin 8 MOTORDATA
        }

        PORTD |= _BV(PD4); // Pin 4 MOTORCLK
    }

    PORTB |= _BV(PB4); // Pin 12 MOTORLATCH
  }

  void enable(void)
  {
    // Setup the latch
    // The following pins are in use if any DC/steppers are used
    // Digital pin 4, 7, 8 and 12 are used to drive the DC/Stepper motors
    // via the 74HC595 serial-to-parallel latch and Digital pin 3 is used for the PWM

    DDRB |= _BV(PB4) | _BV(PB0) | _BV(PB3); // Pin 12 MOTORLATCH, 8 MOTORDATA, Pin 3 PWM
    DDRD |= _BV(PD7) | _BV(PD4) | _BV(PD3); // Pin 7 MOTORENABLE, 4 MOTORCLK, Pin 3 PWM

    latch_tx();

    // enable the chip outputs!
    PORTD &= ~_BV(PD7); // Digial Write LOW to Pin 7
  }
};

static motor_controller mc; // single motor controller that acts on the latch state
uint8_t motor_controller::latch_state = 0;

// Initialize the PWM for motor 1 pins
inline void init_pwm1(void)
{
  TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
  TCCR2B = DC_MOTOR_PWM_RATE & 0xF;
  OCR2A = 0; // set speed
}

// Initialize the PWM for motor 2 pins
inline void init_pwm2(void)
{
  TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
  TCCR2B = DC_MOTOR_PWM_RATE & 0xF;
  OCR2B = 0; // set speed
}

// Initialize the PWM for motor 3 pins
inline void init_pwm3(void)
{
  // use PWM from timer0A / PD6 (pin 6)
  TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
  TCCR0B = DC_MOTOR_PWM_RATE & 0xF;
  OCR0A = 0; // set speed

  DDRD |= _BV(PD6);
}

// Initialize the PWM for motor 4 pins
inline void init_pwm4(void)
{
  // use PWM from timer0B / PD5 (pin 5)
  TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
  TCCR0B = DC_MOTOR_PWM_RATE & 0xF;
  OCR0B = 0; // set speed
  DDRD |= _BV(PD5);
}

inline void set_pwm1(uint8_t new_speed)
{
  // use PWM from timer2A on PB3
  OCR2A = new_speed;
}

inline void set_pwm2(uint8_t new_speed)
{
  // use PWM from timer2B on PB3
  OCR2B = new_speed;
}

inline void set_pwm3(uint8_t new_speed)
{
  // use PWM from timer0A on PD3
  OCR0A = new_speed;
}

inline void set_pwm4(uint8_t new_speed)
{
  // use PWM from timer0B on PD3
  OCR0B = new_speed;
}

dc_motor::dc_motor(uint8_t motor_num)
{
  dc_motor::motor_num = motor_num;
  dc_motor::current_state = RELEASE;

  mc.enable();

  // M1, M2, M3, M4 connections on the L239D Motor Shield
  switch (motor_num)
  {
    // set both motor pins to 0 and transmit the latch state
    case 1:
      mc.latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B);
      mc.latch_tx();
      init_pwm1();
      break;
    case 2:
      mc.latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR4_B);
      mc.latch_tx();
      init_pwm2();
      break;
    case 3:
      mc.latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR4_B);
      mc.latch_tx();
      init_pwm3();
      break;
    case 4:
      mc.latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B);
      mc.latch_tx();
      init_pwm4();
      break;
    
    default:
      return;
  }
}

void dc_motor::change_state(state new_state)
{
  uint8_t motor_pin_a, motor_pin_b;

  switch (motor_num)
  {
    case 1:
      motor_pin_a = MOTOR1_A;
      motor_pin_b = MOTOR1_B;
      break;
    case 2:
      motor_pin_a = MOTOR2_A;
      motor_pin_b = MOTOR2_B;
      break;
    case 3:
      motor_pin_a = MOTOR3_A;
      motor_pin_b = MOTOR3_B;
      break;
    case 4:
      motor_pin_a = MOTOR4_A;
      motor_pin_b = MOTOR4_B;
      break;
    default:
      return;
  }

  switch (new_state)
  {
    case RELEASE:
      mc.latch_state &= ~_BV(motor_pin_a);
      mc.latch_state &= ~_BV(motor_pin_b);
      break;
    case FORWARD:
      mc.latch_state |= ~_BV(motor_pin_a);
      mc.latch_state &= ~_BV(motor_pin_b);
      break;
    case REVERSE:
      mc.latch_state &= ~_BV(motor_pin_a);
      mc.latch_state |= ~_BV(motor_pin_b);
      break;
    default:
      break;
  }
  mc.latch_tx();
  current_state = new_state;
}

void dc_motor::set_speed(uint8_t new_speed)
{
  switch (motor_num)
  {
    case 1:
      set_pwm1(new_speed);
      break;
    case 2:
      set_pwm2(new_speed);
      break;
    case 3:
      set_pwm3(new_speed);
      break;
    case 4:
      set_pwm4(new_speed);
      break;

    default:
      return;
  }
}
