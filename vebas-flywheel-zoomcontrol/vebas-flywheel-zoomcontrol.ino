#include <Arduino.h>

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
  #define _ATTINY_
  const int PWM_INPUT_PIN = PB0;   // attiny PB0->leg 5
  const int PWM_INPUT_IRQ_PIN = PCINT0; // Int 0 for pin 2 // attiny
  const int PWM_OUTPUT_PIN = PB4;  // attiny PB3 -> leg 2
  const int PWM_OUTPUT_REVERSE_PIN = PB3;  // attiny PB4 -> leg 3
  const float PWM_FREQUENCY = 504; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  const int LED_PIN = PB1;
// #define DEBUG
#elif defined(ARDUINO_TEENSY31)
#define _TEENSY_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  const float PWM_FREQUENCY = 488; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#else // other platforms
  #define _ARUINO_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  const int PWM_OUTPUT_REVERSE_PIN = PB3;  // attiny PB4 -> leg 3
  const float PWM_FREQUENCY = 488; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#endif

// valid PWM range definition
const int PWM_LOW = 1100;    /* in us */
const int PWM_CENTER = 1500; /* in us */
const int PWM_HIGH = 1900;   /* in us */

// define hysteresis for center value
const int PWM_CTL_HYSTERESIS = 50; /* in us */

// delay on each iteration
const int LOOP_DELAY_IN_MS = 20;      /* in ms */
const int SHORT_PRESS_TIME_IN_MS = 200; /* in ms */
const int LONG_PRESS_TIME_IN_MS = 1000; /* in ms */
// single step pwm delta
const int PWM_STEPSIZE = 50; /* in us */

// pwm output
int pwm_out_duty_in_us = PWM_CENTER; /* in us */
long mode_last_changed = 0;
long step_last_changed = 0;

// last pwm duty time
volatile unsigned long pwm_input_duty_in_us = 0;
volatile unsigned long pwm_start_time = 0;

#ifdef DEBUG
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
#include <DigiKeyboard.h>
#define DEBUG_INIT // DigiKeyboard.begin()
#define DEBUG_PRINT(x) DigiKeyboard.print(x)
#define DEBUG_PRINTLN(x) DigiKeyboard.println(x)
#define DEBUG_FLUSH // DigiKeyboard.refresh();
#else
#define DEBUG_INIT Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_FLUSH // DigiKeyboard.refresh();
#endif
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

unsigned long last_edge_time = 0;
// interrupt routing to calculate the duty length in us
void handle_pwm_input_interrupt() {

  if (digitalRead(PWM_INPUT_PIN)) {
    pwm_start_time = micros();
    last_edge_time = millis();
  } else {
    pwm_input_duty_in_us = micros() - pwm_start_time;
  }
}

#ifdef _ATTINY_
// IRQ subroutine for arv irq handling
ISR(PCINT0_vect)
{
    handle_pwm_input_interrupt();
}
#endif

void setup() {

#ifdef DEBUG
  DEBUG_INIT;
#endif

  pinMode(PWM_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_REVERSE_PIN, OUTPUT);

  mode_last_changed = millis();
#ifdef _ARDUINO_
  attachInterrupt(PWM_INPUT_IRQ_PIN, handle_pwm_input_interrupt, CHANGE);
#elif defined _ATTINY_
  cli();

  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PWM_INPUT_IRQ_PIN); // set int bit

  sei();
#endif

  DEBUG_PRINTLN("init completed");
}

// return the pwm value based on the irq handler and reset it to 0
int get_pwm_duty_time_in_us_and_reset() {

  // reset when last edge is long ago...
  if (last_edge_time + 1000 < millis())
  {
    // pwm_out_duty_in_us = 0;
  }

  if (pwm_input_duty_in_us > 0)
  {
    DEBUG_PRINT("Last time ");
    DEBUG_PRINTLN(pwm_input_duty_in_us);
    return pwm_input_duty_in_us;
  }

  return 0;
}


int sign(int x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}


void do_soft_pwm(int pin, int high_time_in_us)
{
  unsigned long start = micros();

  if (high_time_in_us < PWM_LOW || high_time_in_us > PWM_HIGH)
  {
    return;
  }

  digitalWrite(pin, HIGH);
  while(start + high_time_in_us > micros())
  {
    // do nothing
  }
  digitalWrite(pin, LOW);
}

// return [-1,0,1] depending on the pwm input value
int current_input_mode() {
  int pwm_in_us = get_pwm_duty_time_in_us_and_reset();

  int pwm_deflection = pwm_in_us - PWM_CENTER;

  if (abs(pwm_deflection) < PWM_CTL_HYSTERESIS) {
    return 0;
  }

  if (pwm_deflection > 0) {
    return 1;
  }

  return -1;
}

void update_pwm_output(int pwm)
{
  DEBUG_PRINT("pwm: ");
  DEBUG_PRINTLN(pwm);

  int pwm_r = int(pwm / PWM_STEPSIZE)*PWM_STEPSIZE;

  // pwm_r = 1300;

  do_soft_pwm(PWM_OUTPUT_PIN, pwm_r);
  // calculate the reverse pwm_r value
  int delta_pwm = pwm_r - PWM_LOW;
  int pwm_inv = PWM_HIGH - delta_pwm;

  do_soft_pwm(PWM_OUTPUT_REVERSE_PIN, pwm_inv);

}

unsigned long mode_changed_time = 0;
const int MODE_CHANGE_TIMEDELTA_IN_MS = 100;

void loop() {

  int mode_in = current_input_mode();

  DEBUG_PRINT("mode_in: ");
  DEBUG_PRINTLN(mode_in);

  if (mode_in != 0) {

  } else {
    mode_changed_time = millis();
    mode_last_changed = millis();
  }

  if (mode_changed_time + MODE_CHANGE_TIMEDELTA_IN_MS < millis() ) {
    long delta_t = millis() - mode_last_changed;
    long delta_t_step = millis() - step_last_changed;

    int pwm_offset = mode_in * PWM_STEPSIZE;
    // int pwm_offset = mode_in * (0.1 * delta_t);
    DEBUG_PRINT("pwm_offset: ");
    DEBUG_PRINTLN(pwm_offset);

    if (delta_t_step > SHORT_PRESS_TIME_IN_MS) {
      pwm_out_duty_in_us =
          min(max(pwm_out_duty_in_us + pwm_offset, PWM_LOW), PWM_HIGH);
      step_last_changed = millis();
    }

    // long press
    if (delta_t > LONG_PRESS_TIME_IN_MS) {
      if (mode_in < 0) {
        pwm_out_duty_in_us = PWM_LOW;

      } else {
        pwm_out_duty_in_us = PWM_HIGH;
      }
    }

    DEBUG_PRINT("pwm_out_suty: ");
    DEBUG_PRINTLN(pwm_out_duty_in_us);

  }

  update_pwm_output(pwm_out_duty_in_us);

  DEBUG_PRINTLN("####");
  delay(LOOP_DELAY_IN_MS);
}
