#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <NewPing.h>

#define WSENSOR 8
#define TRIG 10
#define ECHO 11
#define RELAY 12
#define LED 13

#define CALIB_SWITCH 2
#define MODE_SWITCH 3

#define LOOP_DELAY 500
#define MAX_DISTANCE 200
#define MAX_MEASURES 100
#define CONVERGENCE_TOL 0.05

#define EEPROM_DISTANCE_ADDR 0

#define OK_BLINK 100
#define CALIB_BLINK 2
#define ERR_BLINK 10

NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

bool calibration_mode = false; // TODO

int timer_counter; // LED timer counter

// ultrasonic operation vars
unsigned int distance;
const int uncertainty_margin = 3;
int trespass_count = 0;

// ultrasonic calibration vars
float tmp_full_distance = 0, full_distance = 0;
unsigned int full_measures = 0;
bool full_measure_converged = false;

// use timer interrupt to control LED blinking frequency
// http://www.hobbytronics.co.uk/arduino-timer-interrupts
// LED on == fast blink frequency
void update_led_frequency(unsigned int frequency) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  
  timer_counter = 65536 - (16000000/256/frequency);
  
  TCNT1 = timer_counter;
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts();
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer_counter;
  digitalWrite(LED, digitalRead(LED) ^ 1);
}

void setup() {
  // water sensor vs ultrasonic mode switch
  pinMode(MODE_SWITCH, INPUT);

  // water sensor input
  pinMode(WSENSOR, INPUT);

  // ultrasonic trigger calibration button
  pinMode(CALIB_SWITCH, INPUT_PULLUP);
  attachInterrupt(0, trigger_calibration, FALLING);

  // status LED
  pinMode(LED, OUTPUT);

  // relay
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
  
  // EEPROM
  full_distance = EEPROM.readFloat(EEPROM_DISTANCE_ADDR);
  // if no calibration data, status LED error, else status LED on
  if (full_distance == -1)
    update_led_frequency(ERR_BLINK);
  else
    update_led_frequency(OK_BLINK);
  
  // Serial debug. TODO: remove in release version
  // Serial.begin(9600);
}

void trigger_calibration() {
  calibration_mode = true;
}

void loop() {
  pinMode(MODE_SWITCH, OUTPUT); // TODO: remove, just for testing
  digitalWrite(MODE_SWITCH, HIGH); // TODO: remove, just for testing
  pinMode(MODE_SWITCH, INPUT); // TODO: remove, just for testing

  // mode: water sensor; if water detected
  if (digitalRead(MODE_SWITCH) == LOW && digitalRead(WSENSOR) == LOW) {
    digitalWrite(RELAY, HIGH); // disconnect relay
  }

  else if (digitalRead(MODE_SWITCH) == LOW && digitalRead(WSENSOR) == HIGH) {
    digitalWrite(RELAY, LOW);
  }

  // mode: ultrasonic operation
  else if (digitalRead(MODE_SWITCH) == HIGH && !calibration_mode) {
    full_distance = EEPROM.readFloat(EEPROM_DISTANCE_ADDR);
    
    distance = sonar.ping_cm();

    // if distance < full_distance
    if (distance < full_distance) {
      if (trespass_count < uncertainty_margin) {
        trespass_count++;
      } else {
        digitalWrite(RELAY, HIGH); // disconnect relay
      }
    } else {
      digitalWrite(RELAY, LOW);
      trespass_count = 0;
    }
  }

  // mode: ultrasonic calibration
  else if (digitalRead(MODE_SWITCH) == HIGH && calibration_mode) {
    // slow blink to indicate calibrating
    update_led_frequency(CALIB_BLINK);
    
    // if full calib is not converged
    while (!full_measure_converged && full_measures <= MAX_MEASURES) {
      // get averaged full-tank reading
      tmp_full_distance = (tmp_full_distance + long(sonar.ping_cm())) / 2;
      full_measures++;
      // if running average ~ previous, full calib is converged
      if (abs(((tmp_full_distance + full_distance) / 2.0) - full_distance) < CONVERGENCE_TOL) {
        full_measure_converged = true;
      }
      full_distance = tmp_full_distance;
      delay(LOOP_DELAY); // slow down the calibration
    }

    // calibration successful
    if (full_measure_converged) {
      // write full_distance to EEPROM
      EEPROM.writeFloat(EEPROM_DISTANCE_ADDR, full_distance);

      // status LED on
      update_led_frequency(OK_BLINK);

    // or failed to converge after MAX_MEASURES attempts
    } else if (full_measures > MAX_MEASURES) {
      // write "no calibration data" to EEPROM
      EEPROM.writeFloat(EEPROM_DISTANCE_ADDR, -1);

      // status LED error
      update_led_frequency(ERR_BLINK);
    }
    // exit calibration mode and cleanup
    calibration_mode = false;
    full_measures = 0;
    full_measure_converged = false;
  }

  delay(LOOP_DELAY);

}
