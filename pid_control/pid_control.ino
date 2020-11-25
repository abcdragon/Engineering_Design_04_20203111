#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.4

#define _DUTY_MIN 1060 // 위
#define _DUTY_NEU 1480 
#define _DUTY_MAX 1840 // 아래

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

#define _KP 0.004

#define A 70
#define B 320

Servo myservo;

float dist_target; // location to send the ball
float dist_raw, dist_ema;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval; 
int duty_target, duty_curr;
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
  Serial.begin(57600);
  
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);
  
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;
  
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}

void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }
  
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }
  
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    ir_distance_filtered();

  // PID control logic
    error_curr = _DIST_TARGET - dist_ema; // 목표보다 가까우면 양, 멀면 음
    pterm = error_curr;
    control = _KP * pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control * (control > 0 ? _DUTY_MAX - _DUTY_NEU : _DUTY_NEU - _DUTY_MIN);

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    duty_target = min(duty_target, _DUTY_MAX);
    duty_target = max(duty_target, _DUTY_MIN);
  }
  
  if(event_servo){
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_curr > duty_target){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}

float ir_distance(){ // return value unit: mm
  float value, volt = float(analogRead(PIN_IR));
  value = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return 300.0 / (B - A) * (value - A) + 100;
}

float ir_distance_filtered(){ // return value unit: mm
  dist_raw = ir_distance();
  return dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
}
