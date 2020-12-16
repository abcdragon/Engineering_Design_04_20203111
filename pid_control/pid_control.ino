#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.21

#define LENGTH 30
#define k_LENGTH 5

#define _DUTY_MIN 1210 // 위
#define _DUTY_NEU 1480
#define _DUTY_MAX 1750 // 아래

#define _SERVO_SPEED 1000 // servo speed limit (unit: degree/second)
#define _RAMPUP_TIME 360 // servo speed rampup (0 to max) time (unit: ms)

#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100

#define _KP 1
#define _KD 90
#define _KI 0.3
#define _MAX_ITERM 50

Servo myservo;

int iter;
float dist_list[LENGTH], sum, dist_raw, dist_ema, alpha;
float dist_list2[LENGTH];

float dist_target = _DIST_TARGET; // location to send the ball

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

int index = 0;
int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
int duty_chg_per_interval; // current speed (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int duty_target, duty_curr;
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
  Serial.begin(57600);
  
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);

  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;
  dist_raw = sum = dist_ema = 0.0;
  error_curr = error_prev = 0.0;
  
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

  iterm = dterm = pterm = 0.0;
  
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.

  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
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
    dist_raw = ir_distance_filter();
    
  // PID control logic
    error_curr = dist_target - dist_raw; // 목표보다 가까우면 양, 멀면 음
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    
    iterm = max(iterm, -_MAX_ITERM);
    iterm = min(iterm, _MAX_ITERM);
   
    control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    duty_target = min(duty_target, _DUTY_MAX);
    duty_target = max(duty_target, _DUTY_MIN);
  
    error_prev = error_curr;
  }
  
  if(event_servo){
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      if(duty_chg_per_interval < duty_chg_max) {
        duty_chg_per_interval += duty_chg_adjust;
        if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else if(duty_target < duty_curr) {
      if(duty_chg_per_interval > -duty_chg_max) {
        duty_chg_per_interval -= duty_chg_adjust;
        if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    else {
      duty_chg_per_interval = 0;
    }

    duty_curr = max(duty_curr, _DUTY_MIN);
    duty_curr = min(duty_curr, _DUTY_MAX);

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float real[] = {0.0, 70.0, 122.0, 175.0, 204.0, 225.0, 249.0, 280.0, 295.0};
float ideal[] = {0.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 415.0};

int search(float target){
  int mid, low = 0, high = 8;
  while(low < high){
    mid = (low + high) / 2;
    if(real[mid] >= target) high = mid;
    else low = mid + 1;
  }
  return high;
}

float mapping(void){
  dist_raw = ir_distance();
  
  int i = search(dist_raw);
  return ideal[i - 1] + (ideal[i] - ideal[i - 1]) / (real[i] - real[i - 1]) * (dist_raw - real[i - 1]);  
}

float ir_distance_filter() {
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = mapping();
    sum += dist_list[iter];
    iter++;
  }
  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum / (LENGTH - 2 * k_LENGTH);

  float tmp = alpha * dist_cali + (1 - alpha) * dist_ema;
  if(abs(dist_ema - tmp) >= 0.1){
    dist_ema = tmp;
  } else {
    dist_ema = min(dist_ema, tmp);
  }
  
  return dist_ema;
}
