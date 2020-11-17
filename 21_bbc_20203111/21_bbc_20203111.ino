#include<Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DUTY_MIN 800
#define _DUTY_MAX 1900

#define _SERVO_SPEED 200
#define INTERVAL 20

unsigned long last_sampling_time;
int duty_chg_per_interval;
int a, b; // unit: mm

const float alpha = 0.3;
float dist_ema;

Servo servo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 70, b = 350, dist_ema = 0.0;
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.);

  servo.attach(PIN_SERVO);
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  float ema = EMA_filter(dist_cali);
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(ema);

  if(ema > 255){
    servo.writeMicroseconds(_DUTY_MIN);
  }

  else {
    servo.writeMicroseconds(_DUTY_MAX);
  }
  
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
}

float EMA_filter(float dist){
  dist_ema = alpha * dist + (1 - alpha) * dist_ema;
  return dist_ema;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
