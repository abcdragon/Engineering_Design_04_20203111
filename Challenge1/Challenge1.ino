#define LED_PIN 7

int _period, _on, _off;
void setup() {
  _period = _on = _off = 0;
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("창업연계공학설계 도전과제1");
  //Serial.println("기본은 10ms 이고, 입력으로 주기(period)를 바꿀 수 있습니다.");
}

// period's unit : us ==> 백만 분의 1초
//     (참고)     : ms ==> 천분의 1초

// duty cycle에 대한 식은 다음과 같다.
// duty cycle(백분율) = pulse width / period * 100
// pulse width가 LED를 켜고 있는 시간이므로
// pulse width에 대한 식으로 변환하면
// pulse width = duty cycle / 100 * period
// 이 때 pulse width 는 정수형이어야 하므로 식의 순서를 변형하면
// pulse width = (period / 100) * duty cycle 이 된다.
// 이 때 period는 고정으로 100을 나누어야 한다.
// 따라서, set_period 함수에선 ms -> us 단위로 변환할 때 1000을 곱해야 하지만, 
// 계산의 편의성을 위해 미리 100을 나누는 의미로 10을 곱한다.


void set_period(float period){
  _period = period * 10;
}

void set_duty(int duty){
  _on = duty * _period;
  //Serial.print("on : ");
  //Serial.print(_on);
  
  _off = (100 - duty) * _period;
  //Serial.print(", off : ");
  //Serial.println(_off);
}

int count = 1000;
auto t = millis();
void loop() {
  if(Serial.available() > 0){
    float input = Serial.parseFloat();
    set_period(input);

    count = _period == 100 ? 2000 : 1000;
    t = millis();

    int chk = 0;
    while(t + count >= millis()){
       for(int i = 0; i < 101; i++){
        set_duty(i);
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(_on);
      
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(_off);
      }
      
      for(int i = 99; i >= 0; i--){
        set_duty(i);
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(_on);
      
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(_off);
      }

      chk++; 
    }

    Serial.println(chk);
    Serial.println(millis() - t);
  }
}
