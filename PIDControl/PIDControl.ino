#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [3110] 9번핀 LED 연결  
#define PIN_SERVO 10 // [3110] 10번핀 서보 연결
#define PIN_IR A0 //[3104] 적외선 거리센서 PIN - Analog0 정의 

// Framework setting
#define _DIST_TARGET 255 //[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값
#define _DIST_MAX 450 //[3117] 거리 최대값

// Distance sensor
#define _DIST_ALPHA 0.3  //[3099] EMA 필터링을 위한 alpha 값
               // [3108] 0~1 사이의 값
#define SEQ_SIZE 14
#define SAMPLE_SIZE 50
#define ERROR_NUM 10

// Servo range
#define _DUTY_MIN 800     //[3100] 최저 서보 위치
#define _DUTY_NEU 1500 //[3100] 중립 서보 위치
#define _DUTY_MAX 2450     //[3100] 최대 서보 위치


// Servo speed control
#define _SERVO_ANGLE 30.0 
#define _SERVO_SPEED 100.0 

// Event periods
#define _INTERVAL_DIST 20 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// personal best : 4.7 0.005 175


// PID parameters
#define _KP 4.7  //0.0028
#define _KI 0.005
#define _KD 177.5

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //[1928] 측정된 값과 ema 필터를 적용한 값
float dist_sample[SAMPLE_SIZE];
// sensor values
float x[SEQ_SIZE] = {70.95, 91.93, 117.92, 137.06, 157.88, 171.98, 193.68, 211.40, 233.07, 252.89, 273.69, 293.09, 312.82, 327.44};
// real values
float y[SEQ_SIZE] = {100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400, 425};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; // [3116] 주기 당 서보 duty값 변화량
int duty_target, duty_curr; //[1928] 목표 위치와 현재 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수
// [3099]

void setup() {
// initialize GPIO pins for LED and attach servo 
myservo.attach(PIN_SERVO); // attach servo
pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

// initialize global variables
event_dist = event_servo = event_serial = false;
dist_target = _DIST_TARGET;
error_prev = error_curr = 0;
iterm = 0;

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);
duty_curr = _DUTY_NEU;
delay(450);
// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (_SERVO_ANGLE * 2) ) * (_INTERVAL_SERVO / 1000.0); 
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

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();
    // [3099] dist_ema?

  // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;
    iterm += _KI * error_curr;
    if(error_prev == 0.0)
      dterm = 0;
    else
      dterm = _KD * (error_curr - error_prev);
    control = pterm + iterm + dterm;

    error_prev = error_curr;

  // duty_target = f(duty_neutral, control)
  //duty_target = ((control>0)?(_DUTY_MAX - _DUTY_NEU)*_SERVO_ANGLE / 180.0:(_DUTY_NEU - _DUTY_MIN) * _SERVO_ANGLE / 180.0) * control;

  duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MAX - _DUTY_NEU):(_DUTY_NEU - _DUTY_MIN)) / (_DUTY_MAX - _DUTY_NEU);
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    else if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    // [3099]
  }
  
  if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }   

  if(event_serial) {
    event_serial = false; //[3117] // 이거 맞나요? // 저도 이렇게 했어요
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
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance_filtered(void){ // return value unit: mm
  int i, j, k;
  float value, sum = 0;

  // get samples in a ascending order array
  for(i=0; i< SAMPLE_SIZE; i++){
    value = ir_distance_sequence();
    for(j=0; j<i; j++){
      if(dist_sample[j] > value)
        break;
    }
    for(k=i; k > j; k--){
      dist_sample[k] = dist_sample[k - 1];
    }
    dist_sample[j] = value;
  }

  // skip ERROR_NUM: inspired by #ir_filter_김태완
  for(i=ERROR_NUM; i<SAMPLE_SIZE - ERROR_NUM; i++){
    sum += dist_sample[i];
  }
  dist_raw = sum / (SAMPLE_SIZE - 2 * ERROR_NUM);
  
  dist_ema = _DIST_ALPHA * (sum / (SAMPLE_SIZE - 2 * ERROR_NUM)) + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}

float ir_distance_sequence(void){
  int i;
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;

  /*
  for(i=0; i<SEQ_SIZE; i++){
    if(value >= x[i]){
      real_value = (y[i+1] - y[i]) / (x[i+1] - x[i] ) * (value - x[i]) + y[i];
      break;
    }
  }
  if(i==SEQ_SIZE) real_value = _DIST_TARGET;
  */
  
  int s = 0, e = SEQ_SIZE - 1, m;
  
  // binary search
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]){
      e = m - 1;
    }
    else if(value > x[m+1]){
      s = m + 1;
    }
    else{
      break;
    }
  }
  // out of sequence range: set value to neutral position
  // 센서 전원 Off 시, 센서 분리시 등 
  if(s > e){
    if(s == 0) real_value = _DIST_MIN; 
    else if(e == SEQ_SIZE -1) real_value = _DIST_MAX;
    else real_value = _DIST_TARGET;
  }
  real_value = (y[m+1] - y[m]) / (x[m+1] - x[m] ) * (value - x[m]) + y[m];
  
  // calculate real values

  return real_value;
}



// [3099] 구간별 센서값 보정 코드 구현
