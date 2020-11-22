#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0


#define _DUTY_MIN 875 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2300 // servo full counterclockwise position (180 degree)
#define ALPHA 0.3

int a = 87, b = 300;
float raw_dist, dist_cali, dist_ema;
Servo myservo;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,1);

  Serial.begin(57600);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  //while(1){}
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  // put your main code here, to run repeatedly
  raw_dist = ir_distance();
  dist_cali = 300.0 / (b - a) * (raw_dist - a) + 100;
  dist_ema = ALPHA * dist_cali + (1 - ALPHA) * dist_ema;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);
  
  // there is low chance that two float equals, but for handling exceptional case
  if(dist_ema == 255){
    myservo.writeMicroseconds(_DUTY_NEU);
  }
  else if(dist_ema< 255){
    digitalWrite(PIN_LED,255);
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  else{
    digitalWrite(PIN_LED, 0);
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  delay(20);
}
