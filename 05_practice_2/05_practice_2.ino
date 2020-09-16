#define PIN_LED 7
unsigned int toggle, count;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //initialize serial port
  while(!Serial) {
    ; // wait for serial port to connect
  }

  toggle = count = 0;
  digitalWrite(PIN_LED, toggle); //turn on LED for 1 second.
  delay(1000);

  // toggle LED with a delay of 100ms, total 11 times. 
   while(1) {
     toggle = toggle_state(toggle);
     digitalWrite(PIN_LED, toggle);
     // 1 2 3 4 5 6 7 8 9 10 11 -> total 11 times
     if(++count>10) break;
     delay(100);
   }
}

void loop() {
  // put your main code here, to run repeatedly:

}

int toggle_state(int toggle) {
  return !toggle;
}
