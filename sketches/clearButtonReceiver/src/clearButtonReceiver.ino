/*

  Kevyn McPhail
  Deeplocal
  FOB Receiving module code
  
  If Button A is pressed the the arduino returns 1, if button 2 is pressed the arduino returns 2
  Button A input is PIN 3, Button B input is PIN 2, and the momentary button press input is PIN 4.
  On the R02A receiving module, Button A is output D2, Button B is output D3, Momentary button press
  is output VT. 

  Hardware: Sparkfun Pro Micro 5V/16MHz

*/

void setup(){
  Serial.begin(9600);
  for (int i = 2; i<5; i++){
    pinMode(i, INPUT);
  }
}

int firstPin;
int secondPin;
int thirdPin; 

bool button1Down = false;
bool button2Down = false;


void loop(){
  firstPin = digitalRead(3);
  secondPin = digitalRead(2);
  thirdPin = digitalRead(4); 

  if ((firstPin == 1 & secondPin == 0 & thirdPin == 1) && !button1Down) {
    Serial.println(1);
    button1Down = true;
    delay(20);
  }
  if ((firstPin == 0 & secondPin == 1 & thirdPin == 1) && !button2Down) {
    Serial.println(2);
    button2Down = true;
    delay(20);
  }
  if (firstPin == 0 & secondPin == 0) {
    button1Down = false;
    button2Down = false;
  }
}

