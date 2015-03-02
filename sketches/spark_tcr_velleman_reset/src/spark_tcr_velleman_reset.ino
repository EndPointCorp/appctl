
/*
 *
 * spark_tcr_velleman_reset
 *
 * sketch for using sparkfun pro micro (16mhz) with a velleman remote
 * temporary solution for TCR until issues with Lynx are resolved
 *
 */

int pin_button_2 = 16;
int button2down;

void setup(){
  Serial.begin(9600);
  pinMode(pin_button_2, INPUT_PULLUP); 
}

void loop(){

  if (digitalRead(pin_button_2) == LOW) {
    Serial.println(2);
    
    delay(900);
  }

}





