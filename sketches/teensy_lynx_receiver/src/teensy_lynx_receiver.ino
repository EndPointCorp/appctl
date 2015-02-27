/*
 *
 * teensy_lynx_receiver.ino
 *
 * sketch for using teensy with Lynx receiver
 *
 */

int pin_led = 13;

int pin_d0 = 0;
int pin_d1 = 1;
int pin_d2 = 2;
int pin_d3 = 3;
int pin_d4 = 4;

int button0down;
int button1down;
int button2down;
int button3down;
int button4down;

boolean button0pressed;
boolean button1pressed;
boolean button2pressed;
boolean button3pressed;
boolean button4pressed;

void setup () {
  Serial.begin(9600);

  pinMode(pin_led, OUTPUT);
  pinMode(pin_d0, INPUT);
  pinMode(pin_d1, INPUT);
  pinMode(pin_d2, INPUT);
  pinMode(pin_d3, INPUT);
  pinMode(pin_d4, INPUT);

  button0pressed = false;
  button1pressed = false;
  button2pressed = false;
  button3pressed = false;
  button4pressed = false;
}

void loop () {
  button0down = digitalRead(pin_d0);
  button1down = digitalRead(pin_d1);
  button2down = digitalRead(pin_d2);
  button3down = digitalRead(pin_d3);
  button4down = digitalRead(pin_d4);

  if( button0down && !button0pressed ) {
    button0pressed = button0down;
    Serial.println(1);
  }

  if(button1down && !button1pressed) {
    button1pressed = button1down;
    Serial.println(2);
  }

  if(button2down && !button2pressed) {
    button3pressed = button3down;
    Serial.println(3);
  }

  if(button3down && !button3pressed) {
    button3pressed = button3down;
    Serial.println(4);
  }

  if(button4down && !button4pressed) {
    button4pressed = button4down;
    Serial.println(5);
  }

  button0pressed = button0down;
  button1pressed = button1down;
  button2pressed = button2down;
  button3pressed = button3down;
  button4pressed = button4down;

  if( button0pressed || button1pressed || button2pressed || button3pressed || button4pressed) {
    digitalWrite(pin_led, HIGH);
  }
  else {
    digitalWrite(pin_led, LOW);
  }
}
