#include <Arduino.h>

#define LTEMT PIN_F4
#define RTEMT PIN_B6
#define PICOP1 PIN_D2
#define PICOP2 PIN_D3
#define debug_values 1

const int left_thresh = 200,
  right_thresh = 100;
int left, right, curr = 3, send = 3;
long startTimer;

//left - white, black: 540, 200 (160)
//right - white, black: 340, 100 (70)

//left - green: 230
//right - green : 120

//left - silver: 320 - 370
//right - silver: 200 - 260

void setup() {
  pinMode(LTEMT, INPUT);
  pinMode(RTEMT, INPUT);
  pinMode(PICOP1, OUTPUT);
  pinMode(PICOP2, OUTPUT);

  Serial.begin(9600);
}

void loop() 
{

  left = analogRead(LTEMT);
  right = analogRead(RTEMT);
  #if debug_values
  Serial.print("Left: ");
  Serial.print(left);
  Serial.print("     Right: ");
  Serial.println(right);
  #endif
  
  //* DATA RECEIVED

  if (left > 700 && right > 700) //silver line
  {
    if (curr != 0) {
      startTimer = millis();
      curr = 0;
    }
  }
  else if (left < left_thresh && right > right_thresh) //turn left
  {
    if (curr != 1) {
      startTimer = millis();
      curr = 1;
    }
  }
  else if (left > left_thresh && right < right_thresh) //turn right
  {
    if (curr != 2) {
      startTimer = millis();
      curr = 2;
    }
  }
  else //nothing
  {
    curr = 3;
  }

  //* CURRENT

  switch (curr)
  {
  case 0: //silver
    if (millis() - startTimer > 400) send = 0;
    else send = 3;
    #if debug_values
    Serial.println("SILVER LINE");
    #endif
    break;

  case 1: //left
    if (millis() - startTimer > 400) send = 1;
    else send = 3;
    #if debug_values
    Serial.println("LEFT TURN");
    #endif
    break;

  case 2:
    if (millis() - startTimer > 400) send = 2;
    else send = 3;
    #if debug_values
    Serial.println("RIGHT TURN");
    #endif
    break;

  case 3: //nothing
    send = 3;
    break;
  }

  //* SEND

  switch (send)
  {
  case 0: //silver 11
    digitalWrite(PICOP1, HIGH);
    digitalWrite(PICOP2, HIGH);
    break;

  case 1: //left 01
    digitalWrite(PICOP1, LOW);
    digitalWrite(PICOP2, HIGH);
    break;

  case 2: //right 10
    digitalWrite(PICOP1, HIGH);
    digitalWrite(PICOP2, LOW);
    break;

  case 3: //nothing 00
    digitalWrite(PICOP1, LOW);
    digitalWrite(PICOP2, LOW);
    break;
  
  default:
    break;
  }
}
