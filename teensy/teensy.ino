#include <Arduino.h>

#define LTEMT PIN_F4
#define RTEMT PIN_B6
#define PICOP1 PIN_D2
#define PICOP2 PIN_D3
#define debug_values 1

const int left_thresh = 320,
  right_thresh = 210;
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

  if (left < left_thresh) {
    if (right < right_thresh) {
      send = 0; //double black
    } else {
      send = 1; //turn left
    }
  } else {
    if (right < right_thresh) {
      send = 2; //turn right
    } else {
      send = 3; //double white
    }
  }

  //* CURRENT

  // switch (curr)
  // {
  // case 0: //silver
  //   if (millis() - startTimer > 400) send = 0;
  //   else send = 3;
  //   #if debug_values
  //   Serial.println("SILVER LINE");
  //   #endif
  //   break;

  // case 1: //left
  //   if (millis() - startTimer > 400) send = 1;
  //   else send = 3;
  //   #if debug_values
  //   Serial.println("LEFT TURN");
  //   #endif
  //   break;

  // case 2:
  //   if (millis() - startTimer > 400) send = 2;
  //   else send = 3;
  //   #if debug_values
  //   Serial.println("RIGHT TURN");
  //   #endif
  //   break;

  // case 3: //nothing
  //   send = 3;
  //   break;
  // }

  //* SEND

  switch (send)
  {
  case 0: //double black 11
    digitalWrite(PICOP1, HIGH);
    digitalWrite(PICOP2, HIGH);
    Serial.println("11");
    break;

  case 1: //left 01
    digitalWrite(PICOP1, LOW);
    digitalWrite(PICOP2, HIGH);
    Serial.println("01");
    break;

  case 2: //right 10
    digitalWrite(PICOP1, HIGH);
    digitalWrite(PICOP2, LOW);
    Serial.println("10");
    break;

  case 3: //nothing 00
    digitalWrite(PICOP1, LOW);
    digitalWrite(PICOP2, LOW);
    Serial.println("00");
    break;
  
  default:
    break;
  }
}
