bool ledstate;

void setup() {
  pinMode(25, OUTPUT);
  pinMode(28, INPUT);

  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.print("Serial initialised");
}

void loop() {
  serialRead();
  if(ledstate){
    digitalWrite(25, HIGH);
  } else {
    digitalWrite(25, LOW);
  }
}

// to determine if led got lag
void serialRead()
{
  if (Serial.available())
  {
    char b = Serial.read();
    Serial.flush(); 
    if (b == '1') {
      ledstate = true;
    } else if (b == '0') {
      ledstate = false;
    }
  }
}
