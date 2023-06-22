void setup() {
  // put your setup code here, to run once:
 
 pinMode(11, OUTPUT);
 pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 analogWrite(11, 80);
 analogWrite(10, 0);

}
