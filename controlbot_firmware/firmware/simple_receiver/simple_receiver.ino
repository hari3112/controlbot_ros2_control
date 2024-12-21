#define LEDPIN 13

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPIN,OUTPUT);
  digitalWrite(LEDPIN,LOW);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    int x = Serial.readString().toInt();
    if (x==0)
    {
      digitalWrite(LEDPIN,LOW);
    }
    else
    {
      digitalWrite(LEDPIN,HIGH);
    }
  }

}
