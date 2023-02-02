int value[6],valueON[6],valueOFF[6];
void setup()
{
Serial.begin(74880);
pinMode(2, OUTPUT);
}
void loop()
{
digitalWrite(2,HIGH); // turn ON IR LEDs
delay(10); // give some time to turn ON
for(int i=0;i<=5;i++) valueON[i]=analogRead(i);
digitalWrite(2,LOW); // turn OFF IR LEDs
delay(10); // give some time to turn OFF
for(int i=0;i<=5;i++) valueOFF[i]=analogRead(i);
// calculate actual sensor reading
for(int i=0;i<=5;i++)
{
value[i]=valueON[i]-valueOFF[i];
Serial.print(value[i]);
Serial.print(" ");
}
Serial.println(" ");
}