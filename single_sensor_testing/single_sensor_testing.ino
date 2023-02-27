const int sensorCount = 7;
int sensorValues[sensorCount]={7,8,9,10,11,12,13};
void setup() {
  for(int i=0;i<sensorCount;i++){
    pinMode(sensorValues[i],INPUT);
  }
  Serial.begin(9600);
}

void loop() {
  for(int i=0;i<sensorCount;i++){
    Serial.print(digitalRead(sensorValues[i]));
    Serial.print('\t');
  }
  Serial.println();

}
