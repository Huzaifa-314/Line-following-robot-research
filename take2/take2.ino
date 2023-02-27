const int sensorcount=7;
int sensorpins[sensorcount]={13,12,11,10,9,8,7};
int sensorvalues[sensorcount];

void setup(){
  for(int i=0;i<sensorcount;i++){
    pinMode(sensorpins[i],INPUT);
  }
  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  for(int i=0;i<sensorcount;i++){
    sensorvalues[i] = digitalRead(sensorpins[i]);
    Serial.print(sensorvalues[i]);
    Serial.print('\t');
  }
  int mvalues=0;
  int sum = 0;
  for(int i=0; i<sensorcount; i++){
    mvalues += sensorvalues[i]*(i*1000);
    sum+=sensorvalues[i];
  }
  float position = 3000-(mvalues/sum);
  Serial.println(position);
}
