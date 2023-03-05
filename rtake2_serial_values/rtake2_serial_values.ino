const int sensorcount=7;
int sensorpins[sensorcount]={13,12,11,10,9,8,7};
int sensorvalues[sensorcount];
int lastposition;
int setpoint=3000;

//pid variables
const float kp=0.083333333333;

int basespeed = 130;
//left motor
#define enl 6 //enable left
#define lfm A1 //left forword motor //IN2
#define lbm A2 //left backword //IN1

//right motor
#define enr 5
#define rfm A3 //right forword //IN3
#define rbm A4 //right backword //IN4
void setup(){
  //motor setup
    // Set all the motor control pins to outputs
  pinMode(enl, OUTPUT);
  pinMode(enr, OUTPUT);
  pinMode(lfm, OUTPUT);
  pinMode(lbm, OUTPUT);
  pinMode(rfm, OUTPUT);
  pinMode(rbm, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(lfm, LOW);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, LOW);
  digitalWrite(rbm, LOW);


  //sensor setup
  for(int i=0;i<sensorcount;i++){
    pinMode(sensorpins[i],INPUT);
  }
  Serial.begin(9600);
}

void drive(int speedl,int speedr){
  // Turn on motor A & B
  digitalWrite(lfm, HIGH);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, HIGH);
  digitalWrite(rbm, LOW);

  analogWrite(enl, speedl);
  analogWrite(enr, speedr);
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
  int position = mvalues/sum;
  if(position == -1 && lastposition > setpoint){
    position = 6000;
  }
  if(position == -1 && lastposition < setpoint){
    position = 0;
  }
  lastposition = position;
  
  Serial.print(position);
  Serial.print('\t');
  int Perror = setpoint-position;
  Serial.print(Perror);
  Serial.print('\t');
  //int Derror = Perror-lasterror;
  //Serial.print(kd*Derror);
  //Serial.print('\t');
  float speedchange=(kp*Perror);
  //lasterror=Perror;
  Serial.print(speedchange);
  Serial.print('\t');
   int speedl= basespeed-speedchange;
   int speedr= basespeed+speedchange;
   

   Serial.print(speedl);
   Serial.print('\t');
   Serial.println(speedr);

   drive(speedl,speedr);
  
}
