const int sensorcount=7;
int sensorpins[sensorcount]={12,11,10,19,8,7,4};
int sensorvalues[sensorcount];
int setpoint=((sensorcount-1)*1000)/2;

//pid variables
const float kp=0.010333333333;
const float kd=0.2;
const float ki = 0.0008;

int basespeed = 180;

//left motor
#define enl 6 //enable left
#define lfm A1 //left forword motor //IN2
#define lbm A2 //left backword //IN1

//right motor
#define enr 5
#define rfm A3 //right forword //IN3
#define rbm A4 //right backword //IN4


//all variable declaration
int lastposition;
int lasterror = 0;

void setup() {
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
  speedr = constrain(speedr, 0, 255);
  speedl = constrain(speedl, 0, 255);
  // Turn on motor A & B
  digitalWrite(lfm, HIGH);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, HIGH);
  digitalWrite(rbm, LOW);

  analogWrite(enl, speedl);
  analogWrite(enr, speedr);
  
}
void driveleft(){
  // Turn on motor A & B
  digitalWrite(lfm, LOW);
  digitalWrite(lbm, HIGH);
  digitalWrite(rfm, HIGH);
  digitalWrite(rbm, LOW);

  analogWrite(enl, 95);
  analogWrite(enr, 90);
  
}
void driveright(){
  // Turn on motor A & B
  digitalWrite(lfm, HIGH);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, LOW);
  digitalWrite(rbm, HIGH);

  analogWrite(enl, 90);
  analogWrite(enr, 95);
  
}


void loop() {
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
    driveright();
  }
  else if(position == -1 && lastposition < setpoint){
    position = 0;
    driveleft();
    
  }
  else{
  Serial.print(position);
  Serial.print('\t');
  
  int error = setpoint-position;
  int Perror = error;
  int Ierror = Ierror + error;
  int Derror = error - lasterror;
  lasterror = error;
  Serial.print(Perror);
  Serial.print('\t');
  Serial.print(kd*Derror);
  Serial.print('\t');

  float speedchange=(kp*Perror)+ (Ierror*ki) + (Derror*kd);
  Serial.print(speedchange);
  Serial.print('\t');

  int speedl= basespeed-speedchange;
  int speedr= basespeed+speedchange;
  Serial.print(speedl);
  Serial.print('\t');
  Serial.println(speedr);
  
  
  drive(speedl,speedr);
  }

  
  lastposition = position;
  Serial.println();
}
