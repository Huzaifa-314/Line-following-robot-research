const int sensorcount=5;
const int sidesensorcount=2;
int sidesensorpins[sidesensorcount]={12,4};
int sidesensorvalues[sidesensorcount]={0,0};
int sensorpins[sensorcount]={11,10,9,8,7};
int sensorvalues[sensorcount]={0,0,0,0,0};
int setpoint=((sensorcount-1)*1000)/2;

//left motor
#define enl 6 //enable left
#define lfm A1 //left forword motor //IN2
#define lbm A2 //left backword //IN1

//right motor
#define enr 5
#define rfm A3 //right forword //IN3
#define rbm A4 //right backword //IN4


/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.07; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0.0008; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0.6; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;
//all variable declaration
int lastposition;
int lastError = 0;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 150;
const uint8_t basespeedb = 150;

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
  //side sensors
  for(int i=0;i<sidesensorcount;i++){
    pinMode(sidesensorpins[i],INPUT);
  }
  Serial.begin(9600);
  
}




void readsensors(){
  for(int i=0;i<sensorcount;i++){
    sensorvalues[i] = digitalRead(sensorpins[i]);
  }
}

uint16_t readLineBlack(){
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000
  readsensors();
  for (uint8_t i = 0; i < sensorcount; i++)
  {
    uint16_t value = sensorvalues[i];
    if (value == 1) { onLine = true; }
    
    avg += (uint32_t)value * (i * 1000);
    sum += value;
  }
  
  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (lastposition < (sensorcount - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (sensorcount - 1) * 1000;
    }
  }

  lastposition = avg / sum;
  return lastposition;
      
}


void drive(int speedl,int speedr){
//  speedr = constrain(speedr, 0, 255);
//  speedl = constrain(speedl, 0, 255);
  // Turn on motor A & B
  digitalWrite(lfm, HIGH);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, HIGH);
  digitalWrite(rbm, LOW);

  analogWrite(enl, speedl);
  analogWrite(enr, speedr);
  
}

void loop() {
  for(int i=0;i<sidesensorcount;i++){
    sidesensorvalues[i] = digitalRead(sidesensorpins[i]);
  }
  Serial.print(sidesensorvalues[0]);
  Serial.print(' ');
  readsensors();
  for(int i=0;i<sensorcount;i++){
    Serial.print(sensorvalues[i]);
    Serial.print('\t');
  }
  Serial.print(sidesensorvalues[1]);

//  int mvalues=0;
//  int sum = 0;
//  for(int i=0; i<sensorcount; i++){
//    mvalues += sensorvalues[i]*(i*1000);
//    sum+=sensorvalues[i];
//  }
  int position = readLineBlack();
  Serial.print('\t');
  Serial.print(position);
  Serial.print('\t');
  
  
  int error = setpoint-position;
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }
  drive(motorspeeda,motorspeedb);
  Serial.print(P);
  Serial.println();
}
