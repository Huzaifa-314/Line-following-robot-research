#define enl 6 //enable left
#define lfm A1 //left forword motor //IN2
#define lbm A2 //left backword //IN1

//right motor
#define enr 5
#define rfm A3 //right forword //IN3
#define rbm A4 //right backword //IN4

void setup() {
  // put your setup code here, to run once:
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

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(lfm, LOW);
  digitalWrite(lbm, LOW);
  digitalWrite(rfm, LOW);
  digitalWrite(rbm, LOW);

  analogWrite(enl, 50);
  analogWrite(enr, 50);

}
