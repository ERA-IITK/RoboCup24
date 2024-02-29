#define NMOTORS 2

// Pins
const int enca[] = {2,3};
const int encb[] = {8,9};
const int pwm[] = {10,6};
const int in1[] = {12,4};     // SLP
const int in2[] = {11,5};     // DIR

// Globals
long prevT = 0;
volatile int posi[] = {0,0};
int eprev[]={0,0};
int eintegral[]={0,0};
float kp=1;
float kd=0.006;
float ki=0;

void setup() {
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target[NMOTORS];
  target[0]=20000;
  target[1]=20000;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // error
    int e = target[k] - pos[k];
  
    // derivative
    float dedt = (e-eprev[k])/(deltaT);
  
    // integral
    eintegral[k] = eintegral[k] + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral[k];
    // Serial.print(u);
    // Serial.print(" ");

    // motor power
    pwr = (int) fabs(u);
    if( pwr > 30 ){
      pwr = 30;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // Serial.print(dir);
    // Serial.print(" ");
    // Serial.print(pwr);
    // Serial.print(" ");

    // store previous error
    eprev[k] = e;

    analogWrite(pwm[k],pwr);
    if(dir == 1){
      // Serial.print("f ");
      digitalWrite(in1[k],LOW);
      digitalWrite(in2[k],HIGH);
    }
    else if(dir == -1){
      // Serial.print("b ");
      digitalWrite(in1[k],LOW);
      digitalWrite(in2[k],LOW);
    }
    else{
      digitalWrite(in1[k],HIGH);
      digitalWrite(in2[k],LOW);
    }  
    // signal the motor
    // setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
  else{
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}