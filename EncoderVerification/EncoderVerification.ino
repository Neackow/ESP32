#include <Wire.h>


#define DIR1 9
#define EN1 8
#define SA1 7
#define SB1 6

#define DIR2 18
#define EN2 19
#define SA2 20
#define SB2 21

#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       6   // Pulse Per Rotation for the encoder. In my case, 3. 
#define NMOTORS   2


int   pos[NMOTORS]          = {0,0};  // To safely get the value of posi[NMOTORS] within noInterrupt().
volatile int posi[NMOTORS]  = {0,0};  // Update the encoder count.
int   posPrev[NMOTORS]      = {0,0};  // Previous value of the encoder count.

// For the version without template structure.
//volatile int pos_i1 = 0;
//volatile int pos_i2 = 0;

long  prevT[NMOTORS]    = {0,0};  // Previous time at which a computation was made.
long prevT_V[NMOTORS]   = {0,0};
unsigned long previousMillis[NMOTORS] = {0,0};

float vFilt[NMOTORS]    = {0.0,0.0};  // Filtered speed.
float vPrev[NMOTORS]    = {0.0,0.0};  // Previous speed.


const int dir[] = {9,18};  // Direction pins
const int en[]  = {8,19};  // Enable pins: sets PWM
const int sa[]  = {7,20};  // Encoder pins SA: interrupt required
const int sb[]  = {6,21};  // Encoder pins SB: interrupt required


const long interval = 20000;
float v[NMOTORS]    = {0.0,0.0};
long currT          = 0;
float deltaT        = 0.0;
int diff            = 0;

float target[4] = {0.0,0.0,0.0,0.0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  /*
  pinMode(SA1, INPUT);
  pinMode(SB1, INPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);

  pinMode(SA2, INPUT);
  pinMode(SB2, INPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  analogWrite(EN1, 0);
  digitalWrite(DIR1, 0);
  analogWrite(EN2, 0);
  digitalWrite(DIR2, 0);

  attachInterrupt(digitalPinToInterrupt(SA1), changeEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(SA1), encoder_F1, FALLING);
  //attachInterrupt(digitalPinToInterrupt(SA1), encoder_R1, RISING);
  attachInterrupt(digitalPinToInterrupt(SA2), encoder_R2, RISING);
  attachInterrupt(digitalPinToInterrupt(SA2), encoder_F2, FALLING);*/

  
  for(int k=0; k < NMOTORS; k++){
    pinMode(dir[k],OUTPUT);
    pinMode(en[k],OUTPUT);
    pinMode(sa[k],INPUT);
    pinMode(sb[k],INPUT);
    analogWrite(en[k], 0);
    digitalWrite(dir[k], 0);
  }

  attachInterrupt(digitalPinToInterrupt(sa[0]),changeEncoder<0>,CHANGE);  // It does not work to define 2 attachInterrupt function (RISING and FALLING), only the last is executed. Thus: CHANGE.
  attachInterrupt(digitalPinToInterrupt(sa[1]),changeEncoder<1>,CHANGE);
  
  delay(2000);
  for(int k = 0; k < NMOTORS; k++){
    prevT[k] = micros();
    prevT_V[k] = micros();
    previousMillis[k] = micros();
  }


  Wire.begin(0x40);             // Defines the card's slave address
  Wire.onReceive(receiveEvent); // To receive the command.
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = micros();
  
  /*
  int target[NMOTORS];
  target[0] = 255;
  target[1] = 170;

  int direction[NMOTORS];
  direction[0] = 1;
  direction[1] = 1-direction[0]; // This version is when I want to go forward or backward. It automatically gets 0 or 1 when I put 1 or 0 on direction[0].
  */
  /*
  // To go left, right, etc.
  direction[0] = 1;
  direction[1] = 1;
  */
  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  interrupts();

  if (currentMillis - previousMillis[0] >= interval) {
    previousMillis[0] = currentMillis;
    currT     = micros();
    deltaT    = ((float) (currT-prevT[0]))/1.0e6;
    diff      = (posPrev[0] - pos[0]);
    v[0]      = (diff*60.0)/(PPR*GB_RATIO*deltaT); // The *60.0 is here to get the speed in RPM and not in RPS.

    posPrev[0]  = pos[0];
    prevT[0]    = currT;
  }

  vFilt[0]    = 0.9806*vFilt[0] + 0.00973*v[0] + 0.0097*vPrev[0]; // 5Hz cut-off frequency, sampling at 1600Hz
  vPrev[0]    = v[0]; 

  int direction = (int) target[1];
  setMotor(1,200,DIR1,EN1);
  setMotor(0,200,DIR2,EN2);
  /*
  for(int k = 0; k < NMOTORS; k++){
    setMotor(direction[k],target[k],dir[k],en[k]);
  }*/
  
  /*
  Serial.print("Motor1: ");
  Serial.print(pos[0]);
  Serial.print(" & Motor2: ");
  Serial.print(pos[1]);
  Serial.println();
  Serial.print(currT);
  Serial.print(" ");
  Serial.print(pos[0]);
  Serial.print(" ");*/
  /*Serial.print(v[0]);
  Serial.print(" ");
  Serial.print(vFilt[0]);
  Serial.print(" ");
  Serial.print(v[1]);
  Serial.print(" ");
  Serial.print(vFilt[1]);
  Serial.print(" ");*/
  Serial.print(pos[0]);
  Serial.print(" ");
  Serial.print(pos[1]);
  //Serial.print(" ");
  //Serial.print(diff);
  Serial.println();

}


void receiveEvent(int howMany) 
{
  for(int i=0; i < howMany; i++){
    target[i] = Wire.read();
    //Serial.println(target[i]);
  }
  //Serial.println();
}


void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}

/*
template <int i> 
void encoder_R(){
  int b = digitalRead(sb[i]);
  if(b > 0){
    posi[i]--;
  }
  else{
    posi[i]++;
  }
}

template <int j>
void encoder_F(){
  int b = digitalRead(sb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}*/

template <int i>
void changeEncoder(){
  int a = digitalRead(sa[i]);
  if(a == HIGH){ 
    encoder_R(i);
  }
  else encoder_F(i);
}

void encoder_R(int index){
  int b = digitalRead(sb[index]);
  if(b > 0){
    posi[index]--;
  }
  else{
    posi[index]++;
  }
}

void encoder_F(int index){
  int b = digitalRead(sb[index]);
  if(b > 0){
    posi[index]++;
  }
  else{
    posi[index]--;
  }
}
/*
void encoder_R2(){
  int b = digitalRead(SB2);  
  if(b>0) posi[1]--;           
  else posi[1]++;
}
void encoder_F2(){
  int b = digitalRead(SB2);
  if(b>0) posi[1]++;
  else posi[1]--;
}*/

