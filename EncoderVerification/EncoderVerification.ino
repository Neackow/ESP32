/*
#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1

#define DIR2 32      // Direction pin motor 2
#define EN2  33      // PWM pin motor 2
#define SA2  25      // Interrupt pins motor 2
#define SB2  26      // Interrupt pins motor 2
*/


#define DIR1 2
#define EN1 3
#define SA1 4
#define SB1 5


#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       6   // Pulse Per Rotation for the encoder. In my case, 3. 
#define NMOTORS   2



int   pos[NMOTORS]          = {0,0};  // To safely get the value of posi[NMOTORS] within noInterrupt().
volatile int posi[NMOTORS]  = {0,0};  // Update the encoder count.
int   posPrev[NMOTORS]      = {0,0};  // Previous value of the encoder count.

// For the version without template structure.
//volatile int pos_i1 = 0;
//volatile int pos_i2 = 0;

long  prevT[NMOTORS]      = {0,0};  // Previous time at which a computation was made.


float vFilt[NMOTORS]      = {0.0,0.0};  // Filtered speed.
float vPrev[NMOTORS]      = {0.0,0.0};  // Previous speed.

/*
#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1*/

const int dir[] = {19,32};  // Direction pins
const int en[]  = {18,33};  // Enable pins: sets PWM
const int sa[]  = {17,25};  // Encoder pins SA: interrupt required
const int sb[]  = {16,26};  // Encoder pins SB: interrupt required

unsigned long previousMillis = 0;
const long interval = 20000;
float v[NMOTORS]    = {0.0,0.0};
long currT          = 0;
float deltaT        = 0.0;
int diff            = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  
  pinMode(SA1, INPUT);
  pinMode(SB1, INPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);

  /*
  pinMode(SA2, INPUT);
  pinMode(SB2, INPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);
*/
  analogWrite(EN1, 0);
  digitalWrite(DIR1, 0);
  //analogWrite(EN2, 0);
  //digitalWrite(DIR2, 0);

  attachInterrupt(digitalPinToInterrupt(SA1), changeEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(SA1), encoder_F1, FALLING);
  //attachInterrupt(digitalPinToInterrupt(SA1), encoder_R1, RISING);
  /*attachInterrupt(SA2, encoder_R2, RISING);
  attachInterrupt(SA2, encoder_F2, FALLING);

  
  for(int k=0; k < NMOTORS; k++){
    pinMode(dir[k],OUTPUT);
    pinMode(en[k],OUTPUT);
    pinMode(sa[k],INPUT);
    pinMode(sb[k],INPUT);
    analogWrite(en[k], 0);
    digitalWrite(dir[k], 0);
  }

  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_R<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_R<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_F<0>,FALLING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_F<1>,FALLING);*/
  
  delay(2000);
  long timeOfStart = micros();
  for(int k = 0; k < NMOTORS; k++){
    prevT[k] = timeOfStart;
  }
  previousMillis = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = micros();
  
  int target[NMOTORS];
  target[0] = 255;
  target[1] = 170;

  int direction[NMOTORS];
  direction[0] = 1;
  direction[1] = 1-direction[0]; // This version is when I want to go forward or backward. It automatically gets 0 or 1 when I put 1 or 0 on direction[0].
  /*
  // To go left, right, etc.
  direction[0] = 1;
  direction[1] = 1;
  */
  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  interrupts();

  //vFilt[k]    = 0.9984*vFilt[k] + 0.00078*v + 0.00078*vPrev[k];
  //vPrev[k]    = v;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    currT     = micros();
    deltaT    = ((float) (currT-prevT[0]))/1.0e6;
    diff      = -(pos[0] - posPrev[0]);
    v[0]      = (diff*60.0)/(PPR*GB_RATIO*deltaT); // The *60.0 is here to get the speed in RPM and not in RPS.

    posPrev[0]  = pos[0];
    prevT[0]    = currT;
  }

  vFilt[0]    = 0.9806*vFilt[0] + 0.00973*v[0] + 0.0097*vPrev[0]; // 5Hz cut-off frequency, sampling at 1600Hz
  vPrev[0]    = v[0]; 

  setMotor(direction[0],target[0],DIR1,EN1);
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
  Serial.print(v[0]);
  Serial.print(" ");
  Serial.print(vFilt[0]);
  Serial.print(" ");
  Serial.print(diff);
  Serial.println();

  delayMicroseconds(625);
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

void changeEncoder(){
  int a = digitalRead(SA1);
  if(a == HIGH){ 
    encoder_R1();
  }
  else encoder_F1();
}


void encoder_R1(){            // Reminder: if A is triggered Rising but B is already high, it means B first, so going backward. And vice-versa.
  int b = digitalRead(SB1);  
  if(b>0) posi[0]--;           
  else posi[0]++;
}
void encoder_F1(){
  int b = digitalRead(SB1);
  if(b>0) posi[0]++;
  else posi[0]--;
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

