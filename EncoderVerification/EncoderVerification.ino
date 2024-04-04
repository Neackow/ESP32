/*
#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1

#define DIR2 26      // Direction pin motor 2
#define EN2  25      // PWM pin motor 2
#define SA2  33      // Interrupt pins motor 2
#define SB2  32      // Interrupt pins motor 2
*/

#define NMOTORS 2

const int dir[] = {19,32};  // Direction pins
const int en[]  = {18,33};  // Enable pins: sets PWM
const int sa[]  = {17,25};  // Encoder pins SA: interrupt required
const int sb[]  = {16,26};  // Encoder pins SB: interrupt required

int   pos1          = 0;
int   pos2          = 0;
volatile int posi[NMOTORS]   = {0,0};
volatile int pos_i1 = 0;
volatile int pos_i2 = 0;


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

  attachInterrupt(SA1, encoder_R1, RISING);
  attachInterrupt(SA1, encoder_F1, FALLING);
  attachInterrupt(SA2, encoder_R2, RISING);
  attachInterrupt(SA2, encoder_F2, FALLING);*/

  
  for(int k=0; k < NMOTORS; k++){
    pinMode(dir[k],OUTPUT);
    pinMode(en[k],OUTPUT);
    pinMode(sa[k],INPUT);
    pinMode(sb[k],INPUT);
  }

  analogWrite(en[0], 0);
  digitalWrite(dir[0], 0);
  analogWrite(en[1], 0);
  digitalWrite(dir[1], 0);

  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_R<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_R<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_F<0>,FALLING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_F<1>,FALLING);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int target[NMOTORS];
  target[0] = 170;
  target[1] = 170;

  int direction[NMOTORS];
  direction[0] = 1;
  direction[1] = 0;

  for(int k = 0; k < NMOTORS; k++){
    setMotor(direction[k],target[k],dir[k],en[k]);
  }

  noInterrupts();
  pos1 = posi[0];
  pos2 = posi[1];
  //pos1 = pos_i1;
  //pos2 = pos_i2;
  interrupts();
  
  Serial.print("Motor1: ");
  Serial.print(pos1);
  Serial.print(" & Motor2: ");
  Serial.print(pos2);
  Serial.println();
}

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}


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
}


/*
void encoder_R1(){            // Reminder: if A is triggered Rising but B is already high, it means B first, so going backward. And vice-versa.
  int sb = digitalRead(SB1);  
  if(sb>0) pos_i1--;           
  else pos_i1++;
}
void encoder_F1(){
  int sb = digitalRead(SB1);
  if(sb>0) pos_i1++;
  else pos_i1--;
}

void encoder_R2(){
  int sb = digitalRead(SB2);  
  if(sb>0) pos_i2--;           
  else pos_i2++;
}
void encoder_F2(){
  int sb = digitalRead(SB2);
  if(sb>0) pos_i2++;
  else pos_i2--;
}
*/
