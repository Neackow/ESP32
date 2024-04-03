#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1

#define DIR2 26      // Direction pin motor 2
#define EN2  25      // PWM pin motor 2
#define SA2  33      // Interrupt pins motor 2
#define SB2  32      // Interrupt pins motor 2

int   pos1          = 0;
int   pos2          = 0;
volatile int pos_i1 = 0;
volatile int pos_i2 = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
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
  attachInterrupt(SA2, encoder_F2, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  noInterrupts();
  pos1 = pos_i1;
  pos2 = pos_i2;
  interrupts();
  
  Serial.print("Motor1: ");
  Serial.print(pos1);
  Serial.print(" & Motor2: ");
  Serial.print(pos2);
  Serial.println();
}


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
