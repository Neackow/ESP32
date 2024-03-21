#define DIR1 19  // Change this to whatever GPIO pin you want
#define EN1 18   // This needs to be a PWM pin
#define SA1 17
#define SB1 16


#define GB_RATIO    19  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR         3   // Pulse Per Rotation for the encoder. In my case, 3.


int   pos      = 0;
int   posPrev    = 0;
volatile int pos_i = 0;
long  prevT      = 0;
float eprev      = 0.0;
float eintegral  = 0;

uint32_t t_speed;
float raw_speed;

void setup() {
  Serial.begin(115200);

  pinMode(SA1, INPUT);
  pinMode(SB1, INPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);


  analogWrite(EN1, 0);
  digitalWrite(DIR1, 0);

  /*
  pinMode(SA2,INPUT);
  pinMode(SB2,INPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(EN2,OUTPUT);*/


  attachInterrupt(SA1, readEncoder, RISING);
  //attachInterrupt(SA1, encoder_R1, RISING);
  //attachInterrupt(SA1, encoder_F1, FALLING);

  delay(2000);
}

void loop() {
  
  // Position target
  //int target = 500;

  int pwm = 255; //180/3.0*micros()/1.0e6;
  int dir = 1;
  setMotor(dir, pwm, DIR1, EN1);

  
  noInterrupts();
  pos = pos_i;
  interrupts();

  // Compute velocity
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  int diff = pos - posPrev;
  float velocity1 = (diff)/(PPR*GB_RATIO*deltaT);
  posPrev = pos;
  prevT = currT;

  /*
  uint32_t new_t_speed = micros();
  uint32_t dt = (new_t_speed - t_speed);
  t_speed = new_t_speed;
  raw_speed = float(1000000*pos_i)/(3*GB_RATIO*dt);
  pos_i = 0;*/


  Serial.print("Velocity brute: ");
  Serial.print(velocity1);
  Serial.print(" deltaT: ");
  Serial.print(deltaT);
  Serial.print(" (pos - posPrev): ");
  Serial.print(diff);
  Serial.print(" current-time: ");
  Serial.print(currT);
  Serial.print(" counter: ");
  Serial.print(pos);
  Serial.println();


  // Controller
  
  /*
  float kp = 1.2;
  float ki = 0;
  float kd = 3;

  long currT1 = micros();
  float deltaT1 = ((float) (currT1 - prevT1))/1.0e6;
  prevT1 = currT1;

  int e1 = pos_i1 - target;
  float dedt = (e1-eprev1)/deltaT1;
  eintegral1 += e1*deltaT1;

  float u1 = kp*e1 + kd*dedt + ki*eintegral1;

  // Conversion of the command into a speed and direction
  float pwm = fabs(u1);
  if(pwm > 255){
    pwm = 255;
  }

  int dir = 1;
  if(u1 < 0){
    dir = 0;
  }

  // Set command
  setMotor(dir, pwm, DIR1, EN1);

  // End the loop
  eprev1 = e1;
  */

}

// *********************************************** //
// ************** ENCODERS READING *************** //
// *********************************************** //
void encoder_R1(){            // Reminder: if A is triggered Rising but B is already high, it means B first, so going backward. And vice-versa.
  int sb = digitalRead(SB1);  
  if(sb>0) pos_i--;           
  else pos_i++;
}
void encoder_F1(){
  int sb = digitalRead(SB1);
  if(sb>0) pos_i++;
  else pos_i--;
}

void readEncoder(){
  int b = digitalRead(SB1);
  if(b>0){
    pos_i++;
  }
  else pos_i--;
}


// *********************************************** //
// ************* SET MOTOR FUNCTION ************** //
// *********************************************** //

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}