#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1

#define DIR2 26      // Direction pin motor 2
#define EN2  25      // PWM pin motor 2
#define SA2  33      // Interrupt pins motor 2
#define SB2  32      // Interrupt pins motor 2




#define GB_RATIO    53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR         3   // Pulse Per Rotation for the encoder. In my case, 3.

// 1 = motor 1; 2 = motor 2;
int   pos1          = 0;
int   pos2          = 0;
int   posPrev[2]    = {0,0};
volatile int pos_i1 = 0;
volatile int pos_i2 = 0;
long  prevT[2]     = {0.0,0.0};
float eprev[2]      = {0.0,0.0};
float eintegral[2]  = {0.0,0.0};

float v1Filt[2]   = {0.0,0.0};
float v1Prev[2]   = {0.0,0.0};

// Control command
float pwm = 0.0;
const char* message[2] = {"Test1","Test2"};

void setup() {
  Serial.begin(115200);

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


  //attachInterrupt(SA1, readEncoder, RISING);
  attachInterrupt(SA1, encoder_R1, RISING);
  attachInterrupt(SA1, encoder_F1, FALLING);
  attachInterrupt(SA2, encoder_R2, RISING);
  attachInterrupt(SA2, encoder_F2, FALLING);

  delay(2000);
}



void loop(){

  noInterrupts();
  pos1 = pos_i1;
  pos2 = pos_i2;
  interrupts();

  // TO GET FROM I2C
  float target[2] = {120.0,0.0};   // IN RPM. My max is 150.
  // Compute velocity
  
  computeVelocityAndController(pos1, prevT[0], posPrev[0], target[0], 0);
  computeVelocityAndController(pos2, prevT[1], posPrev[1], target[1], 1);

  // Calls controller inside of computeVelocityAndController();

  /*Serial.print(pos1);
  Serial.print(" ");
  Serial.print(pos2);
  Serial.println();*/

  delay(0.5); // 0.5ms delay: to maintain correct sampling frequency of 2000 Hz. 

}

// *********************************************** //
// ************* VELOCITY & CONTROL ************** //
// *********************************************** //
void computeVelocityAndController(int pos,long prevTime, int posPrevious, float target, int index){
  Serial.println(message[index]);
  long currT = micros();
  float deltaT = ((float) (currT-prevTime))/1.0e6;
  int diff = -(pos - posPrevious);
  float v1 = (diff)/(PPR*GB_RATIO*deltaT) *60.0; // Could add * 60 to be in RPM.
  
  posPrev[index] = pos;
  prevT[index] = currT;
  v1Filt[index] = 0.9984*v1Filt[index] + 0.00078*v1 + 0.00078*v1Prev[index];
  v1Prev[index] = v1;
  
  controller(deltaT, target, index);
}

void controller(float deltaT, float target, int index){
  float e = target - v1Filt[index];
  int dir = 1;  // Spins away from me;

  if(fabs(target) > 90){
    // Controller  
    float kp = 10.0;
    float ki = 4.0;

    eintegral[index] += e*deltaT;

    if(eintegral[index] > 50){
      eintegral[index] = 50.0;
    }
    float integralTerm = ki*eintegral[index];

    if(integralTerm > 200){
      integralTerm = 200.0;
    }

    float u = kp*e + integralTerm; 

    // Conversion of the command into a speed and direction
    pwm = fabs(u);
    if(pwm > 255){
      pwm = 255;
    }
    if(u < 0){
      dir = 0;
    }
    // Set command
    pwm = (int) pwm;
    if(index == 0){
      setMotor(dir, pwm, DIR1, EN1);
    }
    else if(index == 1){
      setMotor(dir, pwm, DIR2, EN2);
    }
    else{
      Serial.println("Invalid motor");
    }
    
  }
  else {
    setMotor(dir, 0, DIR1, EN1);
    setMotor(dir, 0, DIR2, EN2);
  }
  //setMotor(dir, target, DIR1, EN1);

  // End the loop
  eprev[index] = e;
}

// *********************************************** //
// ************** ENCODERS READING *************** //
// *********************************************** //
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


// *********************************************** //
// ************* SET MOTOR FUNCTION ************** //
// *********************************************** //

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}