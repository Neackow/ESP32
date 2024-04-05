#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1

#define DIR2 32      // Direction pin motor 2
#define EN2  33      // PWM pin motor 2
#define SA2  25      // Interrupt pins motor 2
#define SB2  26      // Interrupt pins motor 2

// For the template version.
const int dir[] = {19,32};  // Direction pins
const int en[]  = {18,33};  // Enable pins: sets PWM
const int sa[]  = {17,25};  // Encoder pins SA: interrupt required
const int sb[]  = {16,26};  // Encoder pins SB: interrupt required

// Motor values.
#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       3   // Pulse Per Rotation for the encoder. In my case, 3. 
#define NMOTORS   2

int   pos[NMOTORS]          = {0,0};  // To safely get the value of posi[NMOTORS] within noInterrupt().
volatile int posi[NMOTORS]  = {0,0};  // Update the encoder count.
int   posPrev[NMOTORS]      = {0,0};  // Previous value of the encoder count.

// For the version without template structure.
//volatile int pos_i1 = 0;
//volatile int pos_i2 = 0;

long  prevT[NMOTORS]      = {0.0,0.0};  // Previous time at which a computation was made.
float eprev[NMOTORS]      = {0.0,0.0};  //Previous error in the controller.
float eintegral[NMOTORS]  = {0.0,0.0};  // Store the evolution of the integral error.

float vFilt[NMOTORS]      = {0.0,0.0};  // Filtered speed.
float vPrev[NMOTORS]      = {0.0,0.0};  // Previous speed.

// Control command
//float pwm = 0.0;
float output[NMOTORS]   = {0.0,0.0};      // Output of the controller
float target[4]         = {0.0,0,0.0,0};  // Contains the desired motor behaviour.
const char* message[2]  = {"Test1","Test2"};

// PID controller values.
float kp = 10.0;
float ki = 4.0;


void setup() {
  Serial.begin(9600); //115200 is another option.

  /* Version without template structure.
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
    analogWrite(en[k], 0);
    digitalWrite(dir[k], 0);
  }

  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_R<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_R<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(sa[0]),encoder_F<0>,FALLING);
  attachInterrupt(digitalPinToInterrupt(sa[1]),encoder_F<1>,FALLING);

  // Allows me to have the time to correctly setup the rest of the robot.
  delay(2000);
  long timeOfStart = micros();
  for(int k = 0; k < NMOTORS; k++){
    prevT[k] = timeOfStart;
  }
}



void loop(){

  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  //pos1 = pos_i1;
  //pos2 = pos_i2;
  interrupts();

  // ORDER: TO GET FROM I2C. The structure is: {V1,DIR1,V2,DIR2}. Speed in RPM, max 150 (more like 145).
  float order[4] = {120.0,0,120.0,0};
  for(int k = 0; k < 4; k++){
    target[k] = order[k];
  }
  
  // Compute velocity
  computeVelocityAndController<0>();
  computeVelocityAndController<1>();
  //computeVelocityAndController(pos1, prevT[0], posPrev[0], target[0], 0);
  //computeVelocityAndController(pos2, prevT[1], posPrev[1], target[1], 1);

  // For debugging purposes.
  Serial.print("Motor1: ");
  Serial.print(pos[0]);
  Serial.print(" & Motor2: ");
  Serial.print(pos[1]);
  Serial.println();

  // 0.5ms delay: to maintain correct sampling frequency of 2000 Hz.
  delayMicroseconds(500);  
}

// *********************************************** //
// ************* VELOCITY & CONTROL ************** //
// *********************************************** //
template <int k> // I can use prevT[k], posPrev[k], target[k], pos[k] et on s'en fout d'index.
void computeVelocityAndController(){
  long currT    = micros();
  float deltaT  = ((float) (currT-prevT[k]))/1.0e6;
  int diff      = -(pos[k] - posPrev[k]);
  float v       = (diff*60.0)/(PPR*GB_RATIO*deltaT); // The *60.0 is here to get the speed in RPM and not in RPS.

  posPrev[k]  = pos[k];
  prevT[k]    = currT;
  vFilt[k]    = 0.9984*vFilt[k] + 0.00078*v + 0.00078*vPrev[k];
  vPrev[k]    = v;

  float targetVel = target[2*k];
  float e = targetVel - vFilt[k];

  if(fabs(targetVel) > 90.0){
    eintegral[k] += e*deltaT;

    if(eintegral[k] > 50.0){
      eintegral[k] = 50.0;
    }
    float integralTerm = ki*eintegral[k];

    if(integralTerm > 200.0){
      integralTerm = 200.0;
    }

    // Compute the necessary command to reach the desired speed;
    float u = kp*e + integralTerm; 
    output[k] = fabs(u);
    if(output[k] > 255){
      output[k] = 255;
    }
    
    // Here, it was checking the sign of u. Beware: maybe it is a problem that I don't.

    // Set command
    output[k] = (int) output[k]; // Casting a float to an int results in a rounding to the lower unit: e.g.: 200.9 -> 200.
    // Format: setMotor(direction, targetSpeed, directionPin, enablePin);
    setMotor(target[2*k+1], output[k], dir[k], en[k]);
  }
  else {
    setMotor(target[2*k+1], 0, dir[k], en[k]);
  }

  // End the loop, update the error.
  eprev[k] = e;
}

/* Initial version without the template.
void computeVelocityAndController(int pos,long prevTime, int posPrevious, float target, int index){
  Serial.println(message[index]);
  long currT = micros();
  float deltaT = ((float) (currT-prevTime))/1.0e6;
  int diff = -(pos - posPrevious);
  float v = (diff*60.0)/(PPR*GB_RATIO*deltaT); // Could add * 60 to be in RPM.
  
  posPrev[index] = pos;
  prevT[index] = currT;
  vFilt[index] = 0.9984*vFilt[index] + 0.00078*v + 0.00078*vPrev[index];
  vPrev[index] = v;
  
  controller(deltaT, target, index);
}

void controller(float deltaT, float target, int index){
  float e = target - vFilt[index];
  int dir = 1;  // Motor1: 1 = forward, 0 = backward. Motor2: the opposite.

  if(fabs(target) > 90){
    // Controller  
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
*/

// *********************************************** //
// ************** ENCODERS READING *************** //
// *********************************************** //

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

/* Without template version.
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


// *********************************************** //
// ************* SET MOTOR FUNCTION ************** //
// *********************************************** //

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}