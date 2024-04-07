// Pin definition
const int dir[] = {2,32};  // Direction pins
const int en[]  = {3,33};  // Enable pins: sets PWM
const int sa[]  = {4,25};  // Encoder pins SA: interrupt required
const int sb[]  = {5,26};  // Encoder pins SB: interrupt required

// Motor values.
#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       6   // Pulse Per Rotation for the encoder. In my case, 3x2 since considering RISING & FALLING. 
#define NMOTORS   2

int   pos[NMOTORS]          = {0,0};  // To safely get the value of posi[NMOTORS] within noInterrupt().
volatile int posi[NMOTORS]  = {0,0};  // Update the encoder count.
int   posPrev[NMOTORS]      = {0,0};  // Previous value of the encoder count.

long  prevT_V[NMOTORS]    = {0,0};      // Previous time at which a velocity computation was made.
long  prevT[NMOTORS]      = {0,0};      // Previous time at which a computation was made.
float eprev[NMOTORS]      = {0.0,0.0};  //Previous error in the controller.
float eintegral[NMOTORS]  = {0.0,0.0};  // Store the evolution of the integral error.

float vFilt[NMOTORS]      = {0.0,0.0};  // Filtered speed.
float vPrev[NMOTORS]      = {0.0,0.0};  // Previous speed.
float v[NMOTORS]          = {0.0,0.0};  // Computed speed (live).

// Control command
float output[NMOTORS]   = {0.0,0.0};      // Output of the controller
float target[4]         = {0.0,0,0.0,0};  // Contains the desired motor behaviour.
//const char* message[2]  = {"Test1","Test2"};
unsigned long previousMillis[NMOTORS]  = {0,0};
const long interval     = 20000;
int diff[NMOTORS]  = {0,0};

// PID controller values.
float kp = 10.0;
float ki = 4.0;


void setup() {
  Serial.begin(9600); //115200 is another option.

  for(int k = 0; k < NMOTORS; k++){
    pinMode(dir[k],OUTPUT);
    pinMode(en[k],OUTPUT);
    pinMode(sa[k],INPUT);
    pinMode(sb[k],INPUT);
    analogWrite(en[k], 0);
    digitalWrite(dir[k], 0);
  }
  
  attachInterrupt(digitalPinToInterrupt(sa[0]),changeEncoder<0>,CHANGE);  // It does not work to define 2 attachInterrupt function (RISING and FALLING), only the last is executed. Thus: CHANGE.
  attachInterrupt(digitalPinToInterrupt(sa[1]),changeEncoder<1>,CHANGE);

  // Allows me to have the time to correctly setup the rest of the robot.
  delay(2000);

  for(int k = 0; k < NMOTORS; k++){
    prevT[k] = micros();
    prevT_V[k] = micros();
    previousMillis[k] = micros();
  }
  
}


void loop(){

  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  interrupts();

  // ORDER: TO GET FROM I2C. The structure is: {V1,DIR1,V2,DIR2}. Speed in RPM, max 150 (more like 145).
  float order[4] = {100.0,1,120.0,0};
  for(int k = 0; k < 4; k++){
    target[k] = order[k];
  }
  
  // Compute velocity & control the motors.
  computeVelocityAndController<0>();
  computeVelocityAndController<1>();


  // Print block: for debugging purposes.
  Serial.print(v[0]);
  Serial.print(" ");
  Serial.print(vFilt[0]);
  Serial.print(" ");
  Serial.print(diff[0]);
  Serial.println();

  // 625µs delay: to maintain correct sampling frequency of 1600 Hz.
  // 6 ticks per turn x 53 (Gearbox) x 2.5 RPS (max) = 795 Hz. x2 to avoid aliasing (Shannon's theorem) : ~1600 Hz. 
  delayMicroseconds(625);
}


// *********************************************** //
// ************* VELOCITY & CONTROL ************** //
// *********************************************** //

template <int k> // I can use prevT[k], posPrev[k], target[k], pos[k] et on s'en fout d'index.
void computeVelocityAndController(){
  unsigned long currentMillis = micros(); 
  float deltaT  = ((float) (currentMillis - prevT[k]))/1.0e6;
  prevT[k]      = currentMillis;

  if (currentMillis - previousMillis[k] >= interval){
    previousMillis[k] = currentMillis;
    long currT_V      = micros();
    float deltaT_V    = ((float) (currT_V-prevT_V[k]))/1.0e6;
    //int diff          = -(pos[k] - posPrev[k]);
    diff[k]          = -(pos[k] - posPrev[k]);
    v[k]              = (diff[k]*60.0)/(PPR*GB_RATIO*deltaT_V); // The *60.0 is here to get the speed in RPM and not in RPS.

    posPrev[k]  = pos[k];
    prevT_V[k]  = currT_V;
  }

  vFilt[k]    = 0.9806*vFilt[k] + 0.00973*v[k] + 0.0097*vPrev[k];
  vPrev[k]    = v[k];

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


// *********************************************** //
// ************** ENCODERS READING *************** //
// *********************************************** //

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


// *********************************************** //
// ************* SET MOTOR FUNCTION ************** //
// *********************************************** //

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}