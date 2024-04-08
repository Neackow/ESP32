// Pin definition
const int dir[] = {5,18};  // Direction pins
const int en[]  = {4,19};  // Enable pins: sets PWM
const int sa[]  = {3,20};  // Encoder pins SA: interrupt required
const int sb[]  = {2,21};  // Encoder pins SB: interrupt required

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
int diff[NMOTORS]       = {0,0};
int coeffMotor[NMOTORS] = {1,-1};

// Debugging
float e[NMOTORS]  = {0.0,0.0};
float order[4] = {0.0,1,0.0,0};
unsigned long tempsActuel = 0;
float previousDir[NMOTORS] = {0,0};

// PID controller values.
float kp = 10.0;
float ki = 15.0;


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

  if(tempsActuel < 7000){
    order[0] = 0.0;
    order[1] = 1;
    order[2] = 0.0;
    order[3] = 0;
  }
  else if(tempsActuel >= 7000 && tempsActuel < 20000){
    order[0] = 140.0;
    order[1] = 1;
    order[2] = 140.0;
    order[3] = 0;
  }
  else{
    order[0] = 0.0;
    order[1] = 1;
    order[2] = 0.0;
    order[3] = 0;
  }
  // ORDER: TO GET FROM I2C. The structure is: {V1,DIR1,V2,DIR2}. Speed in RPM, max 150 (more like 145).
  
  for(int k = 0; k < 4; k++){
    target[k] = order[k];
  }
  tempsActuel = millis();
  
  // Compute velocity & control the motors.
  computeVelocityAndController<0>();
  computeVelocityAndController<1>();


  // Print block: for debugging purposes.
  Serial.print(vFilt[0]);
  Serial.print(" ");
  Serial.print(target[0]);
  //Serial.print(" ");
  //Serial.print(vFilt[1]);
  /*Serial.print(" ");
  Serial.print(output[0]);
  Serial.print(" ");
  Serial.print(output[1]);
  Serial.print(" ");
  Serial.print(eintegral[0]);
  Serial.print(" ");
  Serial.print(eintegral[1]);
  Serial.print(" ");
  Serial.print(e[0]);
  Serial.print(" ");
  Serial.print(e[1]);
  Serial.print(" ");
  Serial.print(tempsActuel);*/
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
    diff[k]           = coeffMotor[k]*(posPrev[k] - pos[k]); // The coefficient is there to ensure that both motors have positive speed when rotating in different directions.
    v[k]              = (diff[k]*60.0)/(PPR*GB_RATIO*deltaT_V); // The *60.0 is here to get the speed in RPM and not in RPS.

    posPrev[k]  = pos[k];
    prevT_V[k]  = currT_V;
  }

  vFilt[k]    = 0.9806*vFilt[k] + 0.00973*v[k] + 0.0097*vPrev[k];
  vPrev[k]    = v[k];


  // ******************************************** //
  // To deal with the way I'm telling the controller the speed it needs to reach. Instead of sending negative values, I deal automatically with it here.
  int coeffTargetVel = 1;
  if(k == 0){
    if(target[2*k+1] == 0){ // If MOTOR1 : POSITIVE SPEED when DIR1 and NEGATIVE SPEED when DIR0.
      coeffTargetVel = -1;
    }
  } else {
    if(target[2*k+1] == 1){ // If MOTOR2 : POSITIVE SPEED when DIR0 and NEGATIVE SPEED when DIR1.
      coeffTargetVel = -1;
    }
  }
  // ******************************************** //

  float targetVel = coeffTargetVel*target[2*k];
  e[k] = targetVel - vFilt[k];

  // Main control loop.
  if(fabs(targetVel) > 90.0){
    
    // ******************************************** //
    // Hard reset of the integral term when switching direction.
    if(previousDir[k] != target[2*k+1]){
      eintegral[k] = 0.0;
    }
    // ******************************************** //
    
    eintegral[k] += e[k]*deltaT;

    float limit = 70.0;
    if(eintegral[k] > limit){
      eintegral[k] = limit;
    }
    float integralTerm = ki*eintegral[k];

    if(integralTerm > ki*limit){ // So that it automatically adapts itself when I want to change boundary.
      integralTerm = ki*limit;
    }

    // Compute the necessary command to reach the desired speed;
    float u = kp*e[k] + integralTerm; 

    output[k] = fabs(u);
    if(output[k] > 255){
      output[k] = 255;
    }
    int direction = (int) target[2*k+1]; // target is a float list.
    /*if(u < 0){
      direction = 1 - direction; // Change direction if need be, without affecting the initial command. Not necessary anymore, due to how I deal with direction earlier.
    }*/

    // Set command
    output[k] = (int) output[k]; // Casting a float to an int results in a rounding to the lower unit: e.g.: 200.9 -> 200.

    // Format: setMotor(direction, targetSpeed, directionPin, enablePin);
    setMotor(direction, output[k], dir[k], en[k]);
  }
  else {
    setMotor(target[2*k+1], 0, dir[k], en[k]);
  }
  // End the loop, update the error.
  previousDir[k] = target[2*k+1];
  eprev[k] = e[k];
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