#include <Wire.h>

// ********** <Pin definition> **********
const int dir[] = {9,18};  // Direction pins
const int en[]  = {8,19};  // Enable pins: sets PWM
const int sa[]  = {7,20};  // Encoder pins SA: interrupt required
const int sb[]  = {6,21};  // Encoder pins SB: interrupt required
// ********** </Pin definition> **********

// ********** <Motor values> **********
#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       6   // Pulse Per Rotation for the encoder. In my case, 3x2 since considering RISING & FALLING. 
#define NMOTORS   2   // Number of motors to control.

#define TICKSPERTURN  318     // Number of encoder ticks per wheel turn. 6 x 53 (6 ticks per shaft rotation, with the gearbox added to the mix).
#define DISTPERTURN   0.2513  // Distance per wheel turn in [m]: 2xpix0.04 (wheel diameter: 8cm).
#define NUM_TURN_TEST 7.9586  // To accomplish 2 [m], we need this number of turns due to the wheel diameter.  
#define TURN_CRATE    1.15    // Number of wheel turns required to turn the crate by 90°, in either direction, and also on itself. Tuned to limit overshooting (not perfectly 1/4 a circle: 1.78125).
// ********** </Motor values> **********

// ********** <Controller variables> **********
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
float e[NMOTORS]          = {0.0,0.0};  // Current error.
int diff[NMOTORS]         = {0,0};      // Used to store the counter difference.
const long interval       = 18000;      // INITIAL WORKING VALUE: 20000. Used to filter in the controller.

// Control command
float output[NMOTORS] = {0.0,0.0};          // Output of the controller
float target[5]       = {0,0,0,0,0};        // Will be used for the desired motor behaviour. Intermediary variable to have a smooth profile.
int order[5]          = {0,0,0,0,0};        // For manual functioning. 

unsigned long previousMillis[NMOTORS] = {0,0}; // Store previous time.
int coeffMotor[NMOTORS]   = {1,-1};            // To deal with motor direction differences.
unsigned long tempsActuel = 0;                 // Used in manual command.
int previousDir[NMOTORS]  = {0,0};             // Used for hard reset of the integral term.
// ********** </Controller variables> **********

// ********** <Variables for the control modes> **********
int initialValueTS        = 0; // This is to store the initial counter value for this test.
int startingTestingSpeed  = 0; // Not the cleanest way: define an int which allows to know if we just started the mode, since all the rest is updated each loop.

int initialValue          = 0;
int startingTurning       = 0;

int startingSlowingDown   = 0;
int initialValueOutput[2] = {0,0};
long lastTimeSlowingDown  = 0;
long lastTimeEvolveSpeed  = 0;
int storeTargetVel[5]     = {0,0,0,0,0}; // Used to store the values received by I2C. Contains desired final motor behaviour.

int previousProtocol      = 0; // Allows to deal with slowingDown in the testingTheNewSpeed protocol.
int available             = 1; // By default, the controller says that it is available to receive more commands.
// ********** </Variables for the control modes> **********

// ********** <PID constants> **********
float kp = 10.0;    // INITIAL WORKING VALUE: 10.
float ki = 15.0;    // INITIAL WORKING VALUE: 15.
// ********** </PID constants> **********


void setup() {
  Serial.begin(9600);

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
  delay(1000);

  for(int k = 0; k < NMOTORS; k++){
    prevT[k] = micros();
    prevT_V[k] = micros();
    previousMillis[k] = micros();
  }
  
  Wire.begin(0x40);               // Defines the board's slave address.
  Wire.onReceive(receiveMessage); // To receive the command.
  Wire.onRequest(sendMessage);    // To answer when the master wants to read.
}


void loop(){

  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  interrupts();

  // ********** <Manual control of the command: debugging tool> **********
  // Will still work in the loop, because target is set to 0 initially.
  /*
  int speedReq = 120;
  if(tempsActuel < 7000){
    storeTargetVel[0] = 0;
    storeTargetVel[1] = 1;
    storeTargetVel[2] = 0;
    storeTargetVel[3] = 0;
    //order[4] = 0;
  }
  else if(tempsActuel >= 7000 && tempsActuel < 20000){
    storeTargetVel[0] = speedReq;
    storeTargetVel[1] = 1;
    storeTargetVel[2] = speedReq;
    storeTargetVel[3] = 0;
    //order[4] = 0;
  }
  else{
    storeTargetVel[0] = 0;
    storeTargetVel[1] = 1;
    storeTargetVel[2] = 0;
    storeTargetVel[3] = 0;
    //order[4] = 0;
  }
  tempsActuel = millis();*/
  // ********** </Manual control of the command: debugging tool> **********
  
  // The target is acquired via I2C. See function receiveEvent() and in setup().

  // Compute velocity & control the motors. Depending on the fifth argument of target, the desired behaviour varies.
  if(target[4] == 0){ // Comparing a float with integer is internally taken to be comparison between two floats, so it's OK.
    // In the normal behaviour, simply apply the commands. Use evolveSpeed for smooth profile.
    evolveSpeed<0>();
    evolveSpeed<1>();
  } else if (target[4] == 1){
    // Test the newly applied speed.
    testingTheNewSpeed();
  } else if(target[4] == 2){ // This is called whenever we want to stop.
    slowingDownToZero();
  } else if(target[4] == 3){
    // Control mode which will make the crate spin on itself.
    turning();
  } else {
    Serial.println("Invalid order for the crate-on-wheels.");
    for(int k = 0; k < NMOTORS; k++){
      setMotor(0, 1-k, dir[k], en[k]);
    }
  }

  // ********** <Printing block: debugging tool> **********
  Serial.print(output[0]);
  Serial.print(" ");
  Serial.print(output[1]);
  /*Serial.print(" ");
  Serial.print(e[0]);
  Serial.print(" ");
  Serial.print(e[1]);*/
  Serial.println();
  // ********** </Printing block: debugging tool> **********

  // 625 µs delay: to maintain correct sampling frequency of 1600 Hz.
  // 6 ticks per turn x 53 (Gearbox) x 2.5 RPS (max) = 795 Hz. x2 to avoid aliasing (Shannon's theorem) : ~1600 Hz at most.
  // The velocity filter's coefficients have been established for this sampling frequency.
  delayMicroseconds(625);
}


// *********************************************** //
// ************* I2C COMMUNICATION *************** //
// *********************************************** //

void receiveMessage(int howMany) 
{
  for(int i=0; i < howMany; i++){
    storeTargetVel[i] = Wire.read();
    //Serial.println(storeTargetVel[i]);
  }
  //Serial.println();
  target[1] = storeTargetVel[1];  // Direction 1
  target[3] = storeTargetVel[3];  // Direction 2
  target[4] = storeTargetVel[4];  // Command index. storeTargetVel is only used for storage, do not modify its value.
}

void sendMessage()
{
  byte message = byte(available);
  Wire.write(message);
}


// *********************************************** //
// *************** CONTROL MODES ***************** //
// *********************************************** //

// This function is used to have a smooth velocity profile when used continuously. To slow down to 0, refer preferentially to the appropriate function.
template <int j>
void evolveSpeed(){
  long currentTime = micros();
  if(currentTime - lastTimeEvolveSpeed >= interval){ // Re-use the interval of 18 ms. Arbitrary.
    lastTimeEvolveSpeed = currentTime;
    if(target[j] > storeTargetVel[j]){ // Comparison between a float and an int: it will work.
      target[j] = target[j] - 0.5;
    } else if (target[j] < storeTargetVel[j]){
      target[j] = target[j] + 0.5;
    } else {
      target[j] = target[j];
    }
  }
  computeVelocityAndController<j>();
}

// To test the new speed : go forward for 2 [m], then backward over the same distance. Then, stop. We will go further than 2 [m] since we slow down nicely.
// This is quite an ugly implementation. Some kind of FSM would have been better suited for this task. But it works.
void testingTheNewSpeed(){
  if(!startingTestingSpeed){
    available = 0; // We say that we are no longer available.
    startingTestingSpeed = 1;
    initialValueTS = fabs(pos[0]);
  } else if(startingTestingSpeed){
    // NUM_TURN_TEST [turns] comes from the fact that we want 2 [m] and that 1 turn = 0.2513 [m].
    // While not 2 [m] done and going forward, keep going.
    if(fabs(initialValueTS - fabs(pos[0])) < NUM_TURN_TEST * TICKSPERTURN && target[1] == 1){ // Full of fabs() to ensure a count going up, no matter the case.
      evolveSpeed<0>();
      evolveSpeed<1>();
    } else if(fabs(initialValueTS - fabs(pos[0])) >= NUM_TURN_TEST * TICKSPERTURN && target[1] == 1){ // We have gone over 2 [m]. Now, come back.
      previousProtocol += 1;
      slowingDownToZero();
    } else if(fabs(initialValueTS - fabs(pos[0])) < NUM_TURN_TEST * TICKSPERTURN && target[1] == 0){ // We finished slowing down the first time.
      evolveSpeed<0>();
      evolveSpeed<1>();
    } else {
      Serial.println("Testing the new velocity: test done!"); 
      if(vFilt[0] != 0.0){
        previousProtocol += 1; // To avoid going back again in the previous statement.
        slowingDownToZero();
      } else {  // When we return in this loop after slowing down, we enter this and finish this protocol.
        startingTestingSpeed = 0;
        target[4] = 0;
        available = 1;
      }      
    }
  } else {
    Serial.println("Invalid value for startingTestingSpeed.");
  }
}

void slowingDownToZero(){
  if(!startingSlowingDown){
    available = 0; // We say that we are no longer available.
    startingSlowingDown = 1;
    if(target[0] > target[2]){
      target[0] = target[2]; // In case we were turning, we now say that the target is the same speed for both wheels, but the lowest one, in order to already slow down.
    } else {
      target[2] = target[0]; // This will have no impact if the targets were the same.
    }
    target[4] = 2; // This is used when calling this function from another protocol.
    lastTimeSlowingDown = micros();
    // Get the controller started on the new command.
    computeVelocityAndController<0>();
    computeVelocityAndController<1>();
  } else if(startingSlowingDown){
    if(fabs(vFilt[0]) > 5.0){ // Arbitrary boundary on an arbitrary motor, the goal just being to have a safe-band around 0 to be sure to stop fully.
      long currentTime = micros();
      if(currentTime - lastTimeSlowingDown >= interval){ // Re-use the interval of 18 ms. Arbitrary.
        lastTimeSlowingDown = currentTime;
        for(int k = 0; k < 2; k++){
          target[2*k] = target[2*k] - 0.5;  // Slowly decrease the desired velocity. 0.5 has been found empirically, can be tuned to obtained desired results.
        }
      }
      computeVelocityAndController<0>();
      computeVelocityAndController<1>();
    } else {
      Serial.println("Slowing down: done!");
      // Force 0 output and reset vFilt. We are at full stop.
      for(int k = 0; k < 2; k++){
        target[2*k] = 0;
        vFilt[k]    = 0.0;  // Say we are at 0 speed.
        output[k]   = 0.0;  // Force a 0 output.
      }
      if(previousProtocol == 1){
        target[0] = storeTargetVel[0]; // Get back the target velocity before slowing down.
        target[1] = 0;  // Change direction.
        target[2] = storeTargetVel[1];
        target[3] = 1;  // Change direction.
        target[4] = 1;  // If we used "slowingDown" in the testingTheNewSpeed, we get ready to go back to that mode.
        initialValueTS = fabs(pos[0]);  // To exit the other conditions.
      } else if (previousProtocol == 2) { // Don't change the targets. The protocol testingTheNewSpeed is done.
        target[4] = 1;
        previousProtocol = 0; // Reset this.
      } else {
        target[4] = 0;
        available = 1;  // We are available again for new commands. We put it here so that the testingTheNewSpeed protocol can keep going.
      }
      startingSlowingDown = 0;
    }
  } else {
    Serial.println("Invalid value for startingSlowingDown.");
  }
}

// To turn, wheter it be by 90° or to spin on itself.
void turning(){
  if(!startingTurning){
    available = 0;
    startingTurning = 1;
    initialValue = fabs(pos[0]); // This checks which motor we need to take into account. When spinning on itself, this will lead to motor 1. 
    // It doesn't really matter which motor we use to check. Once one of the wheel did its job, I suppose the crate did a full rotation.
    // This can be improved upon if I realise it never turns fully around. 
  } else if(startingTurning){
    if(fabs(initialValue - fabs(pos[0])) < TURN_CRATE * TICKSPERTURN){ 
      computeVelocityAndController<0>();
      computeVelocityAndController<1>(); // Could custom with boundaries that make it slow down and stop at a nice time.
    } else {
      Serial.println("Turning: done!");
      for(int k = 0; k < 2; k++){
        target[2*k] = 0;
        vFilt[k]    = 0.0;  // Say we are at 0 speed.
        output[k]   = 0.0;  // Force a 0 output.
      }
      target[4] = 0; 
      startingTurning = 0;
      available = 1;
    }
  } else {
    Serial.println("Invalid value for startingTurning.");
  }
}


// *********************************************** //
// ************* VELOCITY & CONTROL ************** //
// *********************************************** //

template <int k> // I can use prevT[k], posPrev[k], target[k], pos[k]. No need of an index, thanks to template.
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

  vFilt[k]    = 0.9806*vFilt[k] + 0.00973*v[k] + 0.00973*vPrev[k]; // Low-pass filter at 5 Hz.
  vPrev[k]    = v[k];

  // ******************************************** //
  // To deal with the way I'm telling the controller the speed it needs to reach. Instead of sending negative values, I deal automatically with it here.
  float coeffTargetVel = 1.0;
  if(k == 0){
    if(target[2*k+1] == 0){ // If MOTOR1 : POSITIVE SPEED when DIR1 and NEGATIVE SPEED when DIR0.
      coeffTargetVel = -1.0;
    }
  } else {
    if(target[2*k+1] == 1){ // If MOTOR2 : POSITIVE SPEED when DIR0 and NEGATIVE SPEED when DIR1.
      coeffTargetVel = -1.0;
    }
  }
  // ******************************************** //

  float targetVel = coeffTargetVel*target[2*k];
  e[k] = targetVel - vFilt[k];

  // Main control loop.
  if(fabs(targetVel) > 10.0){   // THIS COULD POSE A PROBLEM.
    
    // ******************************************** //
    // Hard reset of the integral term when switching direction.
    if(previousDir[k] != target[2*k+1]){
      eintegral[k] = 0.0;
    }
    // ******************************************** //
    
    eintegral[k] += e[k]*deltaT;

    float limit = 70.0;
    float integralTerm = ki*eintegral[k];

    // Windup: limit the integral term.
    if(fabs(integralTerm) > ki*limit){ // So that it automatically adapts itself when I want to change boundary.
      if(integralTerm < 0){
        integralTerm = -ki*limit;
      }
      else {
        integralTerm = ki*limit;
      }
    }

    // Compute the necessary command to reach the desired speed;
    float u = kp*e[k] + integralTerm; 

    output[k] = fabs(u);
    if(output[k] > 255){
      output[k] = 255;
    }
    int direction = target[2*k+1]; // target is already in int.
    if(u < 0 && targetVel > 0){
      direction = 1 - direction;  // Change direction if need be, without affecting the initial command. This is necessary to deal with potential overshoots.
                                  // Added a second condition because when we want to go backward, it is NORMAL that the error can be negative. But now, risk: overshoot in the negative, doesn't catch back.
    }

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