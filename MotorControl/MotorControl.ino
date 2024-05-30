// Final version of velocity controller. Nicolas Isenguerre, April-Mai 2024. 
// Inspired from: https://github.com/curiores/ArduinoTutorials

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
#define TURN_CRATE    1.40    // Number of wheel turns required to turn the crate by 90°, in either direction, or on itself. Tuned to limit overshooting (not perfectly 1/4 a circle: 1.78125).
// ********** </Motor values> **********

// ********** <Controller variables> **********
int   pos[NMOTORS]          = {0,0};  // To safely get the value of posi[NMOTORS] within noInterrupt().
volatile int posi[NMOTORS]  = {0,0};  // Update the encoder count.
int   posPrev[NMOTORS]      = {0,0};  // Previous value of the encoder count.

long  prevT_V[NMOTORS]    = {0,0};      // Previous time at which a velocity computation was made.
long  prevT[NMOTORS]      = {0,0};      // Previous time at which a computation was made.
float eprev[NMOTORS]      = {0.0,0.0};  // Previous error in the controller.
float eintegral[NMOTORS]  = {0.0,0.0};  // Store the evolution of the integral error.

float vFilt[NMOTORS]  = {0.0,0.0};  // Filtered velocity.
float vPrev[NMOTORS]  = {0.0,0.0};  // Previous velocity.
float v[NMOTORS]      = {0.0,0.0};  // Computed velocity (live).
float e[NMOTORS]      = {0.0,0.0};  // Current error.
int diff[NMOTORS]     = {0,0};      // Used to store the counter difference.
const long interval   = 18000;      // Used to filter in the controller.

// Control command
float output[NMOTORS] = {0.0,0.0};    // Output of the controller
float target[5]       = {0,0,0,0,0};  // Will be used to get desired motor behaviour. Intermediary variable to have a smooth profile.
int order[5]          = {0,0,0,0,0};  // For manual testing. 

unsigned long previousMillis[NMOTORS] = {0,0}; // Store previous time.
int coeffMotor[NMOTORS]   = {1,-1};            // To deal with motor direction differences.
unsigned long tempsActuel = 0;                 // Used in manual command.
int previousDir[NMOTORS]  = {0,0};             // Used for hard reset of the integral term.
// ********** </Controller variables> **********

// ********** <Variables for the control modes> **********
int initialValueTS            = 0; // This is to store the initial counter value for this test.
int startingTestingVelocity   = 0; // Not the cleanest way: define an int which allows to know if we just started the mode, since all the rest is updated each loop.

int initialValue      = 0;
int startingTurning   = 0;

int startingSlowingDown         = 0;
int initialValueOutput[2]       = {0,0};
long lastTimeSlowingDown        = 0;
long lastTimeEvolveVelocity[2]  = {0,0};
int storeTarget[5]              = {0,0,0,0,0}; // Used to store the values received by I2C. Contains desired final motor behaviour.

int testNewVelocityPhase = 0;   // Allows to deal with slowingDown in the testingTheNewVelocity protocol.
int available            = 1;   // By default, the controller says that it is available to receive more commands.
// ********** </Variables for the control modes> **********

// ********** <PID constants> **********
float kp = 8.0;
float ki = 8.0;
// ********** </PID constants> **********

// ********** <Guard-rail GRiSP2 -> Raspberry> **********
int previousCounterValue  = 0;
int currentCounterValue   = 1;    // Initialise at one, to avoid any problems at the first loop.
const long oneSecond      = 1000; // Will be compared to millis().
long lastGuardRailTime    = 0;    // Used to test out the condition.
// ********** </Guard-rail GRiSP2 -> Raspberry> **********


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
    lastTimeEvolveVelocity[k] = micros();
  }
  lastGuardRailTime = millis();
  
  Wire.begin(0x40);               // Defines the board's slave address.
  Wire.onReceive(receiveMessage); // To receive the command.
  Wire.onRequest(sendMessage);    // To answer when the master wants to read.

  // Has the board started correctly?
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop(){

  // Is the GRiSP2 board still up and running?
  check_GRiSP_is_connected();

  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  interrupts();

  // ********** <Manual control of the command: debugging tool> **********
  
  /*int velocityReq = 100;
  if(tempsActuel < 7000){
    storeTarget[0] = 0;
    target[1] = 1;
    storeTarget[2] = 0;
    target[3] = 0;
    target[4] = 0;
  }
  else if(tempsActuel >= 7000 && tempsActuel < 15000){
    storeTarget[0] = velocityReq;
    target[1] = 1;
    storeTarget[2] = velocityReq;
    target[3] = 0;
    target[4] = 0;
  }
  else{
    storeTarget[0] = 0;
    target[1] = 1;
    storeTarget[2] = 0;
    target[3] = 0;
    target[4] = 2;
  }
  tempsActuel = millis();*/
  // ********** </Manual control of the command: debugging tool> **********
  
  // The target is acquired via I2C. See function receiveEvent() and in setup().

  // Compute velocity & control the motors. Depending on the fifth argument of target, the desired behaviour varies.
  if(target[4] == 0){ // Comparing a float with integer is internally taken to be comparison between two floats, so it's OK.
    // In the normal behaviour, simply apply the commands. Use evolveVelocity for smooth profile.
    evolveVelocity<0>();
    evolveVelocity<1>();
  } else if (target[4] == 1){
    // Test the newly applied velocity.
    testingTheNewVelocity();
  } else if(target[4] == 2){ 
    // This is called whenever we want to stop.
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
  /*Serial.print(tempsActuel);
  Serial.print(" ");
  Serial.print(output[0]);
  Serial.print(" ");
  Serial.print(output[1]);

  Serial.print(vFilt[0]);
  Serial.print(",");

  Serial.print(storeTarget[1]);
  Serial.print(" ");
  Serial.print(storeTarget[2]);
  Serial.print(" ");
  Serial.print(storeTarget[3]);
  Serial.print(" ");
  Serial.print(storeTarget[4]);
  Serial.println();*/
  // ********** </Printing block: debugging tool> **********
  //delay(10); // To have data only every 10 ms. Avoids overloading the textfiles.
}


// *********************************************** //
// ************ CHECK GRISP IS ALIVE ************* //
// *********************************************** //

void check_GRiSP_is_connected(){
  long currentTime = millis();
  if(currentTime - lastGuardRailTime > oneSecond){   // Check every two seconds that the GRiSP is pinging correctly. One ping should arrive every second.
    if(currentCounterValue == previousCounterValue){  // The counter did not evolve. There is a problem. Send the target to 0 and call the slowingDown function.
      for(int k = 0; k < 5; k++){
        storeTarget[k] = 0.0; // We set everything to 0. The crate will not start again after the full stop.
      }
      slowingDownToZero();
    }
    lastGuardRailTime = currentTime;
    previousCounterValue = currentCounterValue;
  }
}


// *********************************************** //
// ************* I2C COMMUNICATION *************** //
// *********************************************** //

void receiveMessage(int howMany) 
{
  if(howMany == 1){
    currentCounterValue += Wire.read(); // Increment the counter by the value sent by the GRiSP2, which is 1.
  } else {
    for(int i=0; i < howMany; i++){
      storeTarget[i] = Wire.read();
      //Serial.println(storeTarget[i]);
    }
    //Serial.println();
    target[1] = storeTarget[1];  // Direction 1
    target[3] = storeTarget[3];  // Direction 2
    target[4] = storeTarget[4];  // command_index. storeTarget is only used for storage of velocity, do not modify its value.
  }
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
void evolveVelocity(){
  int index = 0;
  if(j == 1){
    index = 2;
  }
  long currentTime = micros();
  if(currentTime - lastTimeEvolveVelocity[j] >= interval){ // Re-use the interval of 18 ms. Totally arbitrary, could be more, but less would lead to a rapid evolution that may be hard to follow.
    lastTimeEvolveVelocity[j] = currentTime;
    if(target[index] > storeTarget[index]){ // Comparison between a float and an int: it will work.
      target[index] = target[index] - 0.5;
    } else if (target[index] < storeTarget[index]){
      target[index] = target[index] + 0.5;
    } else {
      target[index] = target[index];
    }
  }
  computeVelocityAndController<j>();
}

// To test the new velocity : go forward for 2 [m], then backward over the same distance. Then, stop. We will go further than 2 [m] since we slow down nicely.
// This is quite an ugly implementation. Some kind of FSM would have been better suited for this task. But it works.
void testingTheNewVelocity(){
  if(!startingTestingVelocity){
    available = 0; // We say that we are no longer available.
    startingTestingVelocity = 1;
    initialValueTS = fabs(pos[0]);
  } else if(startingTestingVelocity){
    // NUM_TURN_TEST [turns] comes from the fact that we want 2 [m] and that 1 turn = 0.2513 [m].
    // While not 2 [m] done and going forward, keep going.
    if(fabs(initialValueTS - fabs(pos[0])) < NUM_TURN_TEST * TICKSPERTURN && target[1] == 1){ // Full of fabs() to ensure a count going up, no matter the case.
      evolveVelocity<0>();
      evolveVelocity<1>();
    } else if(fabs(initialValueTS - fabs(pos[0])) >= NUM_TURN_TEST * TICKSPERTURN && target[1] == 1){ // We have gone over 2 [m]. Now, come back.
      testNewVelocityPhase = 1; // First step done, now slow-down smoothly.
      slowingDownToZero();
    } else if(fabs(initialValueTS - fabs(pos[0])) < NUM_TURN_TEST * TICKSPERTURN && target[1] == 0){  // Back for 2 [m]
      evolveVelocity<0>();
      evolveVelocity<1>();
    } else {
      Serial.println("Testing the new velocity: test done!"); 
      if(vFilt[0] != 0.0){
        testNewVelocityPhase = 2; // Second step done, now slow-down smoothly.
        slowingDownToZero();
      } else {  // When we return in this loop after slowing down, we enter this and finish the protocol.
        startingTestingVelocity = 0;
        storeTarget[0] = 0; // Required to avoid going backward when the protocol has finished.
        storeTarget[2] = 0;
        target[4] = 0;      // Back to 'normal' mode.
        available = 1;
      }      
    }
  } else {
    Serial.println("Invalid value for startingTestingVelocity.");
  }
}

void slowingDownToZero(){
  if(!startingSlowingDown){
    available = 0; // We say that we are no longer available.
    startingSlowingDown = 1;
    if(target[0] > target[2]){
      target[0] = target[2];  // In case we were turning, we now say that the target is the same velocity for both wheels, but the lowest one, in order to already slow down.
    } else {                  // At most, step of 30 RPM in the target, will be barely noticeable by the human eye.
      target[2] = target[0];  // This will have no impact if the targets were the same.
    }
    if(vFilt[0] < 0){ // Correct the order sent by stopCrate (by default, forward directions) if we were going backward.
      target[1] = 0;
      target[3] = 1;
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
        target[2*k]   = 0;
        vFilt[k]      = 0.0;  // Say we are at 0 velocity.
        output[k]     = 0.0;  // Force a 0 output.
        eintegral[k]  = 0.0;
      }
      if(testNewVelocityPhase == 1){
        target[1] = 0;  // Change direction.
        target[3] = 1;  // Change direction.
        target[4] = 1;  // If we used "slowingDown" in the testingTheNewVelocity, we get ready to go back to that mode.
        initialValueTS = fabs(pos[0]); // To exit the other conditions, reset the value here.
      } else if (testNewVelocityPhase == 2) { // Don't change the targets. The protocol testingTheNewVelocity is done.
        target[4] = 1;
        testNewVelocityPhase = 0; // Reset this.
      } else {
        target[4] = 0;
        available = 1;  // We are available again for new commands. We put it here so that the testingTheNewVelocity protocol can keep going.
      }
      startingSlowingDown = 0;
    }
  } else {
    Serial.println("Invalid value for startingSlowingDown.");
  }
}

// To turn, whether it be by 90° or to spin on itself.
void turning(){
  if(!startingTurning){
    available = 0;
    startingTurning = 1;
    initialValue = fabs(pos[0]); // This checks which motor we need to take into account. When spinning on itself, this will lead to motor 1. 
    // It doesn't really matter which motor we use to check. Once one of the wheel did its job, assume the crate did a full rotation.
  } else if(startingTurning){
    if(fabs(initialValue - fabs(pos[0])) < TURN_CRATE * TICKSPERTURN){
      evolveVelocity<0>();
      evolveVelocity<1>();
    } else {
      Serial.println("Turning: done!");
      for(int k = 0; k < 2; k++){
        storeTarget[2*k] = 0;
        target[2*k] = 0;
        vFilt[k]    = 0.0;  // Say we are at 0 velocity.
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

template <int k> // Can use prevT[k], posPrev[k], target[k], pos[k]. No need of an index, thanks to template.
void computeVelocityAndController(){
  unsigned long currentMillis = micros(); 
  float deltaT  = ((float) (currentMillis - prevT[k]))/1.0e6;
  prevT[k]      = currentMillis;

  if (currentMillis - previousMillis[k] >= interval){
    previousMillis[k] = currentMillis;
    long currT_V      = micros();
    float deltaT_V    = ((float) (currT_V-prevT_V[k]))/1.0e6;

    diff[k]           = coeffMotor[k]*(posPrev[k] - pos[k]); // The coefficient is there to ensure that both motors have positive velocity when rotating in different directions.
    v[k]              = (diff[k]*60.0)/(PPR*GB_RATIO*deltaT_V); // The *60.0 is here to get the velocity in RPM and not in RPS.

    posPrev[k]  = pos[k];
    prevT_V[k]  = currT_V;

    vFilt[k]    = 0.894*vFilt[k] + 0.053*v[k] + 0.053*vPrev[k]; // Low-pass filter at 1 Hz, sampling freq of 55.56 Hz -> 56 Hz.
    vPrev[k]    = v[k];
  }


  // ******************************************** //
  // To deal with the way the controller is being told the velocity it needs to reach. Instead of sending negative values, deal automatically with it here.
  float coeffTargetVel = 1.0;
  if(k == 0){
    if(target[2*k+1] == 0){ // If MOTOR1 : POSITIVE VELOCITY when DIR1 and NEGATIVE VELOCITY when DIR0.
      coeffTargetVel = -1.0;
    }
  } else {
    if(target[2*k+1] == 1){ // If MOTOR2 : POSITIVE VELOCITY when DIR0 and NEGATIVE VELOCITY when DIR1.
      coeffTargetVel = -1.0;
    }
  }
  // ******************************************** //

  float targetVel = coeffTargetVel*target[2*k];
  e[k] = targetVel - vFilt[k];

  // Main control loop.
  if(fabs(targetVel) > 10.0){   // This condition, be it kind off useless, helps the controller when the target is 0, so keep it.
    
    // ******************************************** //
    // Hard reset of the integral term when switching direction.
    if(previousDir[k] != target[2*k+1]){
      eintegral[k] = 0.0;
    }
    // ******************************************** //
    
    eintegral[k] += e[k]*deltaT;

    float limit = 70.0;
    float integralTerm = ki*eintegral[k];

    // Limiter: limit the integral term.
    if(fabs(integralTerm) > ki*limit){ // So that it automatically adapts itself when need to change boundary (Ki value, typically).
      if(integralTerm < 0){
        integralTerm = -ki*limit;
      }
      else {
        integralTerm = ki*limit;
      }
    }

    // Compute the necessary command to reach the desired velocity;
    float u = kp*e[k] + integralTerm; 

    output[k] = fabs(u);
    if(output[k] > 255){
      output[k] = 255;
    }
    int direction = target[2*k+1]; // target is already in int.

    // In theory, this block is actually useless. However, since it helped me with several situations without impacting the others, keep it. But when rolling, this is useless.
    // It truly is for debugging purposes, when the crate-on-wheels is not on the floor and that we have to deal with overshoots. Doesn't work with overshoots in negative direction, however.
    if(u < 0 && targetVel > 0){
      direction = 1 - direction;  // Change direction if need be, without affecting the initial command. This is necessary to deal with potential overshoots.
                                  // Added a second condition because when we want to go backward, it is NORMAL that the error can be negative. But now, risk: overshoot in the negative, doesn't catch back.
    }
    
    // Set command
    output[k] = (int) output[k]; // Casting a float to an int results in a rounding to the lower unit: e.g.: 200.9 -> 200.

    // Format: setMotor(direction, targetVelocity, directionPin, enablePin);
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

// On rising edge.
void encoder_R(int index){
  int b = digitalRead(sb[index]);
  if(b > 0){
    posi[index]--;
  }
  else{
    posi[index]++;
  }
}

// On falling edge.
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