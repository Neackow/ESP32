// ADAPTED FROM https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino

// Define pin numbers for motors
#define SA1  17      // Interrupt pins motor 1
#define SB1  16      // Interrupt pins motor 1
#define DIR1 19      // Direction pin motor 1
#define EN1  18      // PWM pin motor 1

#define SA2  25      // Interrupt pins motor 2
#define SB2  26      // Interrupt pins motor 2
#define DIR2 32      // Direction pin motor 2
#define EN2  33      // PWM pin motor 2


// 32 33 25 26

// globals: store the number of counts read by the encoder.
long prevT1   = 0;
int posPrev1  = 0;
long prevT2   = 0;
int posPrev2  = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i1 = 0;
volatile int pos_i2 = 0;

#define GB_RATIO  53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR       3   // Pulse Per Rotation for the encoder. In my case, 3.

// Store the PID variables
#define kp 10
#define ki 2
#define kd 0.5

// Filter variables
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

// For the PID controller
float eintegral1 = 0;
float ePrevious1 = 0;
float eintegral2 = 0;
float ePrevious2 = 0;


void setup() {
  Serial.begin(115200);

  pinMode(SA1,INPUT);
  pinMode(SB1,INPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(EN1,OUTPUT);

  pinMode(SA2,INPUT);
  pinMode(SB2,INPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(EN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SA1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(SA2),readEncoder2,RISING);
}

void loop() {
  // read the position in an atomic block to avoid potential misreads
  int pos1 = 0;
  int pos2 = 0;

  // Since ATOMIC_BLOCK is unavailable on ESP32, here is the other solution.
  noInterrupts(); // disable interrupts temporarily while reading
  pos1 = pos_i1;
  pos2 = pos_i2;
  interrupts(); // turn interrupts back on

  float target[2] = {200,100};
  float u1 = pidController1(target[0], pos1);
  float u2 = pidController2(target[1], pos2);

  // Set the motor speed and direction
  int dir1 = 1;
  int dir2 = 1;
  if (u1<0){
    dir1 = 0;
  }
  if (u2<0){
    dir2 = 0;
  }

  // Control over 8 bits.
  int speed1 = (int) fabs(u1);
  if(speed1 > 255){
    speed1 = 255;
  }

  int speed2 = (int) fabs(u2);
  if(speed2 > 255){
    speed2 = 255;
  }

  setMotor(dir1,speed1, EN1, DIR1);
  setMotor(dir2,speed2, EN2, DIR2);

  Serial.print(speed1);
  Serial.print(speed2);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(v2Filt);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int PIN_EN, int PIN_DIR){
  analogWrite(PIN_EN,pwmVal); // Motor speed
  digitalWrite(PIN_DIR,dir);
}

void readEncoder1(){
  // Read encoder B when SA rises
  int b = digitalRead(SB1);
  // int increment = 0;
  if(b>0){
    // If B is high, increment forward
    pos_i1++;
  }
  else{
    // Otherwise, increment backward
    pos_i1--;
  }
}

void readEncoder2(){
  // Read encoder B when SA rises
  int b = digitalRead(SB2);
  // int increment = 0;
  if(b>0){
    // If B is high, increment forward
    pos_i2++;
  }
  else{
    // Otherwise, increment backward
    pos_i2--;
  }
}


float pidController1(float target, int pos){
  // Compute velocity. We are in seconds.
  long currT = micros();
  float deltaT = ((float) (currT-prevT1))/1.0e6;
  float velocity = (pos - posPrev1)/deltaT;

  // Convert count/s to RPM
  float v1 = (velocity/(GB_RATIO*PPR))*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;  

  // Compute the control signal u
  float e = target-v1Filt;
  eintegral1 = eintegral1 + e*deltaT;
  float eDerivative = (e - ePrevious1) / deltaT;
  
  float u = kp*e + ki*eintegral1 + kd*eDerivative;

  posPrev1 = pos;
  prevT1 = currT;
  ePrevious1 = e;

  return u;
}

float pidController2(float target, int pos){
  // Compute velocity with method 1. We are in seconds.
  long currT = micros();
  float deltaT = ((float) (currT-prevT2))/1.0e6;
  float velocity = (pos - posPrev2)/deltaT;

  // Convert count/s to RPM
  float v2 = (velocity/(GB_RATIO*PPR))*60.0;

  // Low-pass filter (25 Hz cutoff)
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;  

  // Compute the control signal u
  float e = target-v2Filt;
  eintegral2 = eintegral2 + e*deltaT;
  float eDerivative = (e - ePrevious2) / deltaT;
  
  float u = kp*e + ki*eintegral2 + kd*eDerivative;

  posPrev2 = pos;
  prevT2 = currT;
  ePrevious2 = e;

  return u;
}