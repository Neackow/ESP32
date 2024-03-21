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

#define GB_RATIO    53  // Gearbox ratio of the motor. In my case - 53:1.
#define PPR         3   // Pulse Per Rotation for the encoder. In my case, 3.
#define filter_size 20  // For the filter : can change at will.
#define windup_lim 1.5  // To deal with the windup.

// Store the PID variables
#define kp 2
#define ki 10
#define kd 0

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

// Filter commands
float speed_list[filter_size];
int list_index = 0;
float weights[filter_size];
float total_weight = 0;

float e = 0.0;

void setup() {
  Serial.begin(9600);

  for (int i=0; i<filter_size; i++) {
    speed_list[i] = 0;                  // Setting up the filter behaviour.
    weights[i] = pow(0.999,i);
    total_weight += weights[i];
  }

  pinMode(SA1,INPUT);
  pinMode(SB1,INPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(EN1,OUTPUT);
/*
  pinMode(SA2,INPUT);
  pinMode(SB2,INPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(EN2,OUTPUT);*/

  attachInterrupt(SA1,encoder_R1,RISING);
  attachInterrupt(SA1,encoder_F1,FALLING); 
  //attachInterrupt(digitalPinToInterrupt(SA2),readEncoder2,RISING);
}

void loop() {
  // read the position in an atomic block to avoid potential misreads
  int pos1 = 0;
  //int pos2 = 0;

  // Since ATOMIC_BLOCK is unavailable on ESP32, here is the other solution.
  noInterrupts(); // disable interrupts temporarily while reading
  pos1 = pos_i1;
  //pos2 = pos_i2;
  interrupts(); // turn interrupts back on

  //float target[2] = {200,100};
  float target = 100.0;   // MAXIMUM 150 RPM.
  float u1 = pidController1(target, pos1);
  //float u2 = pidController2(target[1], pos2);

  // Set the motor speed and direction
  int dir1 = 1;
  //int dir2 = 1;
  if (u1<0){      // PROBLEM: it changes direction and just stays this way. 
  // Something is super odd: the speed remains the same, but printing the u1, we see that it is negative. It is a smaller value than initially. To keep the speed as it is, the speed value should not change.
    dir1 = 0;
  }
  /*
  if (u2<0){
    dir2 = 0;
  }*/

  // Control over 8 bits.
  float speed1 = fabs(u1);
  float pwm_command1 = rpm_to_PWM(speed1);
  if(pwm_command1 > 255){
    pwm_command1 = 255;
  }
  /*
  int speed2 = (int) fabs(u2);
  if(speed2 > 255){
    speed2 = 255;
  }*/

  setMotor(dir1,pwm_command1, EN1, DIR1);
  //setMotor(dir2,speed2, EN2, DIR2);

  Serial.print("  target:");
  Serial.print(target); // In RPM
  //Serial.print(speed2);
  Serial.print("  v1Filt:");
  /*Serial.print(pwm_command1); // PWM command
  Serial.print(" ");
  Serial.print(v1Prev); // In RPM
  Serial.print(" ");*/
  Serial.print(v1Filt); // In RPM
  Serial.print("  u1:");
  Serial.print(u1);     // Should become negative
  Serial.print("  pwm_command1:");
  Serial.print(pwm_command1);
  //Serial.print(v2Filt);
  Serial.println();
  delay(1);
}

float rpm_to_PWM(float command){
  float PWM_value = (255.0/150.0) * command; // At max, my motor goes 150 RPM, which equates 255 PWM command. So, if max speed, then we get max PWM.
  return PWM_value;
}


void setMotor(int dir, int pwmVal, int PIN_EN, int PIN_DIR){
  analogWrite(PIN_EN,pwmVal); // Motor speed
  digitalWrite(PIN_DIR,dir);
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

void encoder_R2(){            // Reminder: if A is triggered Rising but B is already high, it means B first, so going backward. And vice-versa.
  int sb = digitalRead(SB2);  
  if(sb>0) pos_i1--;           
  else pos_i1++;
}

void encoder_F2(){
  int sb = digitalRead(SB2);
  if(sb>0) pos_i2++;
  else pos_i2--;
}


float pidController1(float target, int pos){
  // Compute velocity. We are in seconds.
  long currT = micros();
  float deltaT = ((float) (currT-prevT1))/1.0e6;
  float velocity = -(pos - posPrev1)/deltaT;

  // Convert count/s to RPM
  float v1 = (velocity/(GB_RATIO*PPR))*60.0; // Raw speed in RPM.

  speedFilter(v1); // Compute the filtered version of v1.
  
  // Low-pass filter (25 Hz cutoff)
  //v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  //v1Prev = v1;  

  // Compute the control signal u
  e = target-v1Filt;
  eintegral1 = eintegral1 + e*deltaT;

  if(eintegral1>windup_lim) eintegral1 = windup_lim;
  else if (eintegral1<-windup_lim) eintegral1 = -windup_lim;

  float eDerivative = (e - ePrevious1) / deltaT;
  
  float u = kp*(1+pow(float(target/1000.0),2))*e + ki*eintegral1 + kd*eDerivative;

  posPrev1 = pos;
  prevT1 = currT;
  ePrevious1 = e;

  return u;
}

void speedFilter(float v_in){                          // Seems to average over 30 (filter_size) measures to have the least impacted by noise version of the speed.
  speed_list[list_index] = v_in;
  float sum = 0;
  for(int i=0; i<filter_size;i++){
    sum += speed_list[(i+list_index)%filter_size]*weights[i];
  }
  v1Filt = float(sum)/total_weight;
  list_index = (list_index+1)%filter_size;
}

float pidController2(float target, int pos){
  // Compute velocity with method 1. We are in seconds.
  long currT = micros();
  float deltaT = ((float) (currT-prevT2))/1.0e6;
  float velocity = -(pos - posPrev2)/deltaT;

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