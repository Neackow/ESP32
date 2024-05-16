// Adapted from Curio Res' code: https://github.com/curiores/ArduinoTutorials/blob/main/encoderControl/part3/part3.ino

#define ENCA 8
#define ENCB 9
#define PWM 7
#define DIR 6

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

const long timer = 1000;
long currentTime = 0;
long lastTime = 0;
int pos = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),changeEncoder,CHANGE);
  
  pinMode(PWM,OUTPUT);
  pinMode(DIR,OUTPUT);
  lastTime = micros();
}

void loop() {

  currentTime = micros();

  noInterrupts();
  pos = posi;
  interrupts();

  Serial.println(pos);

}

void setMotor(int dir, int pwm_value, int PIN_DIR, int PIN_EN){
  delay(5);
  digitalWrite(PIN_DIR, dir);
  analogWrite(PIN_EN, pwm_value);
}

void changeEncoder(){
  int a = digitalRead(ENCA);
  if(a == HIGH){ 
    encoder_R();
  }
  else encoder_F();
}

void encoder_R(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi--;
  }
  else{
    posi++;
  }
}

void encoder_F(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}