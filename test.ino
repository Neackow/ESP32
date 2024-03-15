#define PIN_EN 2
#define PIN_DIR 3
#define PIN_SA 4
#define PIN_SB 5


#define GB_RATIO 19
#define filter_size 30

#define cmd_min 15
#define windup_lim 1.5
#define Kp 40
#define Ki 300
//#include <UART.h>


char raw_input[20];
float ref = 0;

int counter = 0;
uint32_t t_speed;
uint32_t t_controler;

float speed;
float raw_speed;

float speed_list[filter_size];
int list_index = 0;
float weights[filter_size];
float total_weight = 0;

float ei = 0;
int command=0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600,SERIAL_8N1);

  
  for (int i=0; i<filter_size; i++) {
    speed_list[i] = 0;
    weights[i] = pow(0.999,i);
    total_weight += weights[i];
  }
  
  pinMode(PIN_SA, INPUT);
  pinMode(PIN_SB, INPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  attachInterrupt(PIN_SA,encoder_R,RISING);
  attachInterrupt(PIN_SA,encoder_F,FALLING); 

  t_speed = micros();
  t_controler = micros();
  Serial.print("hello");
  digitalWrite(PIN_DIR,0);
  analogWrite(PIN_EN,0);
}

void loop() {
  input();
  tachometer();
  controler();
  data_printer();

}

void input(){
  if (Serial.available()>0){
    byte m = Serial.readBytesUntil('\n', raw_input, 20);
    raw_input[m] = '\0';  //insert null charcater
    ref = atof(raw_input); //converts string to int
  }
}

void encoder_R(){
  int sb = digitalRead(PIN_SB);
  if(sb>0) counter--;
  else counter++;
}

void encoder_F(){
  int sb = digitalRead(PIN_SB);
  if(sb>0) counter++;
  else counter--;
}

void tachometer(){
  uint32_t new_t_speed = micros();
  uint32_t dt = new_t_speed - t_speed;
  t_speed = new_t_speed;
  raw_speed = float(1000000*counter)/(3*GB_RATIO*dt);
  counter = 0;
  filter();
}

void filter(){
  speed_list[list_index] = raw_speed;
  float sum = 0;
  for(int i=0; i<filter_size;i++){
    sum += speed_list[(i+list_index)%filter_size]*weights[i];
  }
  speed = float(sum)/total_weight;
  list_index = (list_index+1)%filter_size;
}

void controler(){
  uint32_t new_t_controler = micros();
  uint32_t dt = new_t_controler - t_controler;
  t_controler = new_t_controler;

  float e = ref - speed;
  ei += e*(float(dt)/10000000);

  if(ei>windup_lim) ei = windup_lim;
  else if (ei<-windup_lim) ei = -windup_lim;


  command = int(Kp*(1+pow(float(ref/10),2))*e + Ki*ei);
  //if (abs(command)<cmd_min/(abs(speed)+0.5)) command = 0;
  if(command>255) command =255;
  else if(command<-255) command =-255;

  analogWrite(PIN_EN,0);
  digitalWrite(PIN_DIR,command>0);
  analogWrite(PIN_EN,abs(command));

}

void data_printer(){
  Serial.print(speed);
  Serial.print(",");
  Serial.print(ref);
  Serial.print(",");
  Serial.print(ei);
  Serial.print(",");
  Serial.print(counter);
  Serial.print(",");
  Serial.print(15);
  Serial.print(",");
  Serial.println(-15);
  Serial1.write(speed);
}