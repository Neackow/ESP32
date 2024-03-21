#define PIN_EN 18  // My motor has these pins for the encoders. Check with ESP datasheet to find out which ones I can use.
#define PIN_DIR 19
#define PIN_SA 17
#define PIN_SB 16


#define GB_RATIO 53     // GEARBOX RATIO
#define filter_size 50

#define cmd_min 15
#define windup_lim 1.5
#define Kp 10
#define Ki 0
//#include <UART.h>


char raw_in[20];
float ref = 0;

int counter = 0;
uint32_t t_speed;
uint32_t t_controller;

float speed;
float raw_speed;
float raw_command;

float speed_list[filter_size];
int list_index = 0;
float command_list[filter_size];
int list_index_cmd = 0;
float weights[filter_size];
float total_weight = 0;

float ei = 0;
int command=0;

void setup() {
  Serial.begin(9600);
  
  for (int i=0; i<filter_size; i++) {
    speed_list[i] = 0;                  // Setting up the filter behaviour.
    weights[i] = pow(0.999,i);
    total_weight += weights[i];
  }
  
  // Setting pin modes.
  pinMode(PIN_SA, INPUT);
  pinMode(PIN_SB, INPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
  attachInterrupt(PIN_SA,encoder_R,RISING);
  attachInterrupt(PIN_SA,encoder_F,FALLING); 

  t_speed = micros();       // Returns the number of microseconds since the Arduino board began running the current program.
  t_controller = micros();
  Serial.print("hello");
  digitalWrite(PIN_DIR,0);  // This controls the PIN either in HIGH or LOW.
  analogWrite(PIN_EN,0);    // This allows to set a more "variable" value, to, e.g., set a LED to half its power.
}

void loop() {
  input();
  tachometer();
  controller();
  data_printer();

}

void input(){
  if (Serial.available()>0){                              // Get the number of bytes (characters) available for reading from the serial port.
    byte m = Serial.readBytesUntil('\n', raw_in, 20);  // reads characters from the serial buffer into an array. The function returns the characters up to the last character before the supplied terminator.
    raw_in[m] = '\0';                                  //insert null character in place of the \n, I believe?
    //atof(raw_input);                        //converts string to int
  }
  ref = 2.0;
}

void encoder_R(){
  int sb = digitalRead(PIN_SB); // Reminder: an encoder works with 2 square waves: A and B. If A rises before B, you are going forward. Else, backward. 
  if(sb>0) counter++;           // So: when A is rising, if B is already UP, it means A is AFTER B and so, we are going backward, so, counter--.
  else counter--;
}

void encoder_F(){
  int sb = digitalRead(PIN_SB);
  if(sb>0) counter--;
  else counter++;
}

void tachometer(){
  uint32_t new_t_speed = micros();
  uint32_t dt = new_t_speed - t_speed;
  t_speed = new_t_speed;
  raw_speed = float(1.0e6*counter)/(3*GB_RATIO*dt); // 1.000.000 is to cancel the fact that dt is expressed in µs. *60 for RPM
  // This means that, since it's 150 RPM MAX for 6V, we need to put a ref of 2.5 max. 
  counter = 0;
  filter();
}

void filter(){                          // Seems to average over 30 (filter_size) measures to have the least impacted by noise version of the speed.
  speed_list[list_index] = raw_speed;
  float sum = 0;
  for(int i=0; i<filter_size;i++){
    sum += speed_list[(i+list_index)%filter_size]*weights[i];
  }
  speed = float(sum)/total_weight;
  list_index = (list_index+1)%filter_size;
}

void filter_command(){                          // Seems to average over 30 (filter_size) measures to have the least impacted by noise version of the speed.
  command_list[list_index_cmd] = raw_command;
  float sum = 0;
  for(int i=0; i<filter_size;i++){
    sum += command_list[(i+list_index_cmd)%filter_size]*weights[i];
  }
  command = (int) float(sum)/total_weight;
  list_index_cmd = (list_index_cmd+1)%filter_size;
}

void controller(){
  uint32_t new_t_controller = micros();
  uint32_t dt = new_t_controller - t_controller;
  t_controller = new_t_controller;

  float e = ref - speed;          // Error
  ei += e*(float(dt)/10000000);

  if(ei>windup_lim) ei = windup_lim;
  else if (ei<-windup_lim) ei = -windup_lim;


  command = int(Kp*(1+pow(float(ref/10),2))*e + Ki*ei);
  //filter_command();
  //if (abs(command)<cmd_min/(abs(speed)+0.5)) command = 0;
  if(command>255) command = 255;
  else if(command<-255) command = -255;

  analogWrite(PIN_EN,0);
  digitalWrite(PIN_DIR,command>0);
  analogWrite(PIN_EN,abs(command));

}

void data_printer(){
  Serial.print(speed);
  Serial.print(",");
  Serial.print(ref);
  /*Serial.print(",");
  Serial.print(ei);
  Serial.print(",");
  Serial.print(command);
  Serial.print(",");
  Serial.print(15);
  Serial.print(",");
  Serial.println(-15);*/
  Serial.println();
}