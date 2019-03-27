#include "Adafruit_TCS34725.h"
/*tcs에 센서를 선언*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
#include <Servo.h>
//pwm pin = 3,5,6,9,10,11
#define servo_gear_pin 3
#define servo_up_pin 5

#define motorL 6
#define motorL_F 8
#define motorL_B 7
#define motorR_F 9
#define motorR_B 10
#define motorR 11

#define TRIG 12
#define ECHO 13

#define ir1 2
#define ir2 A0
#define ir3 A1
#define ir4 A2
#define ir5 A3
#define ir6 4

Servo servo_gear;
Servo servo_up;

/*******************서보모터**************************/
float gear_first = 120;
float gear_limit = 23;
float up_first = 36;
float up_limit = 57;
/*******************인풋**************************/
int up_sign=0, gear_sign=0;
float input_gear=gear_first, input_up=up_first, b_input_gear=gear_first, b_input_up=up_first;
float L_output=0, R_output=0;
float L_PWM=0, R_PWM=0;
float error=0;
float start_time=0,loop_time=0;
int ir1_a,ir2_a,ir3_a,ir4_a,ir5_a,ir6_a;
int ir1_s,ir2_s,ir3_s,ir4_s,ir5_s,ir6_s;
float r=0,g=0,b=0;
float rps4_Lm = -63, rps4_Rm = -48;
float rps3_Lm = -52, rps3_Rm = -39;
float rps2_Lm = -42, rps2_Rm = -32;
float rps1_Lm = -32, rps1_Rm = -26;
float rps1_L = 30, rps1_R = 25;
float rps2_L = 39, rps2_R = 32;
float rps3_L = 47, rps3_R = 37;
float rps4_L = 60, rps4_R = 48;
float rps5_L = 79, rps5_R = 58;
float rps6_L = 107, rps6_R = 75;
String total_state, drive_state;
/**********************************/
float Kp=7;
float Ki=0.5;
float Kd=1;
float L_I_val=0,L_D_val=0,R_I_val=0,R_D_val=0;
float distance = 10;
int T_count=0,Acount=0,Bcount=0,first_drive_count=0,linecase=0;
int check_start=0;
int grabMode=0;
float case_d=6.5;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  /*라인센서*/
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  /*초음파센서*/
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  /*모터*/
  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(motorL_F, OUTPUT);
  pinMode(motorL_B, OUTPUT);
  pinMode(motorR_F, OUTPUT);
  pinMode(motorR_B, OUTPUT);
  digitalWrite(TRIG,LOW);
  digitalWrite(ECHO,LOW);
  servo_gear.attach(servo_gear_pin);
  servo_up.attach(servo_up_pin);
  servo_gear.write(gear_first);
  servo_up.write(up_first);
  digitalWrite(motorR_F,LOW);
  digitalWrite(motorR_B,LOW);
  analogWrite(motorR,0);
  digitalWrite(motorL_F,LOW);
  digitalWrite(motorL_B,LOW);
  analogWrite(motorL,0);
  tcs.begin();
  tcs.setInterrupt(true);
  delay(15);
  Serial.println("start");
}

void loop() {
  while(check_start==0){
    sensor_state();
    L_PWM=rps4_L;
    R_PWM=rps4_R;
    motor_control();
    if(drive_state=="0100"||drive_state=="0110"||drive_state=="0010")check_start=1;
  }
  start_time = micros();
  sensor_state();
  drive();
  
  distance = ultra_sonic();
  if(Acount==0)case_d=9.5;
  else if(Acount==1 && Bcount==0)case_d=6.5;
  else if(Acount==1 && Bcount==1)case_d=9.5;
  if(distance<case_d && grabMode==0){
    stop_bot();
    ultra_sonic();
    delay(30);
    if(Acount==0){
      grab();
      RGB_sense();
      if(r>83 && b <85){        
        missionA_red();
      }
      else if(r<=83 && b>=85-distance){
        missionA_blue();
      }
      Acount=1;
    }
    else grab();
  }
  T_check();
  /*
  Serial.print("IR1:"+String(ir1_a));
  Serial.print(" IR2:"+String(ir2_a));
  Serial.print(" IR3:"+String(ir3_a));
  Serial.print(" IR4:"+String(ir4_a));
  Serial.print(" IR5:"+String(ir5_a));
  Serial.println(" IR6:"+String(ir6_a));
  Serial.println(" Distance:"+String(distance));
  */
  loop_time = micros() - start_time;
}

void T_check(){
    if(String(ir1_s)+String(ir2_s)+String(ir3_s)+String(ir6_s)=="1110"){
    delay(150);left_turn();
    if(T_count==0){
      L_PWM=rps3_Lm-20;
      R_PWM=rps3_Rm-16;
      motor_control();
      delay(150);
      distance=15;
      L_PWM=rps4_L-5;
      R_PWM=rps4_R;
      motor_control();
      delay(50);
      sensor_state();
    }
  }
  if((String(ir1_s)+String(ir6_s)=="11")||(String(ir1_s)+String(ir5_s)=="11")||(String(ir2_s)+String(ir6_s)=="11")){
    if(T_count==0){
      if(Bcount==0){
        //Serial.println("BBB");
        L_PWM=rps4_L;
        R_PWM=rps4_R;
        motor_control();
        delay(50);
        stop_bot();
        ungrab();
        Bcount=1;
      }
      T_count+=1;
      L_PWM=rps3_Lm-23;
      R_PWM=rps3_Rm-18;
      motor_control();
      delay(600);
      if(Bcount==0)redLeftTurn();
      else if(Bcount==1)left_turn();
    }
    else if(T_count==1){
      delay(150);left_turn();
      distance=15;
      L_PWM=rps4_L;
      R_PWM=rps4_R;
      motor_control();
      delay(10);
    }
  }
}

void drive(){
  /*왼쪽이 더 약함*/
  float want_rps_L,want_rps_R;
  /*
  if(drive_state="0000"&&first_drive_count==0){
    while(1){
      first_drive_count=1;
      L_PWM=rps4_L;
      R_PWM=rps4_R;
      motor_control();
      sensor_state();
      if(drive_state!="0000")break;
    }
  }*/
  if(drive_state=="1000"){
    error=-3; want_rps_L=rps4_L; want_rps_R=rps4_R; Kp=9;
    if(Bcount>=1&&grabMode==1){
      want_rps_L=rps4_L+20;
      want_rps_R=rps5_R+25;
      up_limit=55;
    }
  }
  else if(drive_state=="1100"){
    error=-2; want_rps_L=rps4_L; want_rps_R=rps4_R; Kp=9;
    if(Bcount>=1&&grabMode==1){
      want_rps_L=rps4_L+20;
      want_rps_R=rps5_R+25;
      up_limit=55;
    }
  }
  else if(drive_state=="0100"){
    error=-1; want_rps_L=rps5_L; want_rps_R=rps5_R; Kp=9;
  }
  else if(drive_state=="0110"){
    error=0; want_rps_L=rps5_L; want_rps_R=rps5_R; Kp=10;
  }
  else if(drive_state=="0010"){
    error=1; want_rps_L=rps5_L; want_rps_R=rps5_R; Kp=9;
  }
  else if(drive_state=="0011"){
    error=2; want_rps_L=rps4_L; want_rps_R=rps4_R; Kp=9;
  }
  else if(drive_state=="0001"){
    error=3; want_rps_L=rps4_L; want_rps_R=rps4_R; Kp=9;
  }
  L_PWM = calculate_pid(error,L_I_val,L_D_val,L_output)*1.3 + want_rps_L;
  R_PWM = calculate_pid(-error,R_I_val,R_D_val,R_output) + want_rps_R;
  motor_control();                                                                                  
}

void left_turn(){
  L_PWM = rps3_Lm-25;
  R_PWM = rps5_R;
  motor_control();
  delay(600);
  while(1){
    sensor_state();
    if(total_state=="000110"||total_state=="000100")break;
  }
}

void right_turn(){
  L_PWM = rps3_L+13;
  R_PWM = rps3_Rm-10;
  motor_control();
  delay(1000);
}

float calculate_pid(float error,float &I_val,float &D_val, float output){
  float P_val = error*Kp;
  float previous_error;
  float I_const=20;
  I_val += Ki*error*loop_time/1000000;
  if(I_val>I_const)I_val=I_const;
  else if(I_val<-I_const)I_val=-I_const;
  D_val = (error-previous_error)*Kd*loop_time/1000000;
  output = P_val;
  output += I_val+D_val;
  previous_error=error;
  return output;
}

void motor_control(){
  if(L_PWM>255)L_PWM=255;
  else if(L_PWM<-255)L_PWM=-255;
  if(R_PWM>255)R_PWM=255;
  else if(R_PWM<-255)R_PWM=-255;
  if(L_PWM>=0){
    digitalWrite(motorL_F, HIGH);
    digitalWrite(motorL_B, LOW);
    analogWrite(motorL, abs(L_PWM));
  }
  else if(L_PWM<0){
    digitalWrite(motorL_F, LOW);
    digitalWrite(motorL_B, HIGH);
    analogWrite(motorL, abs(L_PWM));
  }
  if(R_PWM>=0){
    digitalWrite(motorR_F, HIGH);
    digitalWrite(motorR_B, LOW);
    analogWrite(motorR, abs(R_PWM));
  }
  else if(R_PWM<0){
    digitalWrite(motorR_F, LOW);
    digitalWrite(motorR_B, HIGH);
    analogWrite(motorR, abs(R_PWM));
  }
}

void RGB_sense(){
  uint16_t clear,red, green, blue;
  tcs.setInterrupt(false);
  delay(30);
  tcs.getRawData(&red,&green,&blue,&clear);
  tcs.setInterrupt(true);
  uint32_t sum = clear;
  r=red;r/=sum;
  g=green;g/=sum;
  b=blue;b/=sum;
  r*=256;g*=256;b*=256;
}

void ir_read(){
  ir1_a=digitalRead(ir1);
  ir2_a=analogRead(ir2);
  ir3_a=analogRead(ir3);
  ir4_a=analogRead(ir4);
  ir5_a=analogRead(ir5);
  ir6_a=digitalRead(ir6);
  if(ir1_a>=1)ir1_s=1;
  else ir1_s=0;
  if(ir2_a>=100)ir2_s=1;
  else ir2_s=0;
  if(ir3_a>=100)ir3_s=1;
  else ir3_s=0;
  if(ir4_a>=100)ir4_s=1;
  else ir4_s=0;
  if(ir5_a>=100)ir5_s=1;
  else ir5_s=0;
  if(ir6_a>=1)ir6_s=1;
  else ir6_s=0;
}

void sensor_state(){
  ir_read();
  total_state = String(ir1_s)+String(ir2_s)+String(ir3_s)+String(ir4_s)+String(ir5_s)+String(ir6_s);
  drive_state = String(ir2_s)+String(ir3_s)+String(ir4_s)+String(ir5_s);
  Serial.print(total_state);Serial.print("  ");
}

float ultra_sonic(){
  digitalWrite(ECHO,LOW);
  digitalWrite(TRIG,HIGH);
  delay(2);
  digitalWrite(TRIG,LOW);
  return (pulseIn(ECHO,HIGH)*0.034/2);
}

/***************************************************/
void missionA_red(){
  sharpRightTurn();
  stop_bot();
  delay(50);
  ungrab();
  delay(50);
  redLeftTurn();
  delay(50);
  Acount=1;
}

void missionA_blue(){
  sharpLeftTurn();
  stop_bot();
  delay(50);
  ungrab();
  delay(50);
  blueRightTurn();
  delay(50);
  Acount=1;
}

void grab(){
  /*집게 오므리기*/
  for(int i=gear_first; i>gear_limit; i--){
    servo_gear.write(i);
    delay(8);
  }
  /*들어 올리기*/
  for(int i=up_first; i<up_limit; i++){
    servo_up.write(i);
    delay(5);
  }
  grabMode=1;
}

void ungrab(){
  /*내리기*/
  for(int i=up_limit; i>up_first; i--){
    servo_up.write(i);
    delay(5);
  }
  delay(10);
  /*집게 풀기*/
  for(int i=gear_limit; i<gear_first; i++){
    servo_gear.write(i);
    delay(5);
  }
  grabMode=0;
}

void stop_bot(){
  digitalWrite(motorL_F, LOW);
  digitalWrite(motorL_B, LOW);
  digitalWrite(motorR_F, LOW);
  digitalWrite(motorR_B, LOW);
  L_I_val=0;
  L_D_val=0;
  R_I_val=0;
  R_D_val=0;
}

void sharpLeftTurn(){
  L_PWM=rps3_Lm-30;
  R_PWM=rps5_R;
  motor_control();
  delay(1100);
}

void sharpRightTurn(){
  L_PWM=rps5_L;
  R_PWM=rps3_Rm-30;
  motor_control();
  delay(1000);
}
void redLeftTurn(){
  L_PWM=rps3_Lm-30;
  R_PWM=rps5_R;
  motor_control();
  delay(700);
  while(1){
    sensor_state();
    if(drive_state=="0100"||drive_state=="0010"||drive_state=="0011"){
      stop_bot()
      ;
      delay(10);
      break;
    }
  }
}

void blueRightTurn(){
  L_PWM=rps5_L;
  R_PWM=rps3_Rm-23;
  motor_control();
  while(1){
    sensor_state();
    if(drive_state=="0011"||drive_state=="0001"){
      stop_bot();
      delay(10);
      break;
    }
  }
}
