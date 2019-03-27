#include <PololuWheelEncoders.h>
#include <Servo.h> 
//pwm pin = 3, 5, 6, 9, 10, 11
#define servo_gear_pin 3
#define servo_up_pin 5

// 1rev = 48 pulse
#define encoderL_A 4
#define encoderL_B 2
#define encoderR_A 12
#define encoderR_B 13

#define motorL 6
#define motorL_F 8
#define motorL_B 7
#define motorR_F 9
#define motorR_B 10
#define motorR 11

#define ECHO 12
#define TRIG 13

#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5

PololuWheelEncoders enc;
Servo servo_gear; 
Servo servo_up;

float gear_first = 100;
float up_first = 35;
float inputL=0, inputR=0;
int enc_i = 1;
int up_sign = 0, gear_sign = 0;
float input_gear=100,input_up=35,b_input_gear=100,b_input_up=35;
float L_C = 0, R_C = 0;
float L_speed = 0, R_speed = 0;
float L_output=0, R_output=0;
int L_PWM=0, R_PWM=0;
float enc_loopL = 0, enc_loopR = 0, b_enc_loopL = 0, b_enc_loopR = 0;
/******************************왼쪽이 더 약함***************************/
float Kp_L = 0.04;
float Ki_L = 0.5;
float Kd_L = 1.2;
float Kp_R = 0.03;
float Ki_R = 0;
float Kd_R = 1;
/*********************************************************************/
float L_I_val = 0, L_D_val = 0;
float R_I_val = 0, R_D_val = 0;

float enc_time = 0,start_time=0, loop_time=0, b_loop_time=0;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  pinMode(ir1,INPUT);
  pinMode(ir2,INPUT);
  pinMode(ir3,INPUT);
  pinMode(ir4,INPUT);
  enc.init(encoderR_A,encoderR_B, encoderL_A, encoderL_B);
  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(motorL_F, OUTPUT);
  pinMode(motorL_B, OUTPUT);
  pinMode(motorR_F, OUTPUT);
  pinMode(motorR_B, OUTPUT);
  Serial.println("start");
  servo_gear.attach(servo_gear_pin);
  servo_up.attach(servo_up_pin);
  servo_gear.write(gear_first);
  servo_up.write(up_first);
  delay(15);
  digitalWrite(motorR_F,HIGH);
  digitalWrite(motorR_B,LOW);
  analogWrite(motorR,0);
  digitalWrite(motorL_F,HIGH);
  digitalWrite(motorL_B,LOW);
  analogWrite(motorL,0);
  Serial.write(0);
  delay(15);
}

void loop(){
  start_time = micros();
  
  while(Serial.available()){
    //inputL = Serial.parseFloat();
    //inputR = Serial.parseFloat();
    input_gear = Serial.parseFloat();
    input_up = Serial.parseFloat();
    Serial.setTimeout(10);
    //enc_i = 1;
    up_sign = 0;
    gear_sign = 0;
    if(input_up>=35)up_sign=1;
    if(input_gear>0)gear_sign=1;
  }
  
  servo_control();
  /*
  enc_read_by_pulse();
  loop_time = micros() - start_time;
  L_speed = L_C*100000 / loop_time;
  R_speed = R_C*100000 / loop_time;
  */
  /*
  if(micros()>3000000){
    inputL=3;
    inputR=3;
  }
  if(micros()>40000000){
    inputL=L_speed;
    inputR=R_speed;
  }
  delay(100);
  
  enc_read_by_time();
  L_PWM = pid_cal(inputL, L_speed, Kp_L, Ki_L, Kd_L, L_I_val, L_D_val, L_output);
  R_PWM = pid_cal(inputR, R_speed, Kp_R, Ki_R, Kd_R, R_I_val, R_D_val, R_output);
  Serial.print("L_PWM:");Serial.print(L_PWM);Serial.print("  ");
  Serial.print("L_RPS:");Serial.print(L_speed);Serial.print("  ");
  Serial.print("R_PWM:");Serial.print(R_PWM);Serial.print("  ");
  Serial.print("R_RPS:");Serial.println(R_speed);
  motor_control();*/
}
void servo_control(){  
  if(input_up<35)input_up=b_input_up;
  if(input_up<35)input_up=35;
  else if(input_up>80)input_up=80;
  if(input_gear==0)input_gear=b_input_gear;
  if(input_gear>100)input_gear=100;
  if(up_sign==1){
    if(b_input_up<input_up){
      for(int i=b_input_up; i<input_up; i++){
        servo_up.write(i);
        delay(5);
      }
    }
    else if(b_input_up>input_up){
      for(int i=b_input_up; i>input_up; i--){
        servo_up.write(i); 
        delay(5);
      }
    }
    up_sign=0;
  }
  if(gear_sign==1){
    if(b_input_gear<input_gear){
      for(int i=b_input_gear; i<input_gear; i++){
        //Serial.print(i);
        servo_gear.write(i);
        delay(5);
      }
    }
    else if(b_input_gear>input_gear){
      for(int i=b_input_gear; i>input_gear; i--){
        servo_gear.write(i);
        //Serial.print(i);
        delay(5);
      }
    }
    gear_sign=0;
  }
  b_input_up = input_up;
  b_input_gear = input_gear;
  servo_up.write(input_up);
  servo_gear.write(input_gear);
}

void motor_control(){
  if(L_PWM>=0 && inputL!=0){
    digitalWrite(motorL_F, HIGH);
    digitalWrite(motorL_B, LOW);
    analogWrite(motorL, abs(L_PWM));
  }
  else if(L_PWM<0 && inputL!=0){
    digitalWrite(motorL_F, LOW);
    digitalWrite(motorL_B, HIGH);
    analogWrite(motorL, abs(L_PWM));
  }
  else if(inputL==0){
    analogWrite(motorL, 0);
    L_I_val = 0;
    L_D_val = 0;
    L_output = 0;
  }
  if(R_PWM>=0 && inputR!=0){
    digitalWrite(motorR_F, HIGH);
    digitalWrite(motorR_B, LOW);
    analogWrite(motorR, abs(R_PWM));
  }
  else if(R_PWM<0 && inputR!=0){
    digitalWrite(motorR_F, LOW);
    digitalWrite(motorR_B, HIGH);
    analogWrite(motorR, abs(R_PWM));
  }
  else if(inputR==0){
    analogWrite(motorR, 0);
    R_I_val = 0;
    R_D_val = 0;
    R_output = 0;
  }
}
int pid_cal(float val,float enc_speed,
            float Kp,float Ki,float Kd, float & I_val, float & D_val, float & output){
  float error = val - enc_speed;
  float previous_error;
  float P_val = Kp * error;
  
  I_val += Ki * error * loop_time/1000000;
  D_val = (error - previous_error) * Kd * loop_time/1000000;
  previous_error = error;
  output += P_val + I_val + D_val;
  if(output>255){
    output=255;
  }
  else if(output<-255){
    output=-255;
  }
  return output;
}

void enc_read_by_pulse(){
  b_enc_loopL = enc_loopL;
  b_enc_loopR = enc_loopR;
  enc_loopL = enc.getCountsAndResetM1();
  enc_loopR = enc.getCountsAndResetM2();
  L_C = (enc_loopL+b_enc_loopL);
  R_C = (enc_loopR+b_enc_loopR);
}

void enc_read_by_time(){
  if(enc_i==1){
    enc_time = start_time;
    b_enc_loopL = enc_loopL;
    b_enc_loopR = enc_loopR;
    enc_i=0;
  }
  /********enc_read**********/
  enc_loopL = enc.getCountsM1();
  if(abs(enc_loopL)>10000000){
    enc_loopL = enc.getCountsAndResetM1();
  }
  enc_loopR = enc.getCountsM2();
  if(abs(enc_loopR)>10000000){
    enc_loopR = enc.getCountsAndResetM2();
  }
  /**************************/
  float limit_time = micros()-enc_time;
  if(limit_time/1000000>0.1){
    //Serial.print(micros());
    L_C = (enc_loopL - b_enc_loopL)/48;
    R_C = (enc_loopR - b_enc_loopR)/48;
    //Serial.print(L_C);Serial.print("  ");Serial.println(R_C);
    L_speed = L_C / (limit_time/1000000); //RPS
    R_speed = R_C / (limit_time/1000000); //RPS
    enc_i=1;
  }
}
