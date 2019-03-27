#include <Servo.h> 

#define servo_gear_pin 3
#define servo_up_pin 5

Servo servo_gear; 
Servo servo_up;

float gear_first = 100;
float up_first = 35;
int up_sign = 0, gear_sign = 0;
float input_gear=100,input_up=35,b_input_gear=100,b_input_up=35;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("start");
  servo_gear.attach(servo_gear_pin);
  servo_up.attach(servo_up_pin);
  servo_gear.write(gear_first);
  servo_up.write(up_first);
  delay(15);
}

void loop(){
  while(Serial.available()){
    input_gear = Serial.parseFloat();
    input_up = Serial.parseFloat();
    Serial.setTimeout(10);
    up_sign = 0;
    gear_sign = 0;
    if(input_up>=35)up_sign=1;
    if(input_gear>0)gear_sign=1;
  }
  servo_control();
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
