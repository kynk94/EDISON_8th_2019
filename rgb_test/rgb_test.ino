#include "Adafruit_TCS34725.h"
/*tcs에 센서를 선언*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

float r,g,b;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  tcs.begin();
  tcs.setInterrupt(true);
  // put your setup code here, to run once:

}

void loop() {
  RGB_sense();
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
  Serial.print(r);Serial.print("\t");
  Serial.print(g);Serial.print("\t");
  Serial.print(b);Serial.println("\t");
}
