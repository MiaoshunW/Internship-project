/*=============================================================================================================
This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).
 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state
 */

 
#include "HX711.h"
#define calibration_factor   -7050.0 //-7050 worked for my 440lb max scale setup
//=========set up three different Scale===============================
#define DOUT  9 //pin for data
#define CLK  10 //pin for clk
//  5v--vcc    gnd 
#define DOUT2 2
#define CLK2  3
#define DOUT3  5
#define CLK3  6
HX711 scale;
HX711 scale2;
HX711 scale3;

float S_1,S_2,S_3,S_4,S_5,S_6;
void setup() {
  Serial.begin(9600);
  Serial.println("HX711 scale demo");
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale2.begin(DOUT2, CLK2);
  scale2.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale2.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale3.begin(DOUT3, CLK3);
  scale3.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale3.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  Serial.println("Readings:");
}

void loop() {
  Serial.print("Reading:scale ");
  S_1=scale.get_units(1); // get average of 20 scale readings，returns a float
  Serial.print(S_1); 
  Serial.print("kg"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();
  
  Serial.print("Reading:scale2    ");
  S_2=scale2.get_units(1);
  Serial.print(S_2); 
  Serial.print("kg"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();

  Serial.print("Reading:scale3 ");
  S_5=scale3.get_units(1);
  Serial.print(S_5); 
  Serial.print("kg"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();
  Serial.println();  
 
}
