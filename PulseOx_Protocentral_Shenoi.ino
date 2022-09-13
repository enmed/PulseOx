/*
 * File         PulseOx_Protocentral_Shenoi.ino
 * Date         Aug 15 , 2022
 * The original code is modified from Ashwin Whitchurch (support@protocentral.com)and ThatGuy1000
    https://github.com/Protocentral/protocentral_max86150_ecg_ppg/blob/master/examples/Example1-PPG-Stream-Arduino-Plotter/Example1-PPG-Stream-Arduino-Plotter.ino
    https://github.com/ThatGuy10000/arduino-pulse-oximeter/blob/master/Project_Code_No_IC.ino
 * 
 * Authors      Ian Murray  (imurray@tamu.edu)
 *              Jason Shenoi(jshenoi@tamu.edu)
 *              Evan George (eeg150030@tamu.edu)
 *              James Zhang (jameszha@tamu.edu)
 
  * Materials: 
  *         ProtoCentral MAX86150 Breakout Board https://protocentral.com/product/protocentral-max86150-ppg-and-ecg-breakout-with-qwiic-v2/  
            I2C Qwiic cable female 4 pin with male pin male breadboard jumper wires
* Instructions
  1) Upload the code to your Arduino
  2) Place your finger on the sensor
  3) In the Arduino IDE, Open Tools->'Serial Monitor'
  4) Make sure the drop down is set to 115200 baud
  5) See the SpO2 numerical output
    
  The SpO2 is calculated from the IR and Red waveforms using a R value, based on the DC value wihch is 1024, and the AC values which is constantly changing. This paper oultines the basics of pulse ox https://www.instructables.com/Arduino-Pulse-Oximeter/ 
* Pin connections to arduino
 Qwicc 
    4 -S CL = A5 (or SCL- yellow)
    3 - SDA = A4 (or SDA- blue)
    2 - 5V  = 5V (red)
    1 -GND = GND (black)
    Qwic connector information https://github.com/Protocentral/protocentral_max86150_ecg_ppg/blob/master/hardware/pc_max86150_breakout_qwiic_v1_3.pdf
  
  Hardware Pins connections (Breakout board to Arduino):
  -5V = 5V
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  
 License
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "max86150.h"

#define SENSOR_MAX 300000

MAX86150 max86150Sensor;

//#define debug Serial //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21

uint16_t ppgunsigned16;

float redAC = 0;
float redDC = SENSOR_MAX;
float irAC = 0;
float irDC = SENSOR_MAX;
float rVal;
float SpO2;
const float fitFactor = 0.45; //Factor to multiply by R to fit SO2 to the device


void setup() {
  Serial.begin(57600);
  Serial.println("MAX86150 PPG Streaming Example");

  // Initialize sensor
  if (max86150Sensor.begin(Wire, I2C_SPEED_FAST) == false)
  {
    Serial.println("MAX86150 was not found. Please check wiring/power. ");
    while (1);
  }

  Serial.println(max86150Sensor.readPartID());

  max86150Sensor.setup(); //Configure sensor. Use 6.4mA for LED drive

}

void loop() {
//    if (max86150Sensor.check() > 0){ 
//      ppgunsigned16 = (uint16_t) (max86150Sensor.getFIFORed());
//      Serial.println(ppgunsigned16);
//      float redVal = max86150Sensor.getFIFORed();
//      Serial.println(redVal);
//     }
 
    for(int i=1;i<100;i++){
      if (max86150Sensor.check() > 0){ 
        float redVal = (float)(uint16_t)max86150Sensor.getFIFORed();
        float irVal = (float)(uint16_t)max86150Sensor.getFIFOIR();
    
        if (redVal > redAC){
            redAC = redVal; //Makes redAC the max value for the red LED
          }
        
        if (redVal < redDC){
          redDC = redVal; //Makes redDC the min value for the red LED
        }
        
        if (irVal > irAC){
          irAC = irVal; //Makes irAC the max value for the IR LED
        }
    
        if (irVal < irDC){
          irDC = irVal; //Makes irDC the min value for the IR LED
        }
          delay(10);
        }
    }
//          Serial.println("The actual values of the Red on IR lights are: ");
//          Serial.print(redAC);
//          Serial.print(",");
//          Serial.print(redDC);
//          Serial.print(",");
//          Serial.print(irAC);
//          Serial.print(",");
//          Serial.println(irDC);

    rVal = (redAC / redDC) / (irAC / irDC); //Calculates R for the data
    SpO2 = rVal * fitFactor * (-1.0 / 3.0) + (3.4 / 3.0); //Calculates SpO2 for the data
    
//  Resets the Values for the next iteration  
    redAC = 0;
    redDC = SENSOR_MAX;
    irAC = 0;
    irDC = SENSOR_MAX;
    Serial.println(SpO2); //Prints rVal and Sp02 to the serial monitor/plotter
 
}
