#include <AS5147P.h>

/*Tested on: 
 * STM32 Bluepill (LibMaple Core)
 * Arduino Uno
 */

const uint8_t cs_pin = 3;
const uint16_t zero_position = 0; //set this to whatever is the reading of the zero position 

AS5147P encoder0(cs_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  encoder0.init();
  encoder0.setZeroPosition(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(F("Raw Rotation: "));
  Serial.print(encoder0.getRawRotation());
  Serial.print(F("  Corrected Rotation: "));
  Serial.print(encoder0.getRotation());
  Serial.print(F("  Degrees:"));
  Serial.print(encoder0.getDegree());
  Serial.println();
  
  //Serial.println(encoder0.getGain());
  if(encoder0.error()){
    Serial.print(F("Error Detected!"));
    Serial.println(encoder0.getErrors());
  }
  delay(500);
}
