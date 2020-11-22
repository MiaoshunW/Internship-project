/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
                                       //create an RF24 object
RF24 radio(9,10);                      // CE, CSN
const byte address[6] = "00001";       //address through which two modules communicate.
int a=0;
void setup() {
  Serial.begin(9600);
  radio.begin();
  Serial.println("text");
  radio.openWritingPipe(address);     //set the address
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();              //Set module as transmitter
}
void loop() {
  const char text[]="hello wfffff!!";  // the data type must be the same as receiver 
  a=radio.write(&text, sizeof(text));  // send the text
  delay(1000);                         //delay 1s
  if (a==1)                             //a=1 to make sure that text is sent 
  Serial.println(text);
}
