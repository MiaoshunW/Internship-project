/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>                     //create an RF24 object
RF24 radio(10,9);                     // CE, CSN
const byte address[6] = "00001";      //address through which two modules communicate.
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, address);  //set the address
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();             //Set module as receiver
}
void loop() {
  if (radio.available()) {
    char text[34]="";                  // the data type must be the same as receiver
    radio.read(&text, sizeof(text));   // send the text
    Serial.println(text);
  }
  delay(1000);
}
