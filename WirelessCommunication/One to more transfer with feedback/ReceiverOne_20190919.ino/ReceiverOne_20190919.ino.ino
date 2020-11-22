// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN  10
RF24 radio(CE_PIN, CSN_PIN);             // CE, CSN
  
const byte slaveAddress[][6]= {"00011","00022","00033","00044"};            //Receiver address is used to send data by Master
const byte masterAddress[5]= {'T','X','a','a','a'};                         //Master address is used to receive data by Master

double dataReceived[2];                                                     // Receive data tpye must be the same as send data type                                       
double dataSend[2]={209,210};


unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis =100 ;
bool newData =false;
bool rlst ;

//===========

void setup() {

    Serial.begin(9600);
    Serial.println("SimpleRx Starting");
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.openWritingPipe(masterAddress);
    radio.openReadingPipe(1,slaveAddress[1]);
    radio.setRetries(3,5);
    radio.startListening();
}


//=========================Main Loop================================================
void loop() {
      currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
    getData();
    prevMillis = millis();
    }
   senddata();

}

//=========================Receive module================================================

void getData()
{           
  if ( radio.available() ) 
    {
        radio.read( & dataReceived, sizeof(dataReceived) );
        radio.stopListening(); 
        newData = true;       
    }
     if (newData == true) 
     {  
      Serial.print("Data received ");
     Serial.print(" motor1:");
     Serial.print(dataReceived[0]);
     Serial.print("  ,  ");
     Serial.print("motor2:");
     Serial.println(dataReceived[1]);
     Serial.println("  ");
     }         
}

//=========================Tranmitter module================================================
void senddata()
{ 
  if (newData == true) 
  {    
     radio.stopListening();                        //  stop to receive
     rlst=radio.write(&dataSend, sizeof(dataSend));//  start to sent data
     radio.startListening();                        // start to receive
     if (rlst==1)                                  // if data is sent successfully
     {
                                                  
     Serial.println("Reply sent ");
     Serial.print("from motor1:");
     Serial.print(dataSend[0]);
     Serial.print("  ,  ");
     Serial.print("from motor2:");
     Serial.println(dataSend[1]);
     Serial.println("  ");
     }
  }
}
