// SimpleTx - the master or the transmitter

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN   9
#define CSN_PIN 10
const byte slaveAddress[][6] = {"00011","00022","00033","00044"};     //Receiver address is used to send data by Master
const byte masterAddress[5] = {'T','X','a','a','a'};                  //Master address is used to receive data by Master
RF24 radio(CE_PIN, CSN_PIN);                                          // Create a Radio
int txNum=0;
boolean stringComplete = false;
double dataSend_A[2]={90000,-80000};                                  //Data send to Slave A
double dataSend_B[2]={80000,-70000};                                  //Data send to Slave B
double dataSend_C[2]={70000,-60000};                                  //Data send to Slave C
double dataSend_D[2]={60000,-50000};                                  //Data send to Slave D
double dataReceived[4];
unsigned long currentMillis;                               
unsigned long prevMillis;
unsigned long txIntervalMillis =10 ;                                   // send once per second  delay time                
bool newData = false;                                                        
         
void setup() {

    Serial.begin(9600);
    Serial.println("SimpleTx Starting");
    radio.begin();                                                    //set radio
    radio.setDataRate( RF24_250KBPS );
    send();
    radio.openReadingPipe(1,masterAddress);                           // open master address to receive data from SlaveA,B,C,D 
    radio.setRetries(3, 5);                                           // delay, count
    prevMillis = millis();
}

//=====================================Main LOOP============================================================

void loop() 
{
    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {    //send data in every txIntervalMillis time 
    send();
    prevMillis = millis();
    }
    getData();
}

//====================================Tranmitter module=====================================================
void send()
{    
   radio.stopListening();                            // close receive pipe 
   bool rlst; 
  if (txNum>3)                                       //make a loop to send data one by one
      txNum=0;
     if (txNum==0)
     {
        //set the address
       radio.openWritingPipe(slaveAddress[0]);       /// SlaveAddress A
       rlst=radio.write(&dataSend_A, sizeof(dataSend_A)); // send the dataA  to Slave A
       radio.startListening();                       // open receive pipe start to receive data from slave
       if (rlst==1)                                  // if data is sent so print
         {
           Serial.print("Data sent:  ");
           Serial.print(dataSend_A[0]);
           Serial.print("  ,  ");
           Serial.println(dataSend_A[1]);
          }
     }
  else if (txNum==1)
     {
     radio.openWritingPipe(slaveAddress[1]);         /// SlaveAddress B
     rlst=radio.write(&dataSend_B, sizeof(dataSend_B));
     radio.startListening();  
       if (rlst==1)
         {
          Serial.print("Data sent:  ");
          Serial.print(dataSend_B[0]);
          Serial.print("  ,  ");
          Serial.println(dataSend_B[1]);
         }
     }  
   else if (txNum==2)
       {
        radio.openWritingPipe(slaveAddress[2]);       /// SlaveAddress C
        rlst=radio.write(&dataSend_C, sizeof(dataSend_C));
        radio.startListening();  
        if (rlst==1)
          {
           Serial.print("Data sent:  ");
           Serial.print(dataSend_C[0]);
           Serial.print("  ,  ");
           Serial.println(dataSend_C[1]);
          }
       }
   else if (txNum==3)
      {
        radio.openWritingPipe(slaveAddress[3]);         /// SlaveAddress D 
        rlst=radio.write(&dataSend_D, sizeof(dataSend_D));
        radio.startListening();  
        if (rlst==1)
        {
        Serial.print("Data sent:  ");
        Serial.print(dataSend_D[0]);
        Serial.print("  ,  ");
        Serial.println(dataSend_D[1]); 
        }
      }  
   txNum=txNum+1;
  }
  
//====================================Receive module=====================================================
void getData() 
{
       
    if ( radio.available() ) {                                // start to receive data from reading pipe
        radio.read( &dataReceived, sizeof(dataReceived) );    //  receive data 
        newData = true;
       }
    if (newData == true)
    {
      Serial.println("Data received:  ");
        Serial.print("EncoderPos1:");
        Serial.print(dataReceived[0]);
        Serial.print("  ,  ");
        Serial.print("EncoderPos2:");
        Serial.println(dataReceived[1]);
        Serial.print("output1:");
        Serial.print(dataReceived[3]);
        Serial.print("  ,  ");
        Serial.print("output2:");
        Serial.println(dataReceived[4]);
        Serial.println();
        newData = false;
        
    }

}
