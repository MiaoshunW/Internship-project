/********this is A board *********/
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PID_v1.h>
//receiver part
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN   9
#define CSN_PIN  10
RF24 radio(CE_PIN, CSN_PIN);
//const byte slaveAddress[][6] = {"00001","00002"};
//long dataReceived;   // this must match dataToSend in the TX
const byte slaveAddress[][6]= {"00011","00022","00033","00044"};
const byte masterAddress[5]= {'T','X','a','a','a'};
double dataReceived[2];
double dataSend[4]={109,110,110,110};
//receiver part
double smotor1;
double smotor2;
bool newData = false;
unsigned long currentMillis;
unsigned long prevMillis,prevMillis2;
unsigned long txIntervalMillis =22000; // send once per second  delay time 
bool rlst ;

//receiver part
#define encodPinA1 2 // Quadrature encoder A pin
#define encodPinB1 4 // Quadrature encoder B pin
#define EnA 5 // Quadrature encoder B pin

#define encodPinA2 3 // Quadrature encoder C pin
#define encodPinB2 7 // Quadrature encoder D pin
#define EnB 6 // Quadrature encoder B pin

#define M1 A0 // PWM outputs to L298N H-Bridge motor driver module
#define M2 A1
#define M3 A2 // PWM outputs to L298N H-Bridge motor driver module
#define M4 A3
double input = 0, output = 0, Position = 0;
double kp =0.05, ki=0.001, kd =0.000; // modify for optimal performance
double input0 = 0, output0 = 0, Velocity = 0;
double kp0 =0.1, ki0=30, kd0 =0.000; // modify for optimal performance

double input2 = 0, output2 = 0, Position2 = 0;
double kp2 =0.05, ki2=0.001, kd2 =0.000; // modify for optimal performance
double input3 = 0, output3 = 0, Velocity2 = 0;
double kp3 =0.01, ki3=30, kd3 =0.000; // modify for optimal performance

long temp;
long cycle=0;
double pwm_last=0;
double sig;
double encoderPos = 0;
double encoderPos2 = 0;
double t,dbuf,timem,buf2,buf1,Speed;
double dbuf2,buf4,buf3,Speed2;

PID myPID(&input, &output, &Position, kp, ki, kd, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPID0(&input0, &output0, &Velocity, kp0, ki0, kd0, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

PID myPID2(&input2, &output2, &Position2, kp2, ki2, kd2, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPID3(&input3, &output3, &Velocity2, kp3, ki3, kd3, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
double ratio=40;   //1220--->12.2cm=48000  40




void setup() {
TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
pinMode(encodPinA1, INPUT_PULLUP); // quadrature encoder input A
pinMode(encodPinB1, INPUT_PULLUP); // quadrature encoder input B
attachInterrupt(0, encoder, FALLING); // update encoder position
pinMode(EnA, OUTPUT);

pinMode(encodPinA2, INPUT_PULLUP); // quadrature encoder input A
pinMode(encodPinB2, INPUT_PULLUP); // quadrature encoder input B
attachInterrupt(1, encoder2, FALLING); // update encoder position
pinMode(EnB, OUTPUT);

myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(1); //defaut 1=200ms Determines how often the PID algorithm evaluates
myPID.SetOutputLimits(-255, 255);
myPID0.SetMode(AUTOMATIC);
myPID0.SetSampleTime(1); //defaut 1=200ms Determines how often the PID algorithm evaluates
myPID0.SetOutputLimits(-255,255);

myPID2.SetMode(AUTOMATIC);
myPID2.SetSampleTime(1); //defaut 1=200ms Determines how often the PID algorithm evaluates
myPID2.SetOutputLimits(-255, 255);
myPID3.SetMode(AUTOMATIC);
myPID3.SetSampleTime(1); //defaut 1=200ms Determines how often the PID algorithm evaluates
myPID3.SetOutputLimits(-255, 255);

Serial.begin(9600);
Serial.println("SimpleRx Starting");
radio.begin();
radio.setDataRate(RF24_250KBPS);
radio.openWritingPipe(masterAddress);
radio.openReadingPipe(1,slaveAddress[0]); //receiver address
radio.setRetries(3,5);
radio.startListening();
}

void loop() { 
   currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) 
    {/////F=10s
    getData();
    //=====================Motor1======================================
    Position= 10000; //*sig;//30degree
    input = encoderPos ;
    myPID.Compute(); 
    
    Position2= 1000; //*sig;//30degree
    input2= encoderPos2 ;
    myPID2.Compute();   
    prevMillis = millis();
    } 
     buf2=buf1;
     buf1=encoderPos;
     dbuf=buf1-buf2;
    Speed=output;//here change the Speed now is 0.5of full speed 
    speedup();
    speeddown();      
    input0=dbuf; 
    myPID0.Compute(); 
    pwmOut(output0);
    //=========================Motor2============================

    buf4=buf3;
    buf3=encoderPos2;
    dbuf2=buf3-buf4;
    Speed2=output2;//here change the Speed now is 0.5of full speed 
    speedup2();
    speeddown2();      
    input3=dbuf2; 
    myPID3.Compute(); 
    pwmOut2(output3);
    //=============================Print out========================
     senddata();
    Serial.print(encoderPos);
    Serial.print("    ");
    Serial.print(output);
    Serial.print("    ");
    Serial.print(dbuf);
    Serial.print("    ");
    Serial.println(output0);
    
    Serial.print(encoderPos2);
    Serial.print("    ");
    Serial.print(output2);
    Serial.print("    ");
    Serial.print(dbuf2);
    Serial.print("    ");
    Serial.println(output3);
    cycle++;  
}
//=====================================Motor1=================================================
void speedup()
{
      //--------Speed Up---------------//
    if(encoderPos<Position*0.005)
    Velocity=Speed*0.08;//maximum 270
    else if(encoderPos<Position*0.01)
    Velocity=Speed*0.12;
    else if(encoderPos<Position*0.015)
    Velocity=Speed*0.14;
    else if(encoderPos<Position*0.02)
    Velocity=Speed*0.18;
    else if(encoderPos<Position*0.025)
    Velocity=Speed*0.24;
    else if(encoderPos<Position*0.03)
    Velocity=Speed*0.26;
    else if(encoderPos<Position*0.035)
    Velocity=Speed*0.3;
    else if(encoderPos<Position*0.04)
    Velocity=Speed*0.34;
    else if(encoderPos<Position*0.058)
    Velocity=Speed*0.38;
    else if(encoderPos<Position*0.06)
    Velocity=Speed*0.4;
    else if(encoderPos<Position*0.065)
    Velocity=Speed*0.42;
    else if(encoderPos<Position*0.07)
    Velocity=Speed*0.44;
    else if(encoderPos<Position*0.075)
    Velocity=Speed*0.46;
    else if(encoderPos<Position*0.08)
    Velocity=Speed*0.48;
    else if(encoderPos<Position*0.085)
    Velocity=Speed*0.5;
    else if(encoderPos<Position*0.09)
    Velocity=Speed*0.6;
    else if(encoderPos<Position*0.105)
    Velocity=Speed*0.7;
    else if(encoderPos<Position*0.12)
    Velocity=Speed*0.8;
    else if(encoderPos<Position*0.15)
    Velocity=Speed*0.9;
    else
    Velocity=Speed;
  }

void speeddown()
{
      
    //--------Speed down---------------//
    if   (encoderPos>Position*0.8 && encoderPos<=Position*0.85)
    Velocity=Speed*0.9;
    else if (encoderPos>Position*0.85  && encoderPos<=Position*0.88)
    Velocity=Speed*0.8;
    else if (encoderPos>Position*0.88  && encoderPos<=Position*0.91)
    Velocity=Speed*0.7;
    else if (encoderPos>Position*0.91  && encoderPos<=Position*0.93)
    Velocity=Speed*0.6;
    else if (encoderPos>Position*0.93  && encoderPos<=Position*0.94)
    Velocity=Speed*0.5;
    else if (encoderPos>Position*0.94 && encoderPos<=Position*0.95)
    Velocity=Speed*0.4;
    else if (encoderPos>Position*0.95)
    Velocity=Speed*0.3;
 
  }
 //==========================================Motor2============================================
 void speedup2()
{
      //--------Speed Up---------------//
    if(encoderPos2<Position2*0.005)
    Velocity2=Speed2*0.08;//maximum 270
    else if(encoderPos2<Position2*0.01)
    Velocity2=Speed2*0.12;
    else if(encoderPos2<Position2*0.015)
    Velocity2=Speed2*0.14;
    else if(encoderPos2<Position2*0.02)
    Velocity2=Speed2*0.18;
    else if(encoderPos2<Position2*0.025)
    Velocity2=Speed2*0.24;
    else if(encoderPos2<Position2*0.03)
    Velocity2=Speed2*0.26;
    else if(encoderPos2<Position2*0.035)
    Velocity2=Speed2*0.3;
    else if(encoderPos2<Position2*0.04)
    Velocity2=Speed2*0.34;
    else if(encoderPos2<Position2*0.058)
    Velocity2=Speed2*0.38;
    else if(encoderPos2<Position2*0.06)
    Velocity2=Speed2*0.4;
    else if(encoderPos2<Position2*0.065)
    Velocity2=Speed2*0.42;
    else if(encoderPos2<Position2*0.07)
    Velocity2=Speed2*0.44;
    else if(encoderPos2<Position2*0.075)
    Velocity2=Speed2*0.46;
    else if(encoderPos2<Position2*0.08)
    Velocity2=Speed2*0.48;
    else if(encoderPos2<Position2*0.085)
    Velocity2=Speed2*0.5;
    else if(encoderPos2<Position2*0.09)
    Velocity2=Speed2*0.6;
    else if(encoderPos2<Position2*0.105)
    Velocity2=Speed2*0.7;
    else if(encoderPos2<Position2*0.12)
    Velocity2=Speed2*0.8;
    else if(encoderPos2<Position2*0.15)
    Velocity2=Speed2*0.9;
    else
    Velocity2=Speed2;
  }

void speeddown2()
{
      
    //--------Speed down---------------//
    if   (encoderPos2>Position2*0.8 && encoderPos2<=Position2*0.85)
    Velocity2=Speed2*0.9;
    else if (encoderPos2>Position2*0.85  && encoderPos2<=Position2*0.88)
    Velocity2=Speed2*0.8;
    else if (encoderPos2>Position2*0.88  && encoderPos2<=Position2*0.91)
    Velocity2=Speed2*0.7;
    else if (encoderPos2>Position2*0.91  && encoderPos2<=Position2*0.93)
    Velocity2=Speed2*0.6;
    else if (encoderPos2>Position2*0.93  && encoderPos2<=Position2*0.94)
    Velocity2=Speed2*0.5;
    else if (encoderPos2>Position2*0.94 && encoderPos2<=Position2*0.95)
    Velocity2=Speed2*0.4;
    else if (encoderPos2>Position2*0.95)
    Velocity2=Speed2*0.3;
 
  }
  //===========================================================================================================

void pwmOut(int out) { // to H-Bridge board
if (out > 0) {
digitalWrite(M1,HIGH); // drive motor CW
digitalWrite(M2,LOW);
analogWrite(EnA, out);
}
else {
digitalWrite(M1,LOW);
digitalWrite(M2,HIGH); // drive motor CCW
analogWrite(EnA, abs(out));
}}

void pwmOut2(int out) { // to H-Bridge board
if (out > 0) {
digitalWrite(M3, HIGH); // drive motor CW
digitalWrite(M4, LOW);
analogWrite(EnB, out);
}
else {
digitalWrite(M3, LOW);
digitalWrite(M4, HIGH); // drive motor CCW
analogWrite(EnB, abs(out));
}
}

void encoder() {    // pulse and direction, direct port reading to save cycles
if(digitalRead(encodPinB1)==HIGH)
encoderPos++;      // if(digitalRead(encodPinB1)==HIGH) count ++;
else encoderPos--; // if(digitalRead(encodPinB1)==LOW) count --;
}
void encoder2() { // pulse and direction, direct port reading to save cycles
if(digitalRead(encodPinB2)==HIGH)
encoderPos2++;      // if(digitalRead(encodPinB1)==HIGH) count ++;
else encoderPos2--; // if(digitalRead(encodPinB1)==LOW) count --;
}

/**************receiving part************************/

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
      smotor1=dataReceived[0];
      smotor2=dataReceived[1];
      
      Serial.print("Data received ");
     Serial.print(" motor1:");
     Serial.print(dataReceived[0]);
     Serial.print("  ,  ");
     Serial.print("motor2:");
     Serial.println(dataReceived[1]);
     Serial.println("  ");
     }         
}
void senddata()
{    if (newData == true)
    {
     radio.stopListening();                          //stop receiving start to send
     rlst=radio.write(&dataSend, sizeof(dataSend));  // send the data
     radio.startListening();                         //start to receive the data
     
     if (rlst==1)                     //if send the data success so print
     {    dataSend[0]=encoderPos;     //real position of motor 1
          dataSend[1]=encoderPos2;    //real position of motor 2
          dataSend[3]=dbuf;           //real speed of motor 1
          dataSend[4]=dbuf2;          //real speed of motor 2
          
          Serial.println("Reply sent ");
          Serial.print("from motor1:");
          Serial.print(dataSend[0]);
          Serial.print("  ,  ");
          Serial.println(dataSend[3]);
          
          Serial.print("from motor2:");
          Serial.print(dataSend[1]);
          Serial.print("  ,  ");
          Serial.println(dataSend[4]);
          
          Serial.println("  ");
     }
 }
}
/**************receiving part************************/
