/********this is B board *********/
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
const byte slaveAddress[][6]= {"00011","00022","00033","00044"};
const byte masterAddress[5]= {'T','X','a','a','a'};
double dataReceived[2];
double dataSend[4]={109,110,110,110};
//receiver part
double smotor1;
double smotor2;
bool newData = false;
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis =100 ; // send once per second  delay time 
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
double kp = 0.1, ki= 0.001, kd =0.00000; // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
long temp;
long cycle=0;
double pwm_last=0;
double sig;
double encoderPos = 0;
double encoderPos2 = 0;
double t,dbuf,timem,buf2,buf1;


PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPID2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT ); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
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

myPID2.SetMode(AUTOMATIC);
myPID2.SetSampleTime(1); //defaut 1=200ms Determines how often the PID algorithm evaluates
myPID2.SetOutputLimits(-255, 255);

Serial.begin(9600);
Serial.println("SimpleRx Starting");
radio.begin();
radio.setDataRate(RF24_250KBPS);
radio.openWritingPipe(masterAddress);
radio.openReadingPipe(1,slaveAddress[2]); //receiver address
radio.setRetries(3,5);
radio.startListening();
}

void loop() { 
  currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
    getData();
    prevMillis = millis();
    } 
      senddata();

   setpoint= 100;//    X from 0.1mm to motor encoder
    input= encoderPos ;
    myPID.Compute(); 
    pwmOut(output);
       
    setpoint2= 1000;//    X from 0.1mm to motor encoder
    input2 = encoderPos2 ;
    myPID2.Compute(); 
    pwmOut2(-output2);

   
      Serial.print(" motor1:");
      Serial.print(encoderPos);
      Serial.print("  ,  ");
      Serial.print("motor2:");
      Serial.println(encoderPos2);
    cycle++;

    
}

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
}}

void encoder() { // pulse and direction, direct port reading to save cycles
//if (PINB & 0b00000001) encoderPos++; // if(digitalRead(encodPinB1)==HIGH) count ++;
if(digitalRead(encodPinB1)==HIGH)
encoderPos++; // if(digitalRead(encodPinB1)==HIGH) count ++;
else encoderPos--; // if(digitalRead(encodPinB1)==LOW) count --;
}
void encoder2() { // pulse and direction, direct port reading to save cycles
//if (PINB & 0b00000001) encoderPos++; // if(digitalRead(encodPinB1)==HIGH) count ++;
if(digitalRead(encodPinB2)==HIGH)
encoderPos2++; // if(digitalRead(encodPinB1)==HIGH) count ++;
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
     radio.stopListening(); 
     rlst=radio.write(&dataSend, sizeof(dataSend));
     radio.startListening(); 
     
     if (rlst==1)
     {    dataSend[0]=encoderPos;
          dataSend[1]=encoderPos2;
          dataSend[2]=output;
          dataSend[3]=output2;
          
          Serial.println("Reply sent ");
          Serial.print("from motor1:");
          Serial.print(dataSend[0]);
          Serial.print("  ,  ");
          Serial.println(dataSend[2]);       
          Serial.print("from motor2:");
          Serial.print(dataSend[1]);
          Serial.print("  ,  ");
          Serial.println(dataSend[3]);
          Serial.println("  ");
     }
 }
}
/**************receiving part************************/
