/********this is B board *********/
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PID_v1.h>

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
double kp = 0.2, ki= 0.01, kd =0.000100; // modify for optimal performance
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

}

void loop() { 

   setpoint= 30000;//    X from 0.1mm to motor encoder
    input= encoderPos ;
    myPID.Compute(); 
    pwmOut(output);
       
    setpoint2= 20000;//    X from 0.1mm to motor encoder
    input2 = encoderPos2 ;
    myPID2.Compute(); 
    pwmOut2(-output2);
    cycle++;
    Serial.print(output);
    Serial.print("     ");
    Serial.println(encoderPos);
    
    Serial.print(output2);
    Serial.print("     ");
    Serial.println(encoderPos2);
    
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

void pwmOut2(int out2) { // to H-Bridge board
if (out2 > 0) {
digitalWrite(M3, HIGH); // drive motor CW
digitalWrite(M4, LOW);
analogWrite(EnB, out2);
}
else {
digitalWrite(M3, LOW);
digitalWrite(M4, HIGH); // drive motor CCW
analogWrite(EnB, abs(out2));
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
