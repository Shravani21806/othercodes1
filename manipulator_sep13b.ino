
#include "CytronMotorDriver.h"
#include <Encoder.h>
#include <PinChangeInterrupt.h>
#include <Servo.h>
Servo myservo; 
int pos = 0; 

int EN1A=2;
int EN1B=3;

int EN2A=12;
int EN2B=13;

int pos1=0;
volatile int laststate1;
int t=0;
int q=0;

int pos2=0;
volatile int laststate2;
int r=0;
int s=0;

float error1=0;
float setpoint1=0;
float i1=0;
float d1=0;
float p1=0;
float Kp1=0;
float Ki1=0;
float Kd1=0;
float pid1=0;
float pre_error1=0;
float error2=0;
float setpoint2=0;
float i2=0;
float d2=0;
float p2=0;
float Kp2=0;
float Ki2=0;
float Kd2=0;
float pid2=0;
float pre_error2=0;
int a=0;
int b=0;
float count1=0;
float count2=0;
float m,n;
//  int motor_count2;
//  int motor_count1;



CytronMD motor1(PWM_DIR, 6, 7); 
CytronMD motor2(PWM_DIR, 9, 10); 

void setup() {
  myservo.attach(5);  
  pinMode(10,INPUT);
  pinMode(11,INPUT);
 pinMode(EN1A,INPUT);
 pinMode(EN1B,INPUT);
 pinMode(EN2A,INPUT);
 pinMode(EN2B,INPUT);
digitalWrite(EN2A, HIGH);
  digitalWrite(EN2B, HIGH);
  digitalWrite(EN1A, HIGH);
  digitalWrite(EN1B, HIGH);
  attachPCINT(digitalPinToPCINT(EN2A),motor_count2 , CHANGE);
  attachPCINT(digitalPinToPCINT(EN2B), motor_count2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(2), motor_count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), motor_count1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(12), motor_count2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(13), motor_count2, CHANGE);
 Serial.begin(9600);
}
void loop(){
  b=count2/10;
  a=count1/10;
m=digitalRead(10);
n=digitalRead(11);
myservo.write(5);

if (n==0 && a==0);
{ Serial.print("hii");
  setpoint2=-620;
}
if(b<=-600 && b>=-620  &&  m==1)
{Serial.print("hey");
  for (pos = 90; pos >= 0; pos -= 1)
  {
   myservo.write(pos); 
  }
}
if(pos=90  && m==1)
{
  setpoint2=0;
}
if(pos=90  &&  m==1 && b<=10 && b>=0 && a==0)
{
  setpoint1=-300;
}
if (a>=-300 && a<=-270)
{
  setpoint2=-40;
}
if((a>=-300 && a<=-270) && (b>=45 && b<=30))

   for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos); 
}


  // if(a==0)
  // {Serial.print("hey");
  //   setpoint2=620;
  // }
  //  if(b>=600 && b<=620)
  // {Serial.print("hii");
  //   setpoint1=-300;
  // }

  //  if(a>=-300&&a<=-270)
  // {
  //   setpoint2=-40;

  // }
 

  error1=setpoint1-a;
  p1=error1;
  i1=error1+i1;
  d1=error1-pre_error1;
  pre_error1=error1;
  Kp1=1;
  Kd1=0.5;
  Ki1=0;
  pid1=Kp1*p1+Ki1*i1+Kd1*d1;
 
  error2=setpoint2-b;
  i2=error2+i2;
  p2=error2;
  d2=error2-pre_error2;
  pre_error2=error2;
  Kp2=1;
  Kd2=0.2;
  Ki2=0;
  pid2=Kp2*p2+Ki2*i2+Kd2*d2;
  // pid2=0;
   pid1=0;
  

  pid1=constrain(pid1,-100,100);
  pid2=constrain(pid2,-100,100);

  motor1.setSpeed(pid1);
  motor2.setSpeed(20);
  
Serial.print("n:");
Serial.print(n);
   Serial.print("  pid1: ");
  Serial.print(pid1);
  Serial.print("   pid2: ");
  Serial.print(pid2);
 
  Serial.print("  error1: ");
  Serial.print(error1);
  Serial.print("   error2: ");
  Serial.print(error2);
  Serial.print("  a: ");
  Serial.print(a);
  Serial.print("   b: ");
  Serial.println(b);

  // if(a=0)
  // {Serial.print("hey");
  //   setpoint2=620;
  // }


 
}
void motor_count1(){
int t = digitalRead(2);
  int q = digitalRead(3);
 if (t != laststate1){
  if(t!=q)
  {
    count1++;
  }
  else
  {
    count1--;
  }
 }
 laststate1=t;
}
void motor_count2(){
  int r = digitalRead(12);
  int s = digitalRead(13);
 if (r != laststate2){
  if(r!=s)
  {
    count2++;
  }
  else
  {
    count2--;
  }
 }
 laststate2=r;
}



