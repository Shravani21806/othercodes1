#include "CytronMotorDriver.h"
float p1 = 0;
float p2 = 0;  
int last_state1= 0; 
int last_state2= 0;  
int encoderValue = 0; 
int count1;
int count2;
float error1 = 0;
float error2 = 0;
float setpoint1 = 0;
float setpoint2 = 0;
float integral1 = 0;
float integral2 = 0;
float derivative1= 0;
float derivative2= 0;
float preError1 = 0;
float preError2 = 0;
int p,q,r,s;

float Kp = 0;
float Ki = 0;
float Kd = 0;
float PID1= 0;
float PID2= 0;
CytronMD motor1(PWM_DIR, 9, 8); 
CytronMD motor2(PWM_DIR, 6, 7); 


void setup() {
 pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
   pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), motor_count2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), motor_count2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(12), motor_count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(13), motor_count1, CHANGE);
  Serial.begin(9600);
}

void loop() {
  Kp=1.9;
  Kd=3.2;
  setpoint1 = 0;
  error1 = setpoint1 - count1;
  integral1= error1 + integral1;
  derivative1 = error1 - preError1;
  preError1 = error1;
  PID1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
  PID1 = constrain(PID1, -100, 100);
   motor1.setSpeed(PID1);
Kp=1.9;
Kd=3.2;
 setpoint2=0;
  error2 = setpoint2 - count2;
  integral2 = error2 + integral2;
  derivative2= error2- preError2;
  preError2= error2;
PID2 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
 PID2 = constrain (PID2, -100, 100);
 motor2.setSpeed(PID2);

 
//   setpoint2=600;
//  if (p2==600)
//  {
//   setpoint2=0;
//  }
//  if(p2==0)
//  {
// setpoint1=-600;
//  }
//  if(p1=-600)
//  {
//   setpoint2=600;
//  }

  // Serial.print("error1:");
  // Serial.print(error1);
  //  Serial.print("    error2:");
  // Serial.print(error2);
  //   Serial.print("   count1: ");
  // Serial.print(count1);
  //  Serial.print("  count2:");
  // Serial.println(count2);

    Serial.print("   p: ");
  Serial.print(p);
   Serial.print("  q:");
  Serial.print(q);
      Serial.print("   r: ");
  Serial.print(r);
   Serial.print(" s :");
  Serial.println(s);


  
  
}

void motor_count1(){
  int p = digitalRead(2);
  int q = digitalRead(3);
 if (p != last_state1){
  if(p!=q)
  {
    count1++;
  }
  else
  {
    count1--;
  }
 }
 last_state1=p;
}
void motor_count2(){
  int r = digitalRead(12);
  int s = digitalRead(13);
 if (r != last_state2){
  if(r!=s)
  {
    count2++;
  }
  else
  {
    count2--;
  }
 }
 last_state2=r;
}


