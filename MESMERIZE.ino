// int setpoint = 0, pre_error, p, i, d, error, pid;
// float kp,kd,ki;
int counter1,counter2;

int j_pin;
int a;
int c;
int b;
int d;
int e;
int f;
int led=2;
int s1, s2, s3, s4, s5, s6, s7, s8;
int IN1 = A6;
int IN2 = A5;
int IN3 = A4;
int IN4 = A3;
int ENA = 9;
int ENB = 11;
int counter;

void setup() {
  Serial.begin(9600);
  pinMode(13,INPUT),pinMode(2, OUTPUT),pinMode(12, INPUT), pinMode(4, INPUT), pinMode(3, INPUT), pinMode(5, INPUT), pinMode(6, INPUT), pinMode(7, INPUT), pinMode(8, INPUT), pinMode(9, INPUT);
  pinMode(A3, OUTPUT), pinMode(A4, OUTPUT), pinMode(A5, OUTPUT), pinMode(A6, OUTPUT), pinMode(11, OUTPUT), pinMode(9, OUTPUT);
}

void loop() {
 j_pin=digitalRead(13), s1 = digitalRead(12), s2 = digitalRead(3), s3 = digitalRead(4), s4 = digitalRead(5), s5 = digitalRead(6), s6 = digitalRead(7), s7 = digitalRead(8), s8 = digitalRead(9);
 
  if(s4==1&&s5==1){
    forward();
  }
  if(s4==1||s5==1||s3==1){
    forward();
  }
  // if(s1==1||s2==1||s3==1){
  //   left();
  // }
  // if(s6==1||s7==1){
  //   right();
      
  // }
  // if(s5==1&&s6==1&&s7==1){
  //   right();
  // }
  // if(s1==0&&s2==0&&s3==0&&s4==0&&s5==0&&s6==0&&s7==0&&s8==0){
  //   right();
  //   Serial.print("right");
  // }
if(s7==1&&c==0){
  counter1++;
  c=1;
 }
  if(s7==0&&c==1){
  c=0;
 }
 if(s2==1&&b==0){
  counter++;
  b=1;
 }
  if(s2==0&&b==1){
  b=0;
 }
    if(counter==1&&a==1){
    left();
      a=2;
    }
    if(counter1==1&&a==2){
      right();
      Serial.print("right");
      a=3;
    }
     if(counter1==2&&a==3){
      right();
      a=4;
    }
//     if(counter==4&&a==4){
//       left();
//       a=5;
//     }
//     if(counter==5&&a==5){
//       left();
//       a=6;
//     }
//     if(counter==6&&a==6){
//       forward();
//       a=7;
//     }
//     if(counter==7&&a==7){
//       forward();
//       a=8;
//     }


//     if(counter1==1&&d==1){
//     right();
//       Serial.print("right");
//       d=2;
//     }
//     if(counter1==2&&d==2){
//       right();
//       d=3;
//     }
//      if(counter1==3&&d==3){
//       right();
//       d=4;
//     }
//      if(counter1==4&&d==4){
//       forward();
//       d=5;
//     }
//     if(counter1==5&&d==5){
//       right();
//       d=6;
//     }
//     if(counter1==6&&d==6){
//       forward();
//       d=7;
//     }
//     if(counter1==7&&d==7){
//       forward();
//       d=8;
//     }
//     if(counter1==8&&d==8){
//       left();
//       d=9;
//     }
//      if(s7==1&&s2==1&&e==0){
//   counter2++;
//   e=1;
//  }
//   if(s7==0&&s2==0&&e==1){
//   e=0;
//  }
//     if(counter2==1&&f==1){
//     left();
//       Serial.print("right");
//       f=2;
//     }
//     if(counter2==2&&f==2){
//       forward();
//       f=3;
//     }
    //  if(counter==3&&a==3){
    //   right();
    //   a=4;
    // }
    //  if(counter==4&&a==4){
    //   forward();
    //   a=5;
    // }
 if(s2==1&&s3==1&&s4==1&&s5==1&&s6==1&&s7==1){
  digitalWrite(2,HIGH);
 }
 else{
  digitalWrite(2,LOW);
 }
    // if(counter==3){
    //   left();
    // }
 
 

  Serial.print(    s1);
  Serial.print(  s2);
  Serial.print(  s3);
  Serial.print(  s4);  //off when all 6 on
  Serial.print(  s5);
  Serial.print(  s6);
  Serial.print(  s7);
  Serial.print(  s8);
   
  Serial.print("  j_pin");
  Serial.print(j_pin);
   
  Serial.print("  counter");
  Serial.print(counter);
Serial.print("  counter1  ");
  Serial.print(counter1);
  Serial.print("  counter2  ");
  Serial.println(counter2);
  
}
void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(11, 60);
  analogWrite(9, 60);
}
void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
    analogWrite(11, 70);
  analogWrite(9, 70);
}

void right() {
   digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(11, 60);
  analogWrite(9, 70);
}

void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

