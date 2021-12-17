/*
 * This is a code for an inverse kinematics robotic arm used in mechatronics system project
 * -----------------------------------------------------------------------------------------
 * The idea of the project:
 * __________________________
 * A GUI is used to interface with customers. The customers choose the blocks required (green/red) Using Python.
 * A gopro camera is used to detect the objects and the colours.
 * Python sends the arduino to run the program.
 * Using the ultrasonic sensor, the robotic arm will calculate the distance of the object and starts moving to it.
 * An LCD displays the distance to object.
 * The robotic arm places the green colour in a location and the red colour to another location.
 * 
 */

 #include <Servo.h>
//#include <LiquidCrystal.h>

#define Y 62.0
#define X 38.5
#define y 30.0 //distance from robotic arm to ultrasonic sensor*
#define Pi 3.14159
#define L1 12.0
#define L2 24.19
#define trigpin 9
#define echopin 8

#define SpeedDelay 1200 //Servo Speed Delay in Micro Seconds
#define MinPWM 500 //Minimum Pulse Width
#define MaxPWM 2500 //Maximum Pulse Width
#define APD 1 //Angle Per Delay
int o = 1500;
int Count = 0;
char python_start;
char python_rog;
float d,x, r, theta, th1, th2, alpha, Xc, Yc; //x: Distance From UltraSonic Sensor 
float dc = 1.25;
long duration;
int al,t1,t2;
int IR1 = 53;
int IR2 = 52;
int pb1 = 45;
int pb2 = 44;
int R_LED1 = 46;
int R_LED2 = 48;
int buzzer = 50;
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;


//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//LiquidCrystal lcd(40, 41, 38, 37, 36, 23);

void ApplyServoAngle(Servo x, int Angle)
{
  int PWM = int(((float(Angle)/180)*(MaxPWM - MinPWM)) + MinPWM);
  o = x.readMicroseconds();
  if (o <= PWM)
  {
    for (o ; o <= PWM ; o=o+APD)
    {
      x.writeMicroseconds(o);
      delayMicroseconds(SpeedDelay);
    }
  }
  else
  {
    for (o ; o >= PWM ; o=o-APD)
    {
      x.writeMicroseconds(o);
      delayMicroseconds(SpeedDelay);
    }
  }
}

void initial_position()
{ 
  
     ApplyServoAngle(shoulder, 110);
     ApplyServoAngle(base, 90);
     ApplyServoAngle(elbow, 60);
     ApplyServoAngle(wrist, 90);
     ApplyServoAngle(gripper, 135);
     
}

void show_camera()
{ 
     ApplyServoAngle(shoulder, 110);
     ApplyServoAngle(base, 90);
     ApplyServoAngle(elbow, 120);
     ApplyServoAngle(wrist, 90);
}

void Place1()
{
   ApplyServoAngle(shoulder, 90);
   ApplyServoAngle(elbow,100 );
   ApplyServoAngle(base,150 );
   ApplyServoAngle(elbow,130 );
   ApplyServoAngle(shoulder, 60);
   ApplyServoAngle(wrist, 90);
   ApplyServoAngle(gripper, 130);
}

void Place2()
{
   ApplyServoAngle(shoulder, 90);
   ApplyServoAngle(elbow,100 );
   ApplyServoAngle(base,35 );
   ApplyServoAngle(elbow,130 );
   ApplyServoAngle(shoulder, 60);
   ApplyServoAngle(wrist, 90);
   ApplyServoAngle(gripper, 130);
}

void Inverse_kinematics()
{
   r = 31.05;
   float a = r*r;
   float b = L1 * L1;
   float c = L2 * L2;
   th2 = acos((a-b-c)/(2*L1*L2));
   th1 = asin((L2 * sin(Pi-th2))/r)*180/Pi;
   th2 = th2 *180/Pi;
   Serial.print("th1: ");
   Serial.println(th1);
   Serial.print("th2: ");
   Serial.println(th2);
}

void Pick()
{
 ApplyServoAngle(base, alpha);
 ApplyServoAngle(elbow, 180-th2);
 ApplyServoAngle(shoulder, th1);
 ApplyServoAngle(wrist, theta);
 ApplyServoAngle(gripper, 80);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //lcd.begin(16, 2);
  base.attach(3);
  shoulder.attach(4);
  elbow.attach(5);
  wrist.attach(7);
  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
  gripper.attach(6);
  pinMode(pb1, INPUT);
  pinMode(pb2, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(R_LED1,OUTPUT);
  pinMode(R_LED2,OUTPUT);
  pinMode(buzzer,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

      while(Serial.available()==0);
      if(Serial.available()>0)
      {
        python_start = Serial.read();
        Serial.flush();
        Serial.println("Start");
      }
      if (python_start == '3')
      {
        digitalWrite(buzzer,LOW);
        digitalWrite(R_LED1,LOW);
        digitalWrite(R_LED2,LOW);

        initial_position();
        delay(2000);
      
        //lcd.clear();
      do{
          digitalWrite(trigpin, LOW);
          delayMicroseconds(2);     
          digitalWrite(trigpin, HIGH);
          delayMicroseconds(10);
          digitalWrite(trigpin, LOW);
          duration = pulseIn(echopin, HIGH);
          x = duration*0.034/2;
          Serial.print("x: ");
          Serial.println(x);
          Serial.println("DONE");
        //lcd.print("distance =");
        //lcd.print(x);
        //lcd.print(" cm");
        if(x<8 || x>X-3)
          {
            Serial.println("NO object");
             //lcd.clear();
             // lcd.print("NO Object");
          }
           }while(x<8 || x>X-3);
        if(x > 8 && x < X-3) // Specify the maximum distance
        {
          //Base Calculations
          if(x<16.65)
          {
            d = 17.9-x-dc;
            alpha = 90 - atan(d/y)*180/Pi;
            theta = 90;
            Serial.print("alpha: ");
            Serial.println(alpha);
          }
          else if (x>16.65)
          {
            d = x-17.9+dc;
            alpha = 90+ atan(d/y)*180/Pi;
            theta = 90;
            Serial.print("alpha: ");
            Serial.println(alpha);
          }
          else
          {
            alpha = 90;
            theta = 90; 
            Serial.print("alpha: ");
            Serial.println(alpha);
          } 
          Inverse_kinematics();
          tone(buzzer, 950, 500);
          delay(1000);
          tone(buzzer, 950, 500);
          delay(1000);
          tone(buzzer, 950, 500);
          digitalWrite(buzzer,LOW);
          Pick();
          delay(2000);
          //Show Camera
          show_camera();
          // where to place
    SR :
          while(Serial.available()==0);
          if(Serial.available()>0)
          {
            python_rog = Serial.read();
            Serial.flush();
          }
          if(python_rog == '1')
          {
            Serial.println("Red");
            python_rog = ' ';
            if(digitalRead(IR1)==HIGH)
            {
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              digitalWrite(buzzer,LOW);
              Place1();
            }
            // if the place is occupied
            else
            {
              do {

                   Serial.println("Red: Space Occupied");
                   digitalWrite(R_LED1,HIGH); 
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
                   delay(1000);
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
                   delay(1000);
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
              }while(digitalRead(IR1)==LOW);
              digitalWrite(buzzer,LOW);
              digitalWrite(R_LED1,LOW); 
              delay(2000);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              digitalWrite(buzzer,LOW);
              Place1();
              
            }
            delay(1500);
          }
          else if(python_rog == '2')
          {
            Serial.println("Green");
            python_rog = ' ';
            if(digitalRead(IR2)==HIGH)
            {
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              digitalWrite(buzzer,LOW);
              Place2();
            }
            // if the place is occupied
            else
            {
              do {

                   Serial.println("Green: Space Occupied");
                   digitalWrite(R_LED2,HIGH); 
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
                   delay(1000);
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
                   delay(1000);
                   tone(buzzer, 600, 1000);
                   delay(1000);
                   tone(buzzer, 800, 1000);
              }while(digitalRead(IR2)==LOW);
              digitalWrite(buzzer,LOW);
              digitalWrite(R_LED2,LOW); 
              delay(1500);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              delay(1000);
              tone(buzzer, 950, 500);
              digitalWrite(buzzer,LOW);
              Place2();
            }
            delay(1500);
          }
          else
          {
            goto SR; 
          }
        }
        initial_position();
        delay(1000);
      while((digitalRead(IR1)==LOW) && (digitalRead(IR2)==LOW) )
        {
          digitalWrite(R_LED1,HIGH); 
          digitalWrite(R_LED2,HIGH);
          digitalWrite(buzzer,HIGH);
          while(digitalRead(pb1)==LOW);
          digitalWrite(buzzer,LOW);
          while(digitalRead(pb2)==LOW)
          {
            digitalWrite(R_LED1,LOW);
            digitalWrite(R_LED2,LOW);
            delay(500);
            digitalWrite(R_LED1,HIGH);
            digitalWrite(R_LED2,HIGH);
            delay(500);
            digitalWrite(R_LED1,LOW);
            digitalWrite(R_LED2,LOW);
            delay(500);
            digitalWrite(R_LED1,HIGH);
            digitalWrite(R_LED2,HIGH);
            delay(500);
            }
        digitalWrite(R_LED1,LOW);
        digitalWrite(R_LED2,LOW);
        
          }
      }
      python_start = ' ';
}
