#if (ARDUINO >= 100)
 #include <ArduinoHardware.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

//BLDC_Motor_pinL : PIN5
//BLDC_Motor_pinR : PIN6

ros::NodeHandle  nh;

int channel1;
int channel2;
int channel3;
Servo ML, MR;

int Relay1 = 2; //PIN2
int Relay2 = 3; //PIN3

int count = 0;

void servo_cb( const std_msgs::Float64MultiArray&msg)
{
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);
        
      ML.writeMicroseconds(msg.data[0]);//set servo angle, should be from 0-180
      MR.writeMicroseconds(msg.data[1]);//R power 1
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("PWM", servo_cb);

void setup()
{
//      Serial.begin(57600);
      ArduinoHardware().setBaud(57600);
      ML.attach(5, 1050, 1950);
      MR.attach(6, 1050, 1950);
      MR.writeMicroseconds(1500);
      ML.writeMicroseconds(1500);
      
      pinMode(Relay1, OUTPUT);
      pinMode(Relay2, OUTPUT);
      
      
      nh.initNode();
      nh.subscribe(sub);  
}

void loop()
{    
      int channel1 = constrain(channel1,1100,1850);
      int channel2 = constrain(channel2,830,1600);
      
      channel3 = 0;
      
      if (pulseIn(10,HIGH) > 0)
      {
            channel1 = pulseIn(8,HIGH);
            channel2 = pulseIn(9,HIGH);
            channel3 = pulseIn(10,HIGH);
      }
      
      //channel1 = pulseIn(8,HIGH);
      //channel2 = pulseIn(9,HIGH);
      //channel3 = pulseIn(10,HIGH);
      
      //Serial.println(channel1); //1240 1610 2000
      //Serial.println(channel2); //937 1360 1720
      //Serial.println(channel3);------ 0:off, 1000 : manual control, 2000 : automode
      
      int c1, c2;
      
      c1 = map(channel1, 1220, 2010, -100, 100); // -10 ~ 10
      c2 = map(channel2, 920, 1725, -100, 100); // 0~20
      
      if(c1 > -10 && c1 < 10) c1 = 0;
      if(c2 > 0 && c2 < 20) c2 = 0;
      
      Serial.println(channel3);
      int TL, TR;
      if (channel3 == 0) //Controller OFF
      {
            MR.writeMicroseconds(1500);
            ML.writeMicroseconds(1500);
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            Serial.println("shut down");
      }
      else if(channel3 < 1500) //Manual Control Mode
      {
            digitalWrite(Relay1, HIGH);
            digitalWrite(Relay2, HIGH);
            TR = -(c2 * 5 + 5 *c1);
            TL = c2 * 5 - 5 *c1;
            TL+=1500;
            TR+=1500;
            
            
            TL = constrain(TL,1050,1950);
            TR = constrain(TR,1050,1950);
            Serial.print(TL);
            Serial.print("  ");
            Serial.println(TR);
            MR.writeMicroseconds(TR);
            ML.writeMicroseconds(TL);
      }
      else //AutoMode
      {
            Serial.println("AutoMode");
            nh.spinOnce();
      }
}
