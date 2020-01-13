#include<ros.h>
#include<std_msgs/String.h>

int pwm_value_a =0;
int pwm_value_b =0;
int pwm_value_c =0;

int dir_pins[3];
int pwm_pins[3];
int pwms[3];

int command;


void sub_cb(const std_msgs::String &str_obj)
{
  command=str_obj.data;
  
}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub_obj("/commands",&sub_cb);

void setup() {
  
for(int i=0;i<6;i++) //initializing the pwm pins and the direction pins
{
   pinMode(dir_pins[i],OUTPUT);
   pinMode(pwm_pins[i],OUTPUT);
   //pinMode(dir_pins[i],HIGH);
}
Serial.begin(57600);

}

void loop() {
  // put your main code here, to run repeatedly:

  if(command==1)
  {
    pwm_value_a=pwm_value_a + 10;
    if(pwm_value_a>=0)
    {
      digitalWrite(dir_pins[0],HIGH);
      analogWrite(pwm_pins[0],pwm_value_a); //incrementing motor1 speed
    }
    else
    {
      digitalWrite(dir_pins[0],LOW);
      analogWrite(pwm_pins[0],-pwm_value_a); //incrementing motor1 speed
    }
    
  } 
  if(command==2)
  {
    pwm_value_a=pwm_value_a - 10;
    if(pwm_value_a>=0)
    {
      digitalWrite(dir_pins[0],HIGH);
      analogWrite(pwm_pins[0],pwm_value_a); //decrementing motor1 speed
    }
    else
    {
      digitalWrite(dir_pins[0],LOW);
      analogWrite(pwm_pins[0],-pwm_value_a); //decrementing motor1 speed
    }
  } 
  if(command==3)
  {
    pwm_value_b=pwm_value_b + 10;
    if(pwm_value_b>=0)
    {
      digitalWrite(dir_pins[1],HIGH);
      analogWrite(pwm_pins[1],pwm_value_b); //incrementing motor2 speed
    }
    else
    {
      digitalWrite(dir_pins[1],LOW);
      analogWrite(pwm_pins[1],-pwm_value_b); //incrementing motor2 speed
    }
  }  
  if(command==4)
  {
    pwm_value_b=pwm_value_b - 10;
    if(pwm_value_b>=0)
    {
      digitalWrite(dir_pins[1],HIGH);
      analogWrite(pwm_pins[1],pwm_value_b); //decrementing motor2 speed
    }
    else
    {
      digitalWrite(dir_pins[1],LOW);
      analogWrite(pwm_pins[1],-pwm_value_b); //decrementing motor2 speed
    }
  } 
  if(command==5)
  {
    pwm_value_c=pwm_value_c + 10;
    if(pwm_value_c>=0)
    {
      digitalWrite(dir_pins[2],HIGH);
      analogWrite(pwm_pins[2],pwm_value_c); //incrementing motor3 speed
    }
    else
    {
      digitalWrite(dir_pins[2],LOW);
      analogWrite(pwm_pins[2],-pwm_value_c); //incrementing motor3 speed
    }
  } 
  if(command==6)
  {
    pwm_value_c =pwm_value_c - 10;
    if(pwm_value_c>=0)
    {
      digitalWrite(dir_pins[2],HIGH);
      analogWrite(pwm_pins[2],pwm_value_c); //decrementing motor3 speed
    }
    else
    {
      digitalWrite(dir_pins[2],LOW);
      analogWrite(pwm_pins[2],-pwm_value_c); //decrementing motor3 speed
    }
  } 
  nh.spinOnce();
  delay(50);

}