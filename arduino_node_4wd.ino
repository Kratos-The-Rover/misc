#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float32MultiArray.h>

#define VMAX 0.22
#define OMEGA_MAX 2.84

ros::NodeHandle nh;

geometry_msgs::Twist twist_obj;
geometry_msgs::Twist pwms_obj;
float v;
float omega;
float ang_vel;
float vel_wheels[4];
int pwms[4] = {0, 0, 0, 0};
bool dirs[4];
int pwm_pins[4] = {3,5,6,9};
int dir_pins[4] = {10,11,12,13};
int dirs2[4] = {2,4,7,8};
//std_msgs::Float32MultiArray arr;
//arr.data.clear();

float mymap(float c ,float a ,float b ,float d ,float e){
  return d + (c-a)*(e-d)/(b-a);
}
void sub_cb(const geometry_msgs::Twist &twist_obj)
{
  ang_vel = twist_obj.angular.z;
  omega = mymap(ang_vel , -2.84,2.84,-0.5,0.5); 
  //if (ang_vel <= 0.05 && ang_vel >= -0.05) omega = 0;
  //else omega = map(ang_vel, -2.84, 2.84, -0.5, 0.5);
  v = twist_obj.linear.x;
  
  Serial.print("This is v:");
  Serial.println(v);
  Serial.print("This is omega:");
  Serial.println(omega);
  // to be editted
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &sub_cb);
ros::Publisher chatter("chatter", &twist_obj);
ros::Publisher pwms_pub("/pwms", &pwms_obj);


void setup()
{
  Serial.begin(57600);
  for(int i=0; i<4; i++)
  {
      digitalWrite(dirs2[i],LOW);
    
  }
 
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(chatter);
  nh.advertise(pwms_pub);
  
  for (int i = 0; i < 4; i++)
  {
    pinMode(pwm_pins[i], OUTPUT);
   
  }
   for (int i = 0; i < 4; i++)
  {
    pinMode(dir_pins[i], OUTPUT);
  }
  for (int i = 0; i < 4; i++) 
  {
    pinMode(dirs2[i], OUTPUT);
  }
  
  delay(100);   //my_change
}

void loop()
{
   // For right wheels
    vel_wheels[0] = v + omega;
    vel_wheels[1] = v + omega;
    
    // For left wheels
    vel_wheels[2] = v - omega;
    vel_wheels[3] = v - omega;
    
    twist_obj.linear.x=vel_wheels[0];
    twist_obj.linear.y=vel_wheels[1];
    twist_obj.linear.z=vel_wheels[2];
    twist_obj.angular.x=vel_wheels[3];
    
      for (int i=0 ; i<4 ; i++){
        dirs[i] = HIGH;
        if(vel_wheels[i]>0)pwms[i] = mymap(vel_wheels[i] , -0.5 , (VMAX + 0.5), 0.0, 255.0);
        else pwms[i]=0; 
        //map(vel_wheels[i], -OMEGA_MAX, (VMAX + OMEGA_MAX), 0, 255);
      }
    pwms_obj.linear.x=pwms[0];
    pwms_obj.linear.y=pwms[1];
    pwms_obj.linear.z=pwms[2];
    pwms_obj.angular.x=pwms[3];
      
   


    // Sending commands to motor
    for (int i = 0; i < 4; i++)
    {
      analogWrite(pwm_pins[i], pwms[i]);

      
      if (dirs[i] == 0) digitalWrite(dir_pins[i], LOW);
      else digitalWrite(dir_pins[i], HIGH);

    }

      //#ifdef SERIAL_DEBUG
      Serial.print("pwm0 is: ");
      Serial.println(pwms[0]);
      Serial.print("pwm1 is: ");
      Serial.println(pwms[1]);
      Serial.print("pwm2 is: ");
      Serial.println(pwms[2]);
      Serial.print("pwm3 is: ");
      Serial.println(pwms[3]);
      //#endif

  chatter.publish(&twist_obj);
  pwms_pub.publish(&pwms_obj);
  nh.spinOnce();
  
  delay(200);
}
