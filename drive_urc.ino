#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

/*VMAX --> Max Linear Velocity
  OMEGA_MAX --> Max Angular Velocity*/
  
#define VMAX 0.22
#define OMEGA_MAX 2.84

ros::NodeHandle nh; //Node decleration

geometry_msgs::Twist twist_obj; 
geometry_msgs::Twist pwms_obj;


float v; //recieved Lin Vel from cmd_vel
float omega; //recieved Ang vel from cmd_vel
float ang_vel; //omega mapped down to ang_vel

float vel_wheels[6]; //Velocity of Wheels (V)
int pwms[6] = {0, 0, 0, 0, 0, 0};

bool dirs[6];
int dir_pins[6] = {6, 7, 4, 5, 3, 2};
int pwm_pins[6] = {10, 11, 12, 13, 8, 9};

float mymap(float c , float a , float b , float d , float e) {
  return d + (c - a) * (e - d) / (b - a);
}

void sub_cb(const geometry_msgs::Twist &twist_obj)
{
  ang_vel = twist_obj.angular.z;
  omega = mymap(ang_vel , -2.84, 2.84, -0.5, 0.5);
  //if (ang_vel <= 0.05 && ang_vel >= -0.05) omega = 0;
  //else omega = map(ang_vel, -2.84, 2.84, -0.5, 0.5);
  v = twist_obj.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &sub_cb);
ros::Publisher chatter("chatter", &twist_obj);
ros::Publisher pwms_pub("/pwms", &pwms_obj);


void setup()
{
 
//  for (int i = 0; i < 6; i++)
//  {
    //   digitalWrite(dirs2[i],LOW);

//  }

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(chatter);
  nh.advertise(pwms_pub);

  for (int i = 0; i < 6; i++)
  {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);

  }
//  for (int i = 0; i < 6; i++)
//  {
//    pinMode(dir_pins[i], OUTPUT);
//  }
//  for (int i = 0; i < 6; i++)
//  {
    //pinMode(dirs2[i], OUTPUT);
//  }

  delay(100);   //my_change
}

void loop()
{
  // For right wheels
  vel_wheels[0] = v + omega; //back 
  vel_wheels[1] = v + omega; //middle 
  vel_wheels[2] = v + omega; //front

  // For left wheels
  vel_wheels[3] = v - omega; //back
  vel_wheels[4] = v - omega; //middle
  vel_wheels[5] = v - omega; //front

  twist_obj.linear.x = vel_wheels[0];
  twist_obj.linear.y = vel_wheels[1];
  twist_obj.linear.z = vel_wheels[2];
  twist_obj.angular.x = vel_wheels[3];
  twist_obj.angular.y = vel_wheels[4];
  twist_obj.angular.z = vel_wheels[5];

  for (int i = 0 ; i < 6 ; i++) {
    dirs[i] = HIGH;
    if (vel_wheels[i] > 0)pwms[i] = mymap(vel_wheels[i] ,+ -(0.5 + VMAX) , (VMAX + 0.5), -255.0, 255.0);
    else pwms[i] = 0;
    //map(vel_wheels[i], -OMEGA_MAX, (VMAX + OMEGA_MAX), 0, 255);
  }
  pwms_obj.linear.x = pwms[0];
  pwms_obj.linear.y = pwms[1];
  pwms_obj.linear.z = pwms[2];
  pwms_obj.angular.x = pwms[3];
  pwms_obj.angular.y = pwms[4];
  pwms_obj.angular.z = pwms[5];
 
  // Sending commands to motor
  for (int i = 0; i < 6; i++)
  {
    analogWrite(pwm_pins[i], abs(pwms[i]));


    if (dirs[i] == 0) digitalWrite(dir_pins[i], LOW);
    else digitalWrite(dir_pins[i], HIGH);

  }

  chatter.publish(&twist_obj);
  pwms_pub.publish(&pwms_obj);
  nh.spinOnce();

  delay(200);
}
