#include <iostream>
#include <math.h>
#include <wiringPi.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>

#define  L_ENC_A    0
#define  L_ENC_B    1
#define  R_ENC_A    2
#define  R_ENC_B    3

ros::Publisher g_LWheelPublisher;
ros::Publisher g_RWheelPublisher;

std_msgs::Int16 l_enc_ticks;
std_msgs::Int16 r_enc_ticks;

// Keep track of prev encoder pattern
int prev_l_enc_a = 0;
int prev_l_enc_b = 0;

int prev_r_enc_a = 0;
int prev_r_enc_b = 0;

// Encoder variable
int n_l_enc = 0;
int n_r_enc = 0;


void publishTicks()
{
  int l_enc_a = digitalRead(L_ENC_A);
  int l_enc_b = digitalRead(L_ENC_B);
  
  if (digitalRead(L_ENC_A) != prev_l_enc_a || digitalRead(L_ENC_B) != prev_l_enc_b)
  {
    prev_l_enc_a = l_enc_a;
    prev_l_enc_b = l_enc_b;

    n_l_enc++;
  }
  if (n_l_enc >= 4)
  {
    n_l_enc = 0;

    l_enc_ticks.data = r_enc_ticks.data + 1;
  }

  int r_enc_a = digitalRead(R_ENC_A);
  int r_enc_b = digitalRead(R_ENC_B);
  if (digitalRead(R_ENC_A) != prev_r_enc_a || digitalRead(R_ENC_B) != prev_r_enc_b)
  {
    prev_r_enc_a = r_enc_a;
    prev_r_enc_b = r_enc_b;

    n_r_enc++;
  }
  if (n_r_enc >= 4)
  {
    n_r_enc = 0;

    r_enc_ticks.data = r_enc_ticks.data + 1;
  }
  
//  std::cout << "Left encoder = " << l_enc_ticks << std::endl;
//  std::cout << "Right encoder = " << r_enc_ticks << std::endl;

  g_LWheelPublisher.publish(l_enc_ticks);
  g_RWheelPublisher.publish(r_enc_ticks);
}

void resetEncoders()
{
  l_enc_ticks.data = 0;
  r_enc_ticks.data = 0;
}

int main(int argc, char **argv)
{
	if(wiringPiSetup() < 0){
		fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
		return 1;
	}

	pinMode(L_ENC_A, INPUT);
	pinMode(L_ENC_B, INPUT);
	pinMode(R_ENC_A, INPUT);
	pinMode(R_ENC_B, INPUT);
	
	ros::init(argc, argv, "encoder_node");
	ros::NodeHandle node;
	
	g_LWheelPublisher = node.advertise<std_msgs::Int16>("/lwheel", 100);
	g_RWheelPublisher = node.advertise<std_msgs::Int16>("/rwheel", 100);

  while (ros::ok())
  {
    publishTicks();
    ros::spinOnce();
  }

	return 0;
}

