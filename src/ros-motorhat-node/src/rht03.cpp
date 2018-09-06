/*
 * rht03.c:
 *	Driver for the MaxDetect series sensors
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>

#include <wiringPi.h>
#include <maxdetect.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#define	RHT03_PIN	6

ros::Publisher temp_pub;
ros::Publisher humid_pub;

std_msgs::Float32 temp;
std_msgs::Float32 rh;

int result ;
int minT, maxT, minRH, maxRH ;
int temp_t, rh_t;


int numGood, numBad ;

/*
 ***********************************************************************
 * Read from sensor and publish
 ***********************************************************************
 */

void publishReading()
{
  //for (;;)
  //{
    delay (100) ;

    result = readRHT03 (RHT03_PIN, &temp_t, &rh_t) ;

    if (!result)
    {
      printf (".") ;
      fflush (stdout) ;
      ++numBad ;
      //break ;
    }

    ++numGood ;

    if (temp_t < minT) minT = temp_t ;
    if (temp_t > maxT) maxT = temp_t ;
    if (rh_t  < minRH) minRH = rh_t ;
    if (rh_t  > maxRH) maxRH = rh_t ;
    
    temp.data = temp_t / 10.0;
    rh.data = rh_t / 10.0;

	temp_pub.publish(temp);
	humid_pub.publish(rh);

	/*
    printf ("\r%6d, %6d: ", numGood, numBad) ;
    printf ("Temp: %5.1f, RH: %5.1f%%", temp / 10.0, rh / 10.0) ;
    printf ("  Max/Min Temp: %5.1f:%5.1f", maxT / 10.0, minT / 10.0) ;
    printf ("  Max/Min RH: %5.1f:%5.1f", maxRH / 10.0, minRH / 10.0) ;

    printf ("\n") ;
    */
    
  //}	
}

/*
 ***********************************************************************
 * The main program
 ***********************************************************************
 */

int main (int argc, char **argv)
{ 
  ros::init(argc, argv, "temp_humi_node");
  ros::NodeHandle node;
  
  temp_pub = node.advertise<std_msgs::Float32>("/temp", 100);
  humid_pub = node.advertise<std_msgs::Float32>("/humidity", 100);

  if(wiringPiSetup() < 0){
	fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
	return 1;
  }
  piHiPri       (55) ;

  minT =  1000 ;
  maxT = -1000 ;

  minRH =  1000 ;
  maxRH = -1000 ;

  numGood = numBad = 0 ;

  
  
  while (ros::ok())
  {
    publishReading();
    ros::spinOnce();
  }

  return 0 ;
}
