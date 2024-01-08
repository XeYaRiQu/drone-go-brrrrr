/*
 * Copyright (c) 2012, Jan Heuer
 * All rights reserved.
 *
 * Released under a BSD license
 */

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#define DEBUG

#include <ardrone_autonomy/Navdata.h>


#define MSS_PER_GS 9.80

#define M_TAU (M_PI * 2)


geometry_msgs::Quaternion eulerToQuaternion(double x, double y, double z)
{
    geometry_msgs::Quaternion ret;
    double c1 = cos(x/2);
    double s1 = sin(x/2);
    double c2 = cos(y/2);
    double s2 = sin(y/2);
    double c3 = cos(z/2);
    double s3 = sin(z/2);

    ret.w = (c1 * c2 * c3) - (s1 * s2 * s3);
    ret.x = (s1 * s2 * c3) + (c1 * c2 * s3);
    ret.y = (s1 * c2 * c3) + (c1 * s2 * s3);
    ret.z = (c1 * s2 * c3) - (s1 * c2 * s3);
    return ret;
}

inline double degreeToRadian(double degree)
{
    return degree * (M_TAU / 360);
}

class ARDrone_Odom {
private:
    double linx,liny,linz;
    double velx,vely,velz;
	double velx_old,vely_old,velz_old;
    double accx,accy,accz;
    double magx,magy,magz;
    double rotx,roty,rotz;
	double max_velx,max_vely;

    double alt;
    double time;
    ros::Time rtime;
    double dt;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher vo_pub;

public:
    ARDrone_Odom();
    void runloop(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void PubOdom();
};

void ARDrone_Odom::PubOdom()
{
    if (vo_pub.getNumSubscribers() == 0)
        return;

    nav_msgs::Odometry om;
    om.header.stamp = rtime;
	om.header.frame_id = "/nav";

    om.pose.pose.position.x = linx;
    om.pose.pose.position.y = liny;
    om.pose.pose.position.z = linz;
    om.pose.pose.orientation = eulerToQuaternion(degreeToRadian(magx),
                                                 degreeToRadian(magy),
                                                 degreeToRadian(magz));
    double c = degreeToRadian(6.0); // mag covarience..

    double x,y,z;
    x = y = 99999;
    z = 10;

    for (int i = 0; i < 36; i++)
        om.pose.covariance[i] = 0;

    om.pose.covariance[0] = x;
    om.pose.covariance[7] = y;
    om.pose.covariance[14] = z;
    om.pose.covariance[21] = c;
    om.pose.covariance[28] = c;
    om.pose.covariance[35] = c;
    vo_pub.publish(om);
}


void ARDrone_Odom::runloop(const ardrone_autonomy::Navdata::ConstPtr &msg)
{
    if (msg->tm < time) // drop the message, it's out of date.
        return;
    if (time == 0) {
        time = msg->tm;
		velx_old = msg->vx / 1000;
		vely_old = msg->vy / 1000;
        return;
    }
    rtime = ros::Time::now();
    dt = (msg->tm - time) / 1000000; // to seconds...
    double ts = dt * dt;
    time = msg->tm;

    alt = (double)msg->altd / 1000; // mm to m

    velx = msg->vx / 1000;
    vely = msg->vy / 1000;
    velz = msg->vz / 1000;

    double gravity = MSS_PER_GS;

    accx = msg->ax * gravity;
    accy = msg->ay * gravity;
    accz = msg->az * gravity;

    magx = 0; //msg->magX;
    magy = 0; //msg->magY;
    magz = 0; //msg->magZ;

    if (msg->state >= 3 && msg->state != 5) {
       	//linx += ((velx * dt) + (0.5 * ts * accx));
        //liny += ((vely * dt) + (0.5 * ts * accy));
		linx += (velx + velx_old) / 2 * dt;
        liny += (vely + vely_old) / 2 * dt;
		//linx += velx * dt;
        //liny += vely * dt;
        linz = alt;
    } else if (msg->state != 0) { // not flying.
        accx = 0;
        accy = 0;
        accz = gravity;
    }
	
	velx_old = velx;
	vely_old = vely;
	
    PubOdom();
}

ARDrone_Odom::ARDrone_Odom()
{
    linx = 0;
    liny = 0;
    linz = 0;

    velx = 0;
    vely = 0;
    velz = 0;
	
	velx_old = 0;
    vely_old = 0;
    velz_old = 0;

	max_velx = 0;
    max_vely = 0;

    accx = 0;
    accy = 0;
    accz = 0;

    time = 0;

    sub = n.subscribe("ardrone/navdata", 1, &ARDrone_Odom::runloop, this);
    fprintf(stderr, "subscribed to ardrone/navdata\n");
    vo_pub = n.advertise<nav_msgs::Odometry>("vo", 10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_odom");
    ARDrone_Odom odom = ARDrone_Odom();
    ros::spin();

    return 0;
}
