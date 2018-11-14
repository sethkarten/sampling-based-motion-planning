#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h> 
#include <ackermann_msgs/AckermannDrive.h>

double angle = 0.0; 
double distance = 10.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidObstacle");
    ros::NodeHandle n;
    ros::Publisher ackmsg_pub = n.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 1000);
    
    ackermann_msgs::AckermannDrive ackmsg;
    ackmsg.speed = 1.0;
    
    ros::Rate loop_rate(10);
    
    while(n.ok())
    {
        
        if (distance <= 1.5)
        {
            ackermann_msgs::AckermannDrive updated_ackmsg;
            updated_ackmsg.speed = 0.5;
            if (angle <= 0.005) 
            {
                updated_ackmsg.steering_angle = 10.0;
            }
            else 
            {
                if (angle <= 0.5)
                {
                    updated_ackmsg.steering_angle = 10.0;
                }
                else
                {
                    updated_ackmsg.steering_angle = 0.0;
                }
            }
            
            ackmsg_pub.publish(updated_ackmsg);
        } 
        else
        {
            ackmsg_pub.publish(ackmsg);
        }
        distance = 10.0;
        ros::spinOnce(); 
        loop_rate.sleep();
    } 
    return 0;
}
