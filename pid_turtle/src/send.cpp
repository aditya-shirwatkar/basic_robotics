#include "ros/ros.h"
#include "turtlesim/Pose.h"

#include "geometry_msgs/Twist.h"

#include <math.h>

using namespace std;

class PID{
    private:
        ros::NodeHandle n;
        ros::Publisher pose_pub;
        ros::Subscriber pose_sub;

        turtlesim::Pose gpose;
        turtlesim::Pose cpose;
        geometry_msgs::Twist cd_vel;
        
        float k1=7.5,k2=0.00001,k3=0.01; 

    public:
        PID();

        void PoseCallback(const turtlesim::Pose& pose){
            cpose = pose;
        }

        void getGoalPose(){
            cout<<"\n Enter a goal pose ";
            cin>>gpose.x>>gpose.y>>gpose.theta;  
        }

        float deg2rad(float ang){
            return (M_PI * ang)/180.0;
        }

        float rad2deg(float ang){
            return (180 * ang)/M_PI;
        }

        float getDistance(){
            return sqrt(pow((cpose.x - gpose.x),2) + pow((cpose.y - gpose.y),2));
        }

        float getAngle1(){
            return ((atan2(gpose.y - cpose.y, gpose.x - cpose.x)) - cpose.theta);
        }

        float getAngle2(){
            return gpose.theta - cpose.theta;
        }

        void PID1(){ //takes to desired x,y goal pose

            float integral_theta = 0;
            float diff_theta = 0;
            float integral_dist = 0;
            float diff_dist = 0;
            
            float e_ang = getAngle1();
            float e_dist = getDistance(); 

            while //(abs(e_ang) >= deg2rad(1)) /*current pose and delta_rot_1 + 2*/ 
                   (abs(e_dist) >= 0.05) //dist
            {
                
                cout<<"\n Current Pose x is "<<cpose.x <<" y is"<<cpose.y<<" \n";
                cd_vel.angular.z = (k1*e_ang) + (k2*integral_theta) + (k3*diff_theta);
                cd_vel.angular.y = 0;
                cd_vel.angular.x = 0;

                cd_vel.linear.x = (k1*e_dist / 20.0) + (k2*integral_dist) + (k3*diff_dist);
                cd_vel.linear.y = 0;
                cd_vel.linear.z = 0;

                e_ang = getAngle1();
                e_dist = getDistance();

                integral_theta += e_ang;
                integral_dist += e_dist;

                diff_dist = e_dist - diff_dist;
                diff_theta = e_ang - diff_theta;
                pose_pub.publish(cd_vel);
                

                ros::spinOnce();
                /* code */
            }
            
        }

        void PID2(){
            float integral_theta = 0;
            float diff_theta = 0;
            
            float e_ang = getAngle2();

            while (abs(e_ang) >= deg2rad(1))
            {
                cout<<"\n Current Pose theta is "<<rad2deg(cpose.theta);
                cd_vel.angular.z = (k1*e_ang / 4.0) + (k2*integral_theta) + (k3*diff_theta);
                cd_vel.angular.y = 0;
                cd_vel.angular.x = 0;

                cd_vel.linear.x = 0;
                cd_vel.linear.y = 0;
                cd_vel.linear.z = 0;

                e_ang = getAngle2();
                integral_theta += e_ang;
                diff_theta = e_ang - diff_theta;
                
                pose_pub.publish(cd_vel);
                ros::spinOnce();                
            }
            
        }
};

PID::PID(){
    pose_sub = n.subscribe("/turtle1/pose",1000, &PID::PoseCallback,this);
    pose_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        getGoalPose();

        PID::PID1();
        PID::PID2();

        ros::spinOnce();

        loop_rate.sleep();
    
    }

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

    ros::init(argc, argv, "pid_turtle_pose_send_node");

    

    // ros::Rate loop_rate(10);    
    
    PID pid;   

  return 0;
}