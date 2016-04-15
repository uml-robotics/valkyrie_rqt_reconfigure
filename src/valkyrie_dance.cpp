/* Jordan Allspaw
 * Sample program for controlling valkyrie with a ArmJointTrajectoryPacketMessage
 */
#include "ros/ros.h"
#include "ihmc_msgs/JointAnglesPacketMessage.h"
#include "ihmc_msgs/HandPosePacketMessage.h"
#include "ihmc_msgs/ArmJointTrajectoryPacketMessage.h"
#include "ihmc_msgs/JointTrajectoryPointMessage.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ihmc_msgs::ArmJointTrajectoryPacketMessage msg;
    ihmc_msgs::JointTrajectoryPointMessage larder;
    msg.robot_side = 1; 

/*    larder.positions.push_back(-0.19996967911720276); //Sends the "rest command", determined by looking at robot joint state when robot starts up.
    larder.positions.push_back(1.1997565031051636);
    larder.positions.push_back(0.7023981809616089);
    larder.positions.push_back(1.498468279838562);
    larder.positions.push_back(1.2941758632659912);
    larder.positions.push_back(-0.024003546684980392);
    larder.positions.push_back(-0.0453888401389122);
*/
    larder.positions.push_back(0.0); //Sends all 0's, should make robot stick his arm straight out.
    larder.positions.push_back(0.0);
    larder.positions.push_back(0.0);
    larder.positions.push_back(0.0);
    larder.positions.push_back(0.0);
    larder.positions.push_back(0.0);
    larder.positions.push_back(0.0);


    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    larder.velocities.push_back(0.0);
    
    larder.time = 20;

    msg.trajectory_points.push_back(larder);
    
    msg.unique_id=0;

    ros::Publisher pub = n.advertise<ihmc_msgs::ArmJointTrajectoryPacketMessage>("/ihmc_ros/valkyrie/control/arm_joint_trajectory", 1000);
    int i =0;
    while (ros::ok())
          {
              msg.unique_id=i++;
              pub.publish(msg);
              ros::spinOnce();
              loop_rate.sleep();
          }
    return 0;
}
