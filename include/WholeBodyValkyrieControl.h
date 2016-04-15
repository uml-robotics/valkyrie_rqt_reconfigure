/*
The MIT License (MIT)
Copyright (c) <2016> <Jordan Allspaw>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __WHOLEBODYVALKYRIECONTROL_H_INCLUDED__
#define __WHOLEBODYVALKYRIECONTROL_H_INCLUDED__

#include "ros/ros.h"
#include "ihmc_msgs/WholeBodyTrajectoryPacketMessage.h"
#include "ihmc_msgs/ArmJointTrajectoryPacketMessage.h"
#include "ihmc_msgs/JointTrajectoryPointMessage.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include "valkyrie_rqt_reconfigure/valkyrieConfig.h"
#include "sensor_msgs/JointState.h"

#include <dynamic_reconfigure/server.h>

class ValkyrieController {
public:
    ValkyrieController(ros::NodeHandle nh);
    virtual ~ValkyrieController();
    boost::mutex cloudlock;
    void PrepareTransform(float x, float y, float z, float roll, float pitch, float yaw);
    void callback(geometry_msgs::Vector3 msg);
    void drcallback(valkyrie_sandbox::valkyrieConfig &config, uint32_t level);
    tf::StampedTransform getTransform(std::string source, std::string target);
    tf::StampedTransform GlobalPelvisTransform;
    tf::StampedTransform GlobalChestTransform;
    
    bool sendCommand;
    bool tfReady;
    
    void jointStatesCallback(sensor_msgs::JointState msg);
    
    ihmc_msgs::WholeBodyTrajectoryPacketMessage msg;
    ihmc_msgs::ArmJointTrajectoryPacketMessage right_arm_trajectory;
    ihmc_msgs::ArmJointTrajectoryPacketMessage left_arm_trajectory;

    ihmc_msgs::JointTrajectoryPointMessage right_arm_point;
    ihmc_msgs::JointTrajectoryPointMessage left_arm_point;

    geometry_msgs::Vector3 pelvis_world_position;
    geometry_msgs::Vector3 pelvis_linear_velocity;
    geometry_msgs::Vector3 pelvis_angular_velocity;
    geometry_msgs::Quaternion pelvis_world_orientation;
    
    geometry_msgs::Quaternion chest_world_orientation;
    geometry_msgs::Vector3 chest_angular_velocity;
    
    sensor_msgs::JointState jointState;
    
    void timerCallback(const ros::TimerEvent&);
    geometry_msgs::Vector3 transformPelvis(tf::Vector3 vec);
    geometry_msgs::Quaternion transformPelvis(tf::Quaternion rot);
    geometry_msgs::Quaternion transformChest(tf::Quaternion rot);
    
    tf::Quaternion tf_chest_world_orientation;
    
    void setPelvisPosition(bool absolute, float x, float y, float z);
    void setPelvisOrientation(bool absolute, float roll, float pitch, float yaw);
    void setChestOrientation(bool absolute, float roll, float pitch, float yaw);
    
    void PublishCommand();
    
    ihmc_msgs::JointTrajectoryPointMessage rightArm;
    std::vector<float> rightArm_offset;
    ihmc_msgs::JointTrajectoryPointMessage leftArm;
    std::vector<float> leftArm_offset;

    void setRightArm(bool absolute, float j1, float j2, float j3, float j4, float j5, float j6, float j7);
    void setLeftArm(bool absolute, float j1, float j2, float j3, float j4, float j5, float j6, float j7);
    void PublishRelativeCommand();
    ihmc_msgs::JointTrajectoryPointMessage prepareArm(ihmc_msgs::JointTrajectoryPointMessage arm_point, float j1, float j2, float j3, float j4, float j5, float j6, float j7);
    void spin();
    void setup();
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber jointStatesSub;
    ros::Publisher pub;
    ros::Timer timer;
    tf::TransformListener tflistener;
    //dynamic_reconfigure::Server<valkyrie_sandbox::valkyrieConfig> server;
};

#endif 
