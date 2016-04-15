/*
The MIT License (MIT)
Copyright (c) <2016> <Jordan Allspaw>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "WholeBodyValkyrieControl.h"
#include<boost/thread/mutex.hpp>
ValkyrieController *valerie;

void drcallback(valkyrie_sandbox::valkyrieConfig &config, uint32_t level) {
    if (valerie->tfReady) {
        valerie->setPelvisPosition(false, config.hipx, config.hipy, config.hipz);
        valerie->setPelvisOrientation(false, config.pelvisRoll, config.pelvisPitch, config.pelvisYaw );
        valerie->setChestOrientation(false, config.chestRoll, config.chestPitch, config.chestYaw);
        
        valerie->setRightArm(true, config.rightJ1,config.rightJ2,config.rightJ3,config.rightJ4,config.rightJ5,config.rightJ6,config.rightJ7);
        valerie->setLeftArm(true, config.leftJ1,config.leftJ2,config.leftJ3,config.leftJ4,config.leftJ5,config.leftJ6,config.leftJ7);
        
        valerie->PublishCommand();
    }
}

void ValkyrieController::callback(geometry_msgs::Vector3 msg) {
    setPelvisPosition(false, msg.x, msg.y, msg.z);
    //setPelvisOrientation(config.pelvisRoll, config.pelvisPitch, config.pelvisYaw );
    //setChestOrientation(config.chestRoll, config.chestPitch, config.chestYaw);
    //PrepareTransform(msg.x, msg.y, msg.z,0.0,0.0,0.0);
}

void ValkyrieController::jointStatesCallback(sensor_msgs::JointState msg) {
    jointState = msg;
    /*
    rightArm.positions[0] = msg.position[26];
    rightArm.positions[1] = msg.position[27];
    rightArm.positions[2] = msg.position[28];
    rightArm.positions[3] = msg.position[29];
    rightArm.positions[4] = msg.position[30];
    rightArm.positions[5] = msg.position[31];
    rightArm.positions[6] = msg.position[32];
    
    leftArm.positions[0] = msg.position[20];
    leftArm.positions[1] = msg.position[21];
    leftArm.positions[2] = msg.position[22];
    leftArm.positions[3] = msg.position[23];
    leftArm.positions[4] = msg.position[24];
    leftArm.positions[5] = msg.position[25];
    leftArm.positions[6] = msg.position[26]; */
}

tf::StampedTransform ValkyrieController::getTransform(std::string source, std::string target) {
    tf::StampedTransform transform;
    try {
        ros::Time time = ros::Time::now();
        tflistener.waitForTransform(source, target, time, ros::Duration(10.0));
        tflistener.lookupTransform(source, target, time, transform);
    } catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    return transform;
}

geometry_msgs::Vector3 ValkyrieController::transformPelvis(tf::Vector3 vec) {
    tf::StampedTransform localPelvisTransform = GlobalPelvisTransform;
    vec += localPelvisTransform.getOrigin();
    geometry_msgs::Vector3 rtn;
    tf::vector3TFToMsg(vec, rtn);
    return rtn;
}

geometry_msgs::Quaternion ValkyrieController::transformPelvis( tf::Quaternion rot) {
    tf::StampedTransform localPelvisTransform = GlobalPelvisTransform;
    localPelvisTransform.setRotation(localPelvisTransform * rot);
    geometry_msgs::Quaternion newquat;
    tf::quaternionTFToMsg(localPelvisTransform.getRotation(), newquat);
    return newquat;
}

geometry_msgs::Quaternion ValkyrieController::transformChest( tf::Quaternion rot) {
    tf::StampedTransform localChestTransform = GlobalChestTransform;
    localChestTransform.setRotation(localChestTransform * rot);
    geometry_msgs::Quaternion newquat;
    tf::quaternionTFToMsg(localChestTransform.getRotation(), newquat);
    return newquat;
}

ihmc_msgs::JointTrajectoryPointMessage ValkyrieController::prepareArm(ihmc_msgs::JointTrajectoryPointMessage arm_point , float j1 = 0.0, float j2 = 0.0, float j3 = 0.0, float j4 = 0.0, float j5 = 0.0, float j6 = 0.0, float j7 = 0.0) {
    arm_point.positions[0] += j1;
    arm_point.positions[1] += j2;
    arm_point.positions[2] += j3;
    arm_point.positions[3] += j4;
    arm_point.positions[4] += j5;
    arm_point.positions[5] += j6;
    arm_point.positions[6] += j7;
    
    arm_point.time = 5.0;
    return arm_point;
}

void ValkyrieController::setPelvisPosition(bool absolute, float x, float y, float z) {
    boost::mutex::scoped_lock l(cloudlock);
    if(absolute)
    {
        geometry_msgs::Vector3 vec;
        tf::vector3TFToMsg(tf::Vector3(x,y,z) , vec);
        msg.pelvis_world_position[0] = vec;
    }
    else
        msg.pelvis_world_position[0] = transformPelvis(tf::Vector3(x,y,z));
}

void ValkyrieController::setPelvisOrientation(bool absolute, float roll, float pitch, float yaw) {
    boost::mutex::scoped_lock l(cloudlock);
    if(absolute)
    {
        geometry_msgs::Quaternion quat;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll,pitch,yaw),quat);
        msg.pelvis_world_orientation[0] = quat;
    }
    else
        msg.pelvis_world_orientation[0] = transformPelvis(tf::createQuaternionFromRPY(roll,pitch,yaw));
}

void ValkyrieController::setChestOrientation(bool absolute, float roll, float pitch, float yaw) {
    boost::mutex::scoped_lock l(cloudlock);
    if(absolute)
    {
        geometry_msgs::Quaternion quat;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll,pitch,yaw),quat);
        msg.chest_world_orientation[0] = quat;
    }
    else
        msg.chest_world_orientation[0] = transformChest( tf::createQuaternionFromRPY(roll,pitch,yaw));
}

void ValkyrieController::setRightArm(bool absolute, float j1, float j2, float j3, float j4, float j5, float j6, float j7) {
    if(absolute) {
        rightArm.positions[0] = j1;
        rightArm.positions[1] = j2;
        rightArm.positions[2] = j3;
        rightArm.positions[3] = j4;
        rightArm.positions[4] = j5;
        rightArm.positions[5] = j6;
        rightArm.positions[6] = j7;
    }
    else {
        rightArm.positions[0] = j1 + jointState.position[26];
        rightArm.positions[1] = j2 + jointState.position[27];
        rightArm.positions[2] = j3 + jointState.position[28];
        rightArm.positions[3] = j4 + jointState.position[29];
        rightArm.positions[4] = j5 + jointState.position[30];
        rightArm.positions[5] = j6 + jointState.position[31];
        rightArm.positions[6] = j7 + jointState.position[32];
    }

    for(int i = 0; i < 7; i++)
    {
        msg.right_arm_trajectory.trajectory_points[0].positions = rightArm.positions;
    }
}

void ValkyrieController::setLeftArm(bool absolute, float j1, float j2, float j3, float j4, float j5, float j6, float j7) {
    if(absolute)
    {
        leftArm.positions[0] = j1;
        leftArm.positions[1] = j2;
        leftArm.positions[2] = j3;
        leftArm.positions[3] = j4;
        leftArm.positions[4] = j5;
        leftArm.positions[5] = j6;
        leftArm.positions[6] = j7;
    } 
    else {    
        leftArm.positions[0] = j1 + jointState.position[20];
        leftArm.positions[1] = j2 + jointState.position[21];
        leftArm.positions[2] = j3 + jointState.position[22];
        leftArm.positions[3] = j4 + jointState.position[23];
        leftArm.positions[4] = j5 + jointState.position[24];
        leftArm.positions[5] = j6 + jointState.position[25];
        leftArm.positions[6] = j7 + jointState.position[26];
    }
    for(int i = 0; i < 7; i++)
    {
        msg.left_arm_trajectory.trajectory_points[0].positions = leftArm.positions;
    }
}

void ValkyrieController::PublishCommand() {
    pub.publish(msg);
    ROS_INFO("PUBLISHING");
}

void ValkyrieController::PublishRelativeCommand() {
    ihmc_msgs::WholeBodyTrajectoryPacketMessage relativemsg = msg;
    for(int i = 0; i < 7; i++)
    {
        relativemsg.right_arm_trajectory.trajectory_points[0].positions[i] += rightArm_offset[i];
        relativemsg.left_arm_trajectory.trajectory_points[0].positions[i] += leftArm_offset[i];
    }
    pub.publish(msg);
}

void ValkyrieController::PrepareTransform(float x = 0.0, float y = 0.0, float z = 0.9, float roll = 0.0, float pitch = 0.0, float yaw = 0.0) {
    ROS_INFO("SETTING UP");
    sendCommand = false;
    msg.pelvis_world_position.push_back(transformPelvis(tf::Vector3(x,y,z)));
    
    msg.chest_world_orientation.push_back(transformChest( tf::createQuaternionFromRPY(roll,pitch,yaw)));

    ROS_INFO("Transform Ready!");
    //pub.publish(msg);
}

ValkyrieController::ValkyrieController(ros::NodeHandle nh) {
    n = nh;
    //sub = n.subscribe("/helllooo", 1000, &ValkyrieController::callback, this);
}

void ValkyrieController::setup() {
    tfReady = false;

    sub = n.subscribe("/helllooo", 1000, &ValkyrieController::callback, this);
    jointStatesSub = n.subscribe("/ihmc_ros/valkyrie/output/joint_states", 1000, &ValkyrieController::jointStatesCallback, this);
    pub = n.advertise<ihmc_msgs::WholeBodyTrajectoryPacketMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1000);
    timer = n.createTimer(ros::Duration(0.1), &ValkyrieController::timerCallback,this);
    
    pelvis_world_orientation.w = 1.0;

    chest_world_orientation.w = 1.0;
    
    msg.time_at_waypoint.push_back(2.0);

    msg.pelvis_world_position.push_back(transformPelvis(tf::Vector3(0.0,0.0,0.0)));
    msg.pelvis_linear_velocity.push_back(pelvis_linear_velocity);
    msg.pelvis_angular_velocity.push_back(pelvis_angular_velocity);
    msg.pelvis_world_orientation.push_back(pelvis_world_orientation);

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    
    msg.chest_world_orientation.push_back(transformChest(tf_chest_world_orientation));
    msg.chest_angular_velocity.push_back(chest_angular_velocity);
        
    left_arm_trajectory.robot_side = 0;
    right_arm_trajectory.robot_side = 1;
    left_arm_trajectory.unique_id = 42;
    right_arm_trajectory.unique_id = 42;
    
    for(int k = 0; k < 7; k++) {
        rightArm.positions.push_back(0.0);
        rightArm.velocities.push_back(0.0);
        leftArm.positions.push_back(0.0);
        leftArm.velocities.push_back(0.0);
        leftArm_offset.push_back(0.0);
        rightArm_offset.push_back(0.0);
   }
   rightArm.time = 5.0;
   leftArm.time = 5.0;
    
    right_arm_trajectory.trajectory_points.push_back(prepareArm(rightArm));
    left_arm_trajectory.trajectory_points.push_back(prepareArm(leftArm));


    msg.right_arm_trajectory = right_arm_trajectory;
    msg.left_arm_trajectory  = left_arm_trajectory;

    msg.num_waypoints = 1;
    msg.num_joints_per_arm = 7;
    msg.unique_id = 42;
}

void ValkyrieController::spin() {
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown(); 
}

ValkyrieController::~ValkyrieController() {
}

void ValkyrieController::timerCallback(const ros::TimerEvent&) {
    tf::StampedTransform _pelvisTransform, _chestTransform;
    ROS_INFO_THROTTLE(1,"Spinning");
    try {
        ros::Time time = ros::Time::now();
        tflistener.waitForTransform("/world", "/pelvis", time, ros::Duration(1.0));
        tflistener.lookupTransform("/world", "/pelvis", time, _pelvisTransform);
        
        tflistener.waitForTransform("/world", "/torso", time, ros::Duration(1.0));
        tflistener.lookupTransform("/world", "/torso", time, _chestTransform);
        tfReady = true;
        
    } catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    
    boost::mutex::scoped_lock l(cloudlock);
    GlobalPelvisTransform = _pelvisTransform;
    GlobalChestTransform = _chestTransform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WholeBodyValkyrieControl");
    ros::NodeHandle n(std::string("~"));
    valerie = new ValkyrieController(n);
    valerie->setup();
    
    dynamic_reconfigure::Server<valkyrie_sandbox::valkyrieConfig> server;
    dynamic_reconfigure::Server<valkyrie_sandbox::valkyrieConfig>::CallbackType f;
    f = boost::bind(&drcallback, _1, _2);
    server.setCallback(f);
    
    ros::Duration(2.0).sleep();
    ROS_INFO("All functions ready");
    
    valerie->spin();
    return 0;
}
