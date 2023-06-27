/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;


void motion_init();
void joint_motion_init(double targetPos, int joint_number, double duration);
void sendServoCmd();
void moveJoint(int joint_number, double targetPos, double duration);
void sendServoCmdHinge(int joint);


}

#endif
