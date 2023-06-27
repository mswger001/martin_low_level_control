/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"
#include <thread>
#include <vector>

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000);
}
void move_groups(int group_number, double position, double duration)
{
    paramInit();

    double pos[12], lastPos[12], percent;
    for (int j = 0; j < 12; j++)
        lastPos[j] = lowState.motorState[j].q;

    std::vector<std::thread> threads;

    for (int i = 1; i <= duration; i++) {
        if (!ros::ok())
            break;

        percent = static_cast<double>(i) / duration;

        if (group_number == 0) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[0].q = lastPos[0] * (1 - percent) + position * percent;
                lowCmd.motorCmd[3].q = lastPos[3] * (1 - percent) + position * percent;
                lowCmd.motorCmd[6].q = lastPos[6] * (1 - percent) + position * percent;
                lowCmd.motorCmd[9].q = lastPos[9] * (1 - percent) + position * percent;
            });
        } else if (group_number == 1) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[1].q = lastPos[1] * (1 - percent) + position * percent;
                lowCmd.motorCmd[4].q = lastPos[4] * (1 - percent) + position * percent;
                lowCmd.motorCmd[7].q = lastPos[7] * (1 - percent) + position * percent;
                lowCmd.motorCmd[10].q = lastPos[10] * (1 - percent) + position * percent;
            });
        } else if (group_number == 2) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[2].q = lastPos[2] * (1 - percent) + position * percent;
                lowCmd.motorCmd[5].q = lastPos[5] * (1 - percent) + position * percent;
                lowCmd.motorCmd[8].q = lastPos[8] * (1 - percent) + position * percent;
                lowCmd.motorCmd[11].q = lastPos[11] * (1 - percent) + position * percent;
            });
        }

        sendServoCmd();
    }

    // Wait for all threads to finish
    for (auto& thread : threads) {
        thread.join();
    }
}

void move_groups1(int group_number,double position, double duration)
{   
    paramInit();

    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
     for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        //group 0 will have the following items 0
                                           //   3
                                           //   6
                                           //   9
 
        if(group_number==0){
            lowCmd.motorCmd[0].q = lastPos[0]*(1-percent) + position*percent;
            lowCmd.motorCmd[3].q = lastPos[3]*(1-percent) + position*percent;
            lowCmd.motorCmd[6].q = lastPos[6]*(1-percent) + position*percent;
            lowCmd.motorCmd[9].q = lastPos[9]*(1-percent) + position*percent;                
            
        }
        if(group_number==1){
            lowCmd.motorCmd[1].q = lastPos[1]*(1-percent) + position*percent;
            lowCmd.motorCmd[4].q = lastPos[4]*(1-percent) + position*percent;
            lowCmd.motorCmd[7].q = lastPos[7]*(1-percent) + position*percent;
            lowCmd.motorCmd[10].q = lastPos[10]*(1-percent) + position*percent; 
        }
        if(group_number==2){
            lowCmd.motorCmd[2].q = lastPos[2]*(1-percent) + position*percent;
            lowCmd.motorCmd[5].q = lastPos[5]*(1-percent) + position*percent;
            lowCmd.motorCmd[8].q = lastPos[8]*(1-percent) + position*percent;
            lowCmd.motorCmd[11].q = lastPos[11]*(1-percent) + position*percent; 
        }
        
        sendServoCmd();
    }
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}


}
