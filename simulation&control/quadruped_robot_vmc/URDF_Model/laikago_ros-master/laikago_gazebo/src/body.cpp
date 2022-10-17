/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace laikago_model {

ros::Publisher servo_pub[12];
laikago_msgs::LowCmd lowCmd;
laikago_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].positionStiffness = 70;
        lowCmd.motorCmd[i*3+0].velocity = 0;
        lowCmd.motorCmd[i*3+0].velocityStiffness = 3;
        lowCmd.motorCmd[i*3+0].torque = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].positionStiffness = 180;
        lowCmd.motorCmd[i*3+1].velocity = 0;
        lowCmd.motorCmd[i*3+1].velocityStiffness = 8;
        lowCmd.motorCmd[i*3+1].torque = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].positionStiffness = 300;
        lowCmd.motorCmd[i*3+2].velocity = 0;
        lowCmd.motorCmd[i*3+2].velocityStiffness = 15;
        lowCmd.motorCmd[i*3+2].torque = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].position = lowState.motorState[i].position;
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

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].position;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].position = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}


}
