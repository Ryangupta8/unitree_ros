/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Author: Ryan Gupta


#include <unitree_legged_real/A1HighDriver.h>


// using namespace UNITREE_LEGGED_SDK;
// using namespace std;


void* update_loop(void* param)
{
    UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm)
{
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    ros.rpy[0] = lcm.rpy[0];
    ros.rpy[1] = lcm.rpy[1];
    ros.rpy[2] = lcm.rpy[2];
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm)
{
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.mode = lcm.mode;
    ros.imu = ToRos(lcm.imu);
    ros.forwardSpeed = lcm.forwardSpeed;
    ros.sideSpeed = lcm.sideSpeed;
    ros.rotateSpeed = lcm.rotateSpeed;
    ros.bodyHeight = lcm.bodyHeight;
    ros.updownSpeed = lcm.updownSpeed;
    ros.forwardPosition = lcm.forwardPosition;
    ros.sidePosition = lcm.sidePosition;
    for(int i = 0; i<4; i++){
        ros.footPosition2Body[i].x = lcm.footPosition2Body[i].x;
        ros.footPosition2Body[i].y = lcm.footPosition2Body[i].y;
        ros.footPosition2Body[i].z = lcm.footPosition2Body[i].z;
        ros.footSpeed2Body[i].x = lcm.footSpeed2Body[i].x;
        ros.footSpeed2Body[i].y = lcm.footSpeed2Body[i].y;
        ros.footSpeed2Body[i].z = lcm.footSpeed2Body[i].z;
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.mode = ros.mode;
    lcm.forwardSpeed = ros.forwardSpeed;
    lcm.sideSpeed = ros.sideSpeed;
    lcm.rotateSpeed = ros.rotateSpeed;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.yaw = ros.yaw;
    lcm.pitch = ros.pitch;
    lcm.roll = ros.roll;
    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
        lcm.AppRemote[i] = ros.AppRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}


A1HighDriver::A1HighDriver(){

	motiontime = 0;
	SendHighLCM = {0};
	RecvHighLCM = {0};

	roslcm.SubscribeState();

	pthread_create(&tid, NULL, update_loop, &roslcm);
}

void A1HighDriver::GetState(){
	roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);
}

void A1HighDriver::SendCmd(){
	SendHighLCM = ToLcm(SendHighROS);
    roslcm.Send(SendHighLCM);
}