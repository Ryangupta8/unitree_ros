#include <PnC/A1PnC/A1Interface.hpp>
#include <Utils/IO/IOUtilities.hpp>

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "Eigen/Dense"
#include <math.h>

#define PI 3.14159265

class RealRobotInit {
  private:
    Eigen::VectorXd desired_config, q_curr, qdot_curr;
  public:
    Eigen::VectorXd starting_config;
    Eigen::VectorXd jpos_cmd;
    double curr_time;

    RealRobotInit(){
        starting_config = Eigen::VectorXd::Zero(12);

        desired_config << 0., 0.7854, -1.57, 0., 0.7854, -1.57, 0, 0.7854, -1.57, 0., 0.7854, -1.57;

        jpos_cmd = Eigen::VectorXd::Zero(12);
        q_curr = Eigen::VectorXd::Zero(12);
        qdot_curr = Eigen::VectorXd::Zero(12);

        curr_time = 0;

    }

    void smooth_changing(int idx) {
        jpos_cmd[idx] = starting_config[idx] + (desired_config[idx] - starting_config[idx]) *
                        0.5 * (1. - cos(curr_time / 5000. * PI));
        if (curr_time > 5000.) {
            jpos_cmd[idx] = desired_config[idx];
        }
    }

};




using namespace UNITREE_LEGGED_SDK;

void* update_loop(void* param) {
    LCM *data = (LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Init ROS Node
    ros::init(argc, argv, "wbic_ros");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // Robot Initializatioon
    RealRobotInit* robot_init;
    robot_init = new RealRobotInit();
    bool init = 1;

    // PD Gains
    Eigen::VectorXd Kp, Kd;
    Kp << 130., 110., 140., 130., 110., 140., 130., 110., 140., 130., 110., 140.;
    Kd << 6., 6., 6., 6., 6., 6., 6., 6., 6., 6., 6., 6.;

    // Robot Controller
    LeggedType rname;
    rname = LeggedType::A1;
    Control control(rname, LOWLEVEL);

    // LCM Comms Init
    long motiontime=0;
    float torque = 0;
    LowCmd SendLowLCM = {0};
    LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }
    A1Interface* interface_;
    A1SensorData* sensordata_;
    A1Command* command_;

    interface_ = new A1Interface();
    sensordata_ = new A1SensorData();
    command_ = new A1Command();

    // First set motor PD gains
    SendLowROS.motorCmd[FL_0].Kp = Kp[0];
    SendLowROS.motorCmd[FL_1].Kp = Kp[1];
    SendLowROS.motorCmd[FL_2].Kp = Kp[2];
    SendLowROS.motorCmd[FL_0].Kd = Kd[0];
    SendLowROS.motorCmd[FL_1].Kd = Kd[1];
    SendLowROS.motorCmd[FL_2].Kd = Kd[2];

    SendLowROS.motorCmd[FR_0].Kp = Kp[3];
    SendLowROS.motorCmd[FR_1].Kp = Kp[4];
    SendLowROS.motorCmd[FR_2].Kp = Kp[5];
    SendLowROS.motorCmd[FR_0].Kd = Kd[3];
    SendLowROS.motorCmd[FR_1].Kd = Kd[4];
    SendLowROS.motorCmd[FR_2].Kd = Kd[5];

    SendLowROS.motorCmd[RL_0].Kp = Kp[6];
    SendLowROS.motorCmd[RL_1].Kp = Kp[7];
    SendLowROS.motorCmd[RL_2].Kp = Kp[8];
    SendLowROS.motorCmd[RL_0].Kd = Kd[6];
    SendLowROS.motorCmd[RL_1].Kd = Kd[7];
    SendLowROS.motorCmd[RL_2].Kd = Kd[8];

    SendLowROS.motorCmd[RR_0].Kp = Kp[9];
    SendLowROS.motorCmd[RR_1].Kp = Kp[10];
    SendLowROS.motorCmd[RR_2].Kp = Kp[11];
    SendLowROS.motorCmd[RR_0].Kd = Kd[9];
    SendLowROS.motorCmd[RR_1].Kd = Kd[10];
    SendLowROS.motorCmd[RR_2].Kd = Kd[11];


    while (ros::ok()){
        // If we are starting, set robot_init->starting_config
        if(init) { // Robot init time

            for(int i=0; i<100; ++i) {
                roslcm.Get(RecvLowLCM);
                RecvLowROS = ToRos(RecvLowLCM);
            }
            robot_init->starting_config[0] = RecvLowROS.motorState[FL_0].q;
            robot_init->starting_config[1] = RecvLowROS.motorState[FL_1].q;
            robot_init->starting_config[2] = RecvLowROS.motorState[FL_2].q;
            robot_init->starting_config[3] = RecvLowROS.motorState[FR_0].q;
            robot_init->starting_config[4] = RecvLowROS.motorState[FR_1].q;
            robot_init->starting_config[5] = RecvLowROS.motorState[FR_2].q;
            robot_init->starting_config[6] = RecvLowROS.motorState[RL_0].q;
            robot_init->starting_config[7] = RecvLowROS.motorState[RL_1].q;
            robot_init->starting_config[8] = RecvLowROS.motorState[RL_2].q;
            robot_init->starting_config[9] = RecvLowROS.motorState[RR_0].q;
            robot_init->starting_config[10] = RecvLowROS.motorState[RR_1].q;
            robot_init->starting_config[11] = RecvLowROS.motorState[RR_2].q;
            init = 0;
        } else { // Controller time
            motiontime++;
            roslcm.Get(RecvLowLCM);
            RecvLowROS = ToRos(RecvLowLCM);
        }

        // For some motiontime we need to get to init position before we run controller
        if(robot_init->curr_time <= 5000) { // 5 seconds --> Stand up to initial pos
            for(int i=0; i<12; ++i) {
                robot_init->smooth_changing(i);
            }
            SendLowROS.motorCmd[FL_0].q = robot_init->jpos_cmd[0];
            SendLowROS.motorCmd[FL_1].q = robot_init->jpos_cmd[1];
            SendLowROS.motorCmd[FL_2].q = robot_init->jpos_cmd[2];
            SendLowROS.motorCmd[FR_0].q = robot_init->jpos_cmd[3];
            SendLowROS.motorCmd[FR_1].q = robot_init->jpos_cmd[4];
            SendLowROS.motorCmd[FR_2].q = robot_init->jpos_cmd[5];
            SendLowROS.motorCmd[RL_0].q = robot_init->jpos_cmd[6];
            SendLowROS.motorCmd[RL_1].q = robot_init->jpos_cmd[7];
            SendLowROS.motorCmd[RL_2].q = robot_init->jpos_cmd[8];
            SendLowROS.motorCmd[RR_0].q = robot_init->jpos_cmd[9];
            SendLowROS.motorCmd[RR_1].q = robot_init->jpos_cmd[10];
            SendLowROS.motorCmd[RR_2].q = robot_init->jpos_cmd[11];

        } else { // Run the controller
            // Collect Sensor Data
            sensordata_->q[0] = RecvLowROS.motorState[FL_0].q;
            sensordata_->q[1] = RecvLowROS.motorState[FL_1].q;
            sensordata_->q[2] = RecvLowROS.motorState[FL_2].q;
            sensordata_->q[3] = RecvLowROS.motorState[FR_0].q;
            sensordata_->q[4] = RecvLowROS.motorState[FR_1].q;
            sensordata_->q[5] = RecvLowROS.motorState[FR_2].q;
            sensordata_->q[6] = RecvLowROS.motorState[RL_0].q;
            sensordata_->q[7] = RecvLowROS.motorState[RL_1].q;
            sensordata_->q[8] = RecvLowROS.motorState[RL_2].q;
            sensordata_->q[9] = RecvLowROS.motorState[RR_0].q;
            sensordata_->q[10] = RecvLowROS.motorState[RR_1].q;
            sensordata_->q[11] = RecvLowROS.motorState[RR_2].q;

            sensordata_->qdot[0] = RecvLowROS.motorState[FL_0].dq;
            sensordata_->qdot[1] = RecvLowROS.motorState[FL_1].dq;
            sensordata_->qdot[2] = RecvLowROS.motorState[FL_2].dq;
            sensordata_->qdot[3] = RecvLowROS.motorState[FR_0].dq;
            sensordata_->qdot[4] = RecvLowROS.motorState[FR_1].dq;
            sensordata_->qdot[5] = RecvLowROS.motorState[FR_2].dq;
            sensordata_->qdot[6] = RecvLowROS.motorState[RL_0].dq;
            sensordata_->qdot[7] = RecvLowROS.motorState[RL_1].dq;
            sensordata_->qdot[8] = RecvLowROS.motorState[RL_2].dq;
            sensordata_->qdot[9] = RecvLowROS.motorState[RR_0].dq;
            sensordata_->qdot[10] = RecvLowROS.motorState[RR_1].dq;
            sensordata_->qdot[11] = RecvLowROS.motorState[RR_2].dq;
 
        }

        robot_init->curr_time += 2;

        SendLowLCM = ToLcm(SendLowROS);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
