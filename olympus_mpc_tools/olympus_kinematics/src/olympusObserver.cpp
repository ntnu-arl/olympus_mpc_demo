#include "ros/ros.h"
#include "leg_kinematics.hpp"
#include "sensor_msgs/JointState.h"
#include "olympus_ros_msgs/MotorStatusArray.h"
#include "olympus_ros_msgs/MotorStatus.h"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <algorithm>

#define encoder_freq 100.

class state_estimator {
    public:		
        state_estimator()
        {   
            //Create Publishers-Subscribers
            ros::NodeHandle n;
            jointPub   = n.advertise<sensor_msgs::JointState>("/joint_states",10);

            leg1_encoder= n.subscribe("/leg1_node/motor_statuses", 10, &state_estimator::read_motors,this);
            leg2_encoder= n.subscribe("/leg2_node/motor_statuses", 10, &state_estimator::read_motors,this);
            leg3_encoder= n.subscribe("/leg3_node/motor_statuses", 10, &state_estimator::read_motors,this);
            leg4_encoder= n.subscribe("/leg4_node/motor_statuses", 10, &state_estimator::read_motors,this);


            // //Read parameters (if no yaml is loaded -> script compatible for custom sim)
            if ( !n.getParam("/olympus_observer/robot/name", full_joint_state.name) )
            {   
                //If no parameter load -> means custom simulation setup
                std::array<std::string,4> suffix  = {"fr","rr","fl","rl"};
                for (int  i = 0; i < 4; i++)
                {
                    full_joint_state.name.push_back(suffix[i]+"_jointMH"); 
                    full_joint_state.name.push_back(suffix[i]+"_joint11"); 
                    full_joint_state.name.push_back(suffix[i]+"_joint21"); 
                    full_joint_state.name.push_back(suffix[i]+"_joint12"); 
                    full_joint_state.name.push_back(suffix[i]+"_joint22"); 
                }

                //Accept radias as in the sim
                conversion_factor = 1;
            } else {
                //Accept degrees as in the sim
                conversion_factor = M_PI/180.;
            }


            std::vector<int>    motor_indices;
            std::vector<int>    signs_;

            //Motor ids
            if(n.getParam("/olympus_observer/robot/motor_ids",motor_indices)) {
                std::copy(motor_indices.begin(),motor_indices.end(),motor_id.begin());
            } else{
                motor_id = {1,2,3,4,5,6,7,8,9,10,11,12};
                ROS_INFO("[observer]: Did not read motor ids");
            }
            
            if ( n.getParam("/olympus_observer/robot/robot_signs", signs_) ){
                for(int i =0;i<12;i++){
                    signs[i] = signs_[i];
                }
            }else {
                ROS_INFO("[Observer] Using default signs");
            }

            ROS_INFO_STREAM("Signs:");
            for(auto sign:signs){
                ROS_INFO_STREAM(sign);
            }
            

            //Initialize measured state:
            measured_joint_state.position.assign(12,0);
            measured_joint_state.velocity.assign(12,0);
            measured_joint_state.effort.assign(12,0);

            //Initialize joint state message:
            full_joint_state.position.assign(20,0);
            full_joint_state.velocity.assign(20,0);
            full_joint_state.effort.assign(20,0);
        }

    void state_estimation(){
        auto & pos = measured_joint_state.position;

        Eigen::Vector3d leg_v;
        Eigen::Matrix<double,5,1> q;
        Eigen::Vector3d qr;

        for (int ic = 0; ic < 4; ic++)
        {            
            int * id = &motor_id[3*ic]; // with this we ensure the order is [fr,rr,fl,rl], as the estimator order
            qr << pos[*(id++)],pos[*(id++)],pos[*id];

            q = estimators[ic].state_estimation(conversion_factor*qr);

            //Populate message
            for (int i = 0; i < 5; i++)
            {   
                full_joint_state.position[5*ic+i] = q[i];
            }
        }

        full_joint_state.header.stamp = ros::Time::now(); 
        jointPub.publish(full_joint_state);
    }

    private:
    void read_motors(const olympus_ros_msgs::MotorStatusArray::ConstPtr & msg){  
        for(auto status:msg->statuses ){
            measured_joint_state.position[ status.id-1 ] = signs[status.id-1] * status.position;
            measured_joint_state.velocity[ status.id-1 ] = signs[status.id-1] * status.velocity;
            measured_joint_state.effort  [ status.id-1 ] = signs[status.id-1] * status.torque;
        }
    }

    sensor_msgs::JointState measured_joint_state;
    sensor_msgs::JointState full_joint_state;

    //Read motors
    ros::Subscriber leg1_encoder ;
    ros::Subscriber leg2_encoder ;
    ros::Subscriber leg3_encoder ;
    ros::Subscriber leg4_encoder ;
    std::array<double,12> signs{  1,1,1, 1,1,1, 1,1,1, 1,1,1 };

    ros::Publisher jointPub    ;
    ros::Subscriber encoderSub ;

    std::array<leg_kinematics,4> estimators{leg_kinematics(0),leg_kinematics(1),leg_kinematics(2),leg_kinematics(3)};

    std::array<int,12> motor_id ;  //motor_indices
    double conversion_factor = 1; 
};

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "leg_observer");
ros::NodeHandle n;

double loop_freq = encoder_freq ;
std::cout << loop_freq <<std::endl;
ros::Rate loop_rate(loop_freq); //HZ

#pragma endregion

state_estimator estimator;

while(ros::ok()){

    ros::spinOnce(); //Update state
    estimator.state_estimation(); //estimate state and publish
    loop_rate.sleep();
}
    return 0;
}


