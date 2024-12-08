#include "olympus_drake_ros.hpp"

drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context ){    
  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock = ros::Time(context.get_time());
  pub.publish(clock_msg);

  return drake::systems::EventStatus::Succeeded();
}


olympus_ros_interface::olympus_ros_interface(drake_ros_elements & drake_elements)
    : simulator(drake_elements.simulator),
      input_ports(drake_elements.leg_input_ports),
      output_ports(drake_elements.leg_output_ports),
      body_output_port(drake_elements.body_output_port),
      actuation_output_ports(drake_elements.leg_actuation_output_ports),
      base_joint_type(drake_elements.base_joint_type) {

  ros::NodeHandle n;
  clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

  std::string body_orientation_topic_name ; 
  if( !n.getParam("body_publisher_topic",body_orientation_topic_name ) ){
    body_orientation_topic_name = "olympus/odom"; 
    ROS_WARN_STREAM(" [drake_sim_ros] Couldn't read the \'body_publisher_topic\' parameter. Using the default option: "<< body_orientation_topic_name);
  }

  std::map<std::string, int> leg_order_map;
  if( !n.getParam("leg_order",leg_order_map ) ){
    //Default value
    leg_order_map =  {
        {"FrontRight", 2},
        {"BackRight", 4},
        {"FrontLeft", 1},
        {"BackLeft", 3}
    };

    ROS_WARN(" [drake_sim_ros] Couldn't read the \'leg_order\' parameter. Using the initial order [FL,FR,BL,BR] which may be outdated.");
  }

  body_orientation_pub = n.advertise<nav_msgs::Odometry>(body_orientation_topic_name,10);

  body_odom.header.frame_id = "Body_frame";
  body_odom.child_frame_id  = "world_frame";
  body_odom.pose.pose.orientation.w = 1; //default value for quaternion


  //Set subs/pubs for each leg system
  std::vector<int> hw_order ;  //hw order of legs in sim
  olympus_ros_msgs::MotorStatus ms;
  for ( int i=0; i <num_legs ; i++){
    int leg_number = leg_order_map[ olympus_plant::leg_prefix[i] ] ; 
    std::string leg_node_name = "leg" + std::to_string( leg_number ) + "_node";

    encoder_pub  .push_back( n.advertise<olympus_ros_msgs::MotorStatusArray>(leg_node_name+"/motor_statuses",10) );
    commander_sub.push_back( n.subscribe(leg_node_name+"/command_position", 10, &olympus_ros_interface::reference_callback,this) );  
  
    //index for each motor in each leg
    for (int j=1; j<num_DOF_leg+1 ;j++){
      ms.id = num_DOF_leg*(leg_number-1) +j ; 

      //Will never publish these values, as first reads before publishing. 
      ms.position = 0;
      ms.velocity = 0;
      ms.torque = 0;       

      motor_statuses[i].statuses.push_back(ms);   
    }

    hw_order.push_back( leg_number - 1);
  }

  std::vector<int> temp = ascending_indices(hw_order); //Inverse: Get the sim order of the legs in hw
  std::copy(temp.begin(),temp.end(),leg_order.begin());
}

void olympus_ros_interface::publish(){

  auto& context = simulator.get_context();
  //publish leg status:
  for (int i_leg=0; i_leg<num_legs; i_leg++){
    //publish leg states:
    auto&  state  =  output_ports[ i_leg ]->Eval(context);
    auto&  torque = actuation_output_ports[i_leg]->Eval(context);

    for(int j=0; j < num_DOF_leg;j++){
      
      motor_statuses[i_leg].statuses[motor_order[j]].position = state[ state_ids[j            ] ]*180/M_PI;
      motor_statuses[i_leg].statuses[motor_order[j]].velocity = state[ state_ids[j+num_DOF_leg] ]*180/M_PI;
    
    //publish leg actuations -> output ports in leg_prefix fashion -> [MH,HI,HO]
    motor_statuses[i_leg].statuses[motor_order[j]].torque = torque [j];    
    }

    motor_statuses[i_leg].header.stamp = ros::Time::now();
    encoder_pub[i_leg].publish(motor_statuses[i_leg]);
  }

  //publish body orientation:
  auto&  body_state =  body_output_port->Eval(context);
  body_odom.pose.pose.orientation.w = body_state[0];
  body_odom.pose.pose.orientation.x = body_state[1];
  body_odom.pose.pose.orientation.y = body_state[2];
  body_odom.pose.pose.orientation.z = body_state[3];
  body_odom.pose.pose.position.x    = body_state[4];
  body_odom.pose.pose.position.y    = body_state[5];
  body_odom.pose.pose.position.z    = body_state[6];

  body_odom.twist.twist.angular.x = body_state[7];
  body_odom.twist.twist.angular.y = body_state[8];
  body_odom.twist.twist.angular.z = body_state[9];
  body_odom.twist.twist.linear.x = body_state[10];
  body_odom.twist.twist.linear.y = body_state[11];
  body_odom.twist.twist.linear.z = body_state[12];

  body_odom.header.stamp = ros::Time::now();
  body_orientation_pub.publish(body_odom);

}

void olympus_ros_interface::set_monitor(){
  //a. Monitor Function:
  auto bindedClockFun = std::bind(publishClock,clock_publisher,std::placeholders::_1);
  // Create a function object
  std::function<drake::systems::EventStatus(const drake::systems::Context<double> & )> monitor_func = bindedClockFun;
  // Pass the monitor function to the simulator
  simulator.set_monitor(monitor_func);
}

void olympus_ros_interface::reference_callback(const olympus_ros_msgs::SetpointArray::ConstPtr& setpoint_arr){
  const int leg_id = get_leg_id(setpoint_arr->ids[0]);

  Eigen::Matrix<double,6,1> qd;
  qd.setZero();

  for(int i=0; i<setpoint_arr->ids.size() ; i++){

    int state_id  = motor_order[ ( setpoint_arr->ids[i] - 1) % num_DOF_leg ];
    qd[ state_id ] = setpoint_arr->values[i] * M_PI/180.;

  }
  
  auto &context = simulator.get_mutable_context();
  input_ports[leg_id]->FixValue(&context,qd);
}
