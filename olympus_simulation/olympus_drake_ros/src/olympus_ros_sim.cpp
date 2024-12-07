#include <ros/ros.h>
#include "olympus_drake_ros.hpp"


int main(int argc, char** argv){
  // Initialize the ROS node
  ros::init(argc, argv, "olympus_drake_simulation");

  // Create a ROS node handle
  ros::NodeHandle nh;
  // olympus_plant::leg_prefix ; 

  // Your code heres

  // Spin the ROS node

    #pragma region
    double time_step = 0.001;
    drake::systems::DiagramBuilder<double> builder; // Class responsible from creating the dynamic system.

    olympus_plant olympus(builder);
    // drake_plant & plant = olympus.get_plant();

    meshcat_shared_ptr meshcat_ptr =  std::make_shared<drake::geometry::Meshcat> (); //pointer to visualization class
    drake::visualization::AddDefaultVisualization(&builder,meshcat_ptr); //add visualization

    auto diagram = builder.Build(); //Connections before here
    get_system_graph(diagram.get()); //For Debugging purposes
    #pragma endregion

    //2. Set initial conditions:
    #pragma region
    drake::systems::Simulator sim(*diagram);
    auto& context = sim.get_mutable_context();

   Eigen::VectorXd qd(6); 
   qd.setZero();
   for(int il=0; il< NUM_LEGS ; il ++){
    diagram->get_input_port(olympus.get_leg(il).get_controller_port()).FixValue(&context,qd);
   }

    #pragma endregion
    
    drake_ros_elements interface_elements(sim,olympus.get_base_joint_type());
    int num_state_ports = 5; //4 legs + body;
    for (int i=0;i<4;i++){
      interface_elements.leg_input_ports.push_back( &diagram->get_input_port(i) ); 
      interface_elements.leg_output_ports.push_back( &diagram->get_output_port(i) ); 
      interface_elements.leg_actuation_output_ports.push_back( &diagram->get_output_port(i+num_state_ports) ); 
    }
    interface_elements.body_output_port = &diagram->get_output_port(4);
    olympus_ros_interface interface(interface_elements) ;
    
    
    //3. Simulation Parameters:
    #pragma region
    sim.set_publish_every_time_step(olympus.publish_every_time_step()); // publish simulation as it happens
    sim.set_target_realtime_rate(olympus.get_realtime_rate_target());
    sim.Initialize();
    double sim_time = 0; 
    double sim_update_rate =olympus.get_simulation_update_rate(); 
    assert(sim_update_rate >= olympus.get_timestep());
    
    while( ros::ok()){
      //realtime vis
      sim_time +=sim_update_rate;
      sim.AdvanceTo(sim_time);   
      interface.publish(); 
      ros::spinOnce();
    }

    #pragma endregion


  return 0;
}