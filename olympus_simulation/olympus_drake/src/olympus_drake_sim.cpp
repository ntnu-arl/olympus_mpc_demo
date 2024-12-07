#include "olympus_drake.hpp"

int main() {
    //setup the system to be simulated
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

    

    Eigen::Vector3d des_angles{1,1,1}; 
    Eigen::VectorXd qd(6); 
    qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
    diagram->get_input_port(olympus.get_leg(0).get_controller_port()).FixValue(&context,qd);
    diagram->get_input_port(olympus.get_leg(1).get_controller_port()).FixValue(&context,qd);
    diagram->get_input_port(olympus.get_leg(2).get_controller_port()).FixValue(&context,qd);
    diagram->get_input_port(olympus.get_leg(3).get_controller_port()).FixValue(&context,qd);

    #pragma endregion
    
    //3. Simulation Parameters:
    #pragma region
    sim.set_publish_every_time_step(olympus.publish_every_time_step()); // publish simulation as it happens
    sim.set_target_realtime_rate(olympus.get_realtime_rate_target());
    sim.Initialize();
    double sim_time = 0; 
    double sim_update_rate = olympus.get_simulation_update_rate(); 
    assert(sim_update_rate >= olympus.get_timestep());

    meshcat_ptr->StartRecording();
    try { 
    while( sim_time < 2.5){
      //realtime vis
      sim_time +=sim_update_rate;
      sim.AdvanceTo(sim_time);  
    }
    }
    catch( const std::runtime_error & e ){
      //In case gimbal lock happens, to debug it.
      // meshcat_ptr->PublishRecording();
      // std::cin.get();
    }
    meshcat_ptr->PublishRecording();
    std::cin.get(); //get time to see the recording

    //Best way to export read states:
    auto& context_sim            = sim.get_mutable_context();
    std::cout << diagram->get_output_port(0).Eval(context_sim) <<std::endl; 
    std::cout << diagram->get_output_port(1).Eval(context_sim) <<std::endl; 
    std::cout << diagram->get_output_port(2).Eval(context_sim) <<std::endl; 
    std::cout << diagram->get_output_port(3).Eval(context_sim) <<std::endl; 

    #pragma endregion

    return 0;
}