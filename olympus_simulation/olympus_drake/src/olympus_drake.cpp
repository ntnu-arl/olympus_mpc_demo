#include "olympus_drake.hpp"

// ===== STRUCT DEFINITION ====================================================

BushingParamsStruct::BushingParamsStruct(){
  YAML::Node config = YAML::LoadFile("config/olympus_param.yaml");

  if (!config["bushing_params"]) {
      std::cerr << "[olympus drake]: 'bushing_params' key not found in the YAML file." << std::endl;
  }else{
    // Gains.Kp;
      YAML::Node torque_stiffness = config["bushing_params"]["torque_stiffness"];
      YAML::Node torque_damping   = config["bushing_params"]["torque_damping"  ];
      YAML::Node force_stiffness  = config["bushing_params"]["force_stiffness" ];
      YAML::Node force_damping    = config["bushing_params"]["force_damping"   ];

      for (int i=0; i<3; i++){
      torque_stiffness_constants[i] = torque_stiffness[i].as<double>();
      torque_damping_constants[i]   = torque_damping[i].as<double>();
      force_stiffness_constants[i]  = force_stiffness[i].as<double>();
      force_damping_constant[i]     = force_damping[i].as<double>();
      }
  }
}

SpringParamsStruct::SpringParamsStruct(){
  YAML::Node config = YAML::LoadFile("config/olympus_param.yaml");

  if (!config["spring_params"]) {
      std::cerr << "[olympus drake]: 'spring_params' key not found in the YAML file." << std::endl;
  }else{
      stiffness   = config["spring_params"]["stiffness"].as<double>();
      free_length = config["spring_params"]["free_length"].as<double>();
      damping     = config["spring_params"]["damping"].as<double>();
  }
}

controllerGains::controllerGains(){
  YAML::Node config = YAML::LoadFile("config/olympus_param.yaml");

  if (!config["controller_gains"]) {
      std::cerr << "[olympus drake]: 'controller_gains' key not found in the YAML file." << std::endl;
      std::cerr << "[olympus drake]:  Using the default PID gains" << std::endl;
  }else{

    // Populate the struct
    for (int i = 0; i < 3; ++i) {
        Kp[i] = config["controller_gains"]["Kp"][i].as<double>();
        Kd[i] = config["controller_gains"]["Kd"][i].as<double>();
        Ki[i] = config["controller_gains"]["Ki"][i].as<double>();
    }
  }
}

state_info_struct::state_info_struct(const int & base_type)
    : base_joint_type(base_type) {
  
  switch (base_type){
    case olympus_base_joint::weld:
      num_base_pos = 0;
      num_base_vel = 0;
      break;
    case olympus_base_joint::floating:
      num_base_pos = 7;
      num_base_vel = 6;
      break;
    default: //revolute x/y/z
      num_base_pos = 1;
      num_base_vel = 1;
      break;
  }
  num_leg_joints = 5;
  num_legs = 4;

}

// ===== DEMULTIPLEXER DEFINITION =============================================

olympus_state_demultiplexer::olympus_state_demultiplexer(const state_info_struct & state_struct )
    : base_joint_type(state_struct.base_joint_type),
      num_base_pos(state_struct.num_base_pos),
      num_base_vel(state_struct.num_base_vel),
      num_leg_joints(state_struct.num_leg_joints),
      num_legs(state_struct.num_legs),
      num_states(num_leg_joints*num_legs*2 +num_base_pos+num_base_vel),
      num_pos_states(state_struct.num_base_pos + state_struct.num_legs*state_struct.num_leg_joints ){

  this->DeclareVectorInputPort("full_state", drake::systems::BasicVector<double>(num_states));

  pos_ids.push_back(0);
  vel_ids.push_back(num_pos_states);

  //leg state indices:
  for (int i = 0; i<num_legs ; i++){
    int leg_id = olympus_plant::leg_sim_order[i];

    int leg_pos_start_index = num_base_pos + (leg_id * num_leg_joints);
    int leg_vel_start_index = num_base_vel + (leg_id * num_leg_joints) + num_pos_states;

    pos_ids.push_back(leg_pos_start_index);
    vel_ids.push_back(leg_vel_start_index);
  }

  auto leg_prefix = olympus_plant::leg_prefix ; 
  //1.  Declare an output port for each leg's state
  for (int i = 0; i < num_legs; ++i) {
      this->DeclareVectorOutputPort(
              leg_prefix[i]+"_leg_state",
              drake::systems::BasicVector<double>(num_leg_joints * 2),
              std::bind(&olympus_state_demultiplexer::CalcLegStateOutput, this, std::placeholders::_1, std::placeholders::_2, i));
      }
  
  //2. Declare an output port for the body orientation
  this->DeclareVectorOutputPort(
              "olympus_body_state",
              drake::systems::BasicVector<double>(13), //output full body state always
              std::bind(&olympus_state_demultiplexer::CalcBodyStateOutput, this, std::placeholders::_1, std::placeholders::_2));
}

//TODO: Orientation should be given according to the mocap setup -> revolute joints
void olympus_state_demultiplexer::CalcBodyStateOutput ( 
    const drake::systems::Context<double>& context, 
    drake::systems::BasicVector<double>* output ) const {

  const auto& full_state = this->get_input_port(0).Eval(context);
  auto output_vector = output->get_mutable_value();

  Eigen::Quaterniond quat;
  Eigen::Quaterniond quat_off;
  Eigen::Quaterniond quat_mocap;

  const double * angle;
  const double * twist;

  output_vector.setZero();
  switch (base_joint_type){
    case olympus_base_joint::weld:
      break;

    case olympus_base_joint::floating:
      output_vector.head(4)        = full_state.segment(pos_ids[0], 4);
      output_vector.segment(7, 3)  = full_state.segment(vel_ids[0], 3);
      break;

    case olympus_base_joint::revolute_x:
      angle =  &full_state[ pos_ids[0] ];
      twist =  &full_state[ vel_ids[0] ];

      quat     = Eigen::Quaterniond( Eigen::AngleAxisd( *angle, Eigen::Vector3d(1,0,0)) );
      quat_off = Eigen::Quaterniond( Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,1,0)) );
      quat_mocap = quat_off*quat ; 

      output_vector[0]            = quat_mocap.w(); 
      output_vector.segment(1, 3) = quat_mocap.vec();
      output_vector[9]            = *twist; //z-axis
      break;

    case olympus_base_joint::revolute_y:
      angle =  &full_state[ pos_ids[0] ];
      twist =  &full_state[ vel_ids[0] ];

      quat     = Eigen::Quaterniond( Eigen::AngleAxisd( *angle, Eigen::Vector3d(0,1,0)) );
      quat_off = Eigen::Quaterniond( Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)) );
      quat_mocap = quat_off*quat ; 

      output_vector[0]            = quat_mocap.w(); 
      output_vector.segment(1, 3) = quat_mocap.vec();
      output_vector[9]            = *twist; //z-axis
      break;

    case olympus_base_joint::revolute_z:
      angle =  &full_state[ pos_ids[0] ];
      twist =  &full_state[ vel_ids[0] ];

      quat = Eigen::Quaterniond( Eigen::AngleAxisd(  *angle, Eigen::Vector3d(0,0,1)) ) ;

      output_vector[0]            = quat.w(); 
      output_vector.segment(1, 3) = quat.vec();
      output_vector[9]            = *twist; //z-axis

  }
}

void olympus_state_demultiplexer::CalcLegStateOutput(
    const drake::systems::Context<double>& context, 
    drake::systems::BasicVector<double>* output, 
    const int & leg_index) const {

  const auto& full_state = this->get_input_port(0).Eval(context);
  auto output_vector = output->get_mutable_value();
  
  //index 0 in `pos_ids/vel_ids` is for body states
  const int & leg_pos_start_index = pos_ids[leg_index + 1];
  const int & leg_vel_start_index = vel_ids[leg_index + 1];

  // Copy the leg's q (positions) and v (velocities) to the output vector
  output_vector.head(num_leg_joints) = full_state.segment(leg_pos_start_index, num_leg_joints);
  output_vector.tail(num_leg_joints) = full_state.segment(leg_vel_start_index, num_leg_joints);
}

// ===== LEG SYSTEM DEFINITION ================================================

leg_system::leg_system(drake_builder & builder, drake_plant* plant_, 
                       const std::string & leg_prefix_)
    : plant(plant_),
      leg_prefix(leg_prefix_) {

  YAML::Node config = YAML::LoadFile("config/olympus_param.yaml");

  //1. Get joint names:
  std::vector<std::string> joint_names;
  if (!config["joint_names"]) {
    std::cerr << "[olympus drake]: 'joint_names' key not found in the YAML file." << std::endl;
  } else {
    if (config["joint_names"][leg_prefix]) {
      for (const auto& joint : config["joint_names"][leg_prefix]) {
          joint_names.push_back(joint.as<std::string>());
      }
    } else {
      std::cerr << "[olympus drake] Error: '" << leg_prefix << "' key not found in 'joint_names'." << std::endl;
    }
  }

  //2. Get body names:
  std::vector<std::string> body_names;
  if (!config["body_names"]) {
    std::cerr << "[olympus drake]: 'body_names' key not found in the YAML file." << std::endl;
  } else {
    if (config["body_names"][leg_prefix]) {
      for (const auto& body : config["body_names"][leg_prefix]) {
          body_names.push_back(body.as<std::string>());
      }
    } else {
      std::cerr << "[olympus drake] Error: '" << leg_prefix << "' key not found in 'body_names'." << std::endl;
    }
  }


  //3. Get joint and body object pointers from loaded urdf
  for (int i=0;i<num_joints;i++){
    leg_bodies[i] = &plant->GetBodyByName(body_names[i]);
    leg_joints[i] = &plant->GetJointByName(joint_names[i]);
  }
  set_state_ids(); //TODO: see if it is needed

  bool add_spring = true;
  //4. Add other parameters, such as springs etc.
  if (!config["add_leg_spring"]) {
      std::cerr << "[olympus drake]: 'add_leg_spring' key not found in the YAML file." << std::endl;
      std::cerr << "[olympus drake]: By default the spring is added" << std::endl;
  }else{
      add_spring = config["add_leg_spring"].as<bool>();
  }
  
  std::vector<double> motor_limit_;
  if (!config["motor_limits"]) {
      std::cerr << "[olympus drake]: 'motor_limits' key not found in the YAML file." << std::endl;
  }else{
    motor_limit_ = config["motor_limits"].as<std::vector<double>>();
    if (motor_limit_.size()!= 3){
      std::cerr << "[olympus drake]: 3 values for motor_limits must be provided. Using default limits of 20Nm" << std::endl;
    }else {
      for (int i=0; i<3;i++){ motor_limit[i] = motor_limit_[i];}
    }
      
  }
  
  //4. Complete leg system
  if (add_spring){  add_linear_spring(); } 
  add_bushing_joint();
  add_actuators();
  add_PID_system(builder);
}

void leg_system::handle_shanks_collisions(drake::geometry::SceneGraph<double> & scene_graph){
  drake::geometry::GeometrySet shanks_collision_set;

  auto &link21 = *(leg_bodies[2]);
  auto &link22 = *(leg_bodies[4]);

  auto &link21_collision = plant->GetCollisionGeometriesForBody(link21);
  auto &link22_collision = plant->GetCollisionGeometriesForBody(link22);

  shanks_collision_set.Add(link21_collision);
  shanks_collision_set.Add(link22_collision);

  // Exlude collisions between shank joints
  auto collision_manager = scene_graph.collision_filter_manager();

  collision_manager.Apply(
  drake::geometry::CollisionFilterDeclaration().
  ExcludeWithin(shanks_collision_set)
  );
}

drake::systems::InputPortIndex & leg_system::get_controller_port(){
  return controller_desired_state_port;
}

drake::systems::controllers::PidController<double>* leg_system::get_controller_ptr(){
  return controller; 
}

drake::systems::Saturation<double>* leg_system::get_controller_saturation_ptr(){
  return controller_saturation; 
}

void leg_system::set_state_ids(){
  int mh_index = int( leg_bodies[0]->index() ) ;
  int hi_index = int( leg_bodies[1]->index() ) ;
  int ho_index = int( leg_bodies[3]->index() ) ;

  //Relative state index, in the same leg:
  // state_index (si)
  int mh_si = 0;
  int hi_si = hi_index-mh_index;
  int ho_si = ho_index-mh_index;
  int mh_vel_si = mh_si + num_joints;
  int hi_vel_si = hi_si + num_joints;
  int ho_vel_si = ho_si + num_joints;

  state_ids = {mh_si,hi_si,ho_si, mh_vel_si,hi_vel_si,ho_vel_si};
}

void leg_system::add_linear_spring(){
  auto&  spring = plant->AddForceElement<drake::multibody::LinearSpringDamper>(
      *(leg_bodies[2]),
      drake::Vector3<double>::Zero(),
      *(leg_bodies[4]),
      drake::Vector3<double>::Zero(),
      SpringParams.free_length,
      SpringParams.stiffness,
      SpringParams.damping
  );
}

void leg_system::add_bushing_joint(){

  double attachment_point_link21 =  0.29977;
  double attachment_point_link22 =  0.29929;

  auto& link21_frame = leg_bodies[2]->body_frame();
  auto& link22_frame = leg_bodies[4]->body_frame();

  auto& link21_endpoint = plant->AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>
      (
          leg_bodies[2]->name()+"_ee",
          link21_frame,
          drake_tfd(drake::math::RollPitchYaw<double>(-M_PI_2,0,0),Eigen::Vector3d{0,0,-attachment_point_link21})
      ) 
  );

  //Angle offset to avoid gimball lock of bushing joint
  auto& link22_endpoint = plant->AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>
      (
          leg_bodies[4]->name()+"_ee",
          link22_frame,
          drake_tfd(drake::math::RollPitchYaw<double>(-M_PI_2,0,0),Eigen::Vector3d{0,0,-attachment_point_link22})  
      ) 
  );

  auto&  bushing = plant->AddForceElement<drake::multibody::LinearBushingRollPitchYaw>(
      link21_endpoint,
      link22_endpoint,
      BushingParams.torque_stiffness_constants,
      BushingParams.torque_damping_constants,
      BushingParams.force_stiffness_constants,
      BushingParams.force_damping_constant
  );

}

void leg_system::add_actuators(){
  auto& motorMH = plant->AddJointActuator(leg_prefix+"motor_MH"   ,*( leg_joints[0] ),motor_limit[0]);
  auto& motor1  = plant->AddJointActuator(leg_prefix+"motor_inner",*( leg_joints[1] ),motor_limit[1]);
  auto& motor2  = plant->AddJointActuator(leg_prefix+"motor_outer",*( leg_joints[3] ),motor_limit[2]);
}

void leg_system::add_PID_system(drake_builder & builder){
  //1. Define Control Projection Matrix:
  //Given estimated state x_in = (q_in, v_in), the controlled state x_c = (q_c, v_c) is computed by x_c = P_x * x_in
  int num_states = num_joints*2; 

  //2. Populate it
  Eigen::MatrixXd ControlProjectionMatrix;
  ControlProjectionMatrix.resize(6,num_states);
  ControlProjectionMatrix.setZero();
  for ( int i_s=0;i_s<6;i_s++){
    ControlProjectionMatrix(i_s,state_ids[i_s]) = 1;
  }

  //3. Add PID system
  controller = builder.AddNamedSystem<drake::systems::controllers::PidController<double>>(
      leg_prefix+"_controller",
      ControlProjectionMatrix,
      Gains.Kp,
      Gains.Ki,
      Gains.Kd);

  //Motor limits must be enforced to PID system independently, as actuator limits are efnorces only if there are joint pd.
  Eigen::Vector3d input_limit(motor_limit[0],motor_limit[1],motor_limit[2]);
  controller_saturation = builder.AddSystem<drake::systems::Saturation<double>>(-input_limit,input_limit);
  builder.Connect( controller -> get_output_port(), controller_saturation -> get_input_port() );
    

  controller_desired_state_port = builder.ExportInput(controller -> get_input_port_desired_state(),leg_prefix+"_leg_setpoint");
}

// ===== OLYMPUS PLANT  DEFINITION ============================================

olympus_plant::olympus_plant(drake_builder & builder) {

  //A. Read custom simulation - robot parameters:  
  #pragma region
  YAML::Node sim_config = YAML::LoadFile("config/simulation_param.yaml"); 

  //Initialization of simulation parameters:
  Eigen::Vector3d custom_gravity{0,0,0};

  //Populate them
  if (!sim_config["gravity"]) {
      std::cerr << "[olympus drake]: 'gravity' key not found in the YAML file. Using zero gravity" << std::endl;
  }else{
    auto gravity_from_yaml = sim_config["gravity"].as<std::vector<double>>();
    for (int i=0; i<3; i++){       
      custom_gravity[i] = gravity_from_yaml[i];
    }
  }

  if (!sim_config["time_step"]) {
      std::cerr << "[olympus drake]: 'time_step' key not found in the YAML file. Using a time_step value of "<< time_step << std::endl;
  }else{
    time_step = sim_config["time_step"].as<double>();
  }

  if (!sim_config["sim_update_rate"]) {
      std::cerr << "[olympus drake]: 'sim_update_rate' key not found in the YAML file. Using a simulation_update_rate value of "<< simulation_update_rate << std::endl;
  }else{
    simulation_update_rate = sim_config["sim_update_rate"].as<double>();
  }

  if (!sim_config["realtime_rate"]) {
      std::cerr << "[olympus drake]: 'realtime_rate' key not found in the YAML file. Using a realtime_rate_target value of "<< realtime_rate_target << std::endl;
  }else{
    realtime_rate_target = sim_config["realtime_rate"].as<double>();
  }

  if (!sim_config["publish_every_time_step"]) {
      std::cerr << "[olympus drake]: 'publish_every_time_step' key not found in the YAML file. Using a publish_every_time_step_ value of "<< publish_every_time_step_ << std::endl;
  }else{
    publish_every_time_step_ = sim_config["publish_every_time_step"].as<bool>();
  }

  YAML::Node robot_config = YAML::LoadFile("config/olympus_param.yaml");

  //Initialization of robot parameters:
  std::string robot_name = "olympus";
  std::string torso_name = "Body";

  if (!robot_config["robot_name"]) {
      std::cerr << "[olympus drake]: 'robot_name' key not found in the YAML file. Using "<< robot_name << std::endl;
  }else{
    robot_name = robot_config["robot_name"].as<std::string>();
  }

  if (!robot_config["base_joint_type"]) {
      std::cerr << "[olympus drake]: 'base_joint_type' key not found in the YAML file. Using \'weld\' joint as default option" << std::endl;
  }else{
    joint_type = robot_config["base_joint_type"].as<int>(); 

    //Testbed displacement exist only in revolute joints, where the experimentat setup is in drake.
    if (!robot_config["testbed_params"]) {
      std::cerr << "[olympus drake]: 'testbed_params' key not found in the YAML file. Using zero values as offset and testbed damping" << std::endl;
    }else{
      switch(joint_type){
        case olympus_base_joint::revolute_x:
          testbed_DZ = robot_config["testbed_params"]["roll_DX"].as<double>(); 
          testbed_damping = robot_config["testbed_params"]["damping_roll"].as<double>(); 
          break;
        case olympus_base_joint::revolute_y:
          testbed_DZ = robot_config["testbed_params"]["pitch_DX"].as<double>(); 
          testbed_damping = robot_config["testbed_params"]["damping_pitch"].as<double>(); 
          break;
        case olympus_base_joint::revolute_z:
          testbed_DZ = robot_config["testbed_params"]["yaw_DX"].as<double>(); 
          testbed_damping = robot_config["testbed_params"]["damping_yaw"].as<double>(); 
          break;
      }
    }
  }

  if (!robot_config["torso_name"]) {
      std::cerr << "[olympus drake]: 'torso_name' key not found in the YAML file. Using "<< torso_name << std::endl;
  }else{
    torso_name = robot_config["torso_name"].as<std::string>();
  }

  std::vector<std::string> leg_prefix_;
  for (auto leg:robot_config["joint_names"]){
    leg_prefix_.push_back(  leg.first.as<std::string>() ); 
  }
  if (leg_prefix_.size()==4){
    std::copy(leg_prefix_.begin(),leg_prefix_.end(),leg_prefix.begin());
  }else{
    std::cerr << "[olympus drake]: More joint groupds were loaded than expected. These are: "<<std::endl;
    for (auto str:leg_prefix_){ std::cerr << str <<std::endl;}
  }

  #pragma endregion

  //B. add systems
  #pragma region
  int actuators_per_leg = 3;
  int num_legs = 4;
  int num_actuators = num_legs*actuators_per_leg;
  //1. Add quadruped plant
  auto [plant_,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,time_step);
  plant = &plant_;

  drake::multibody::Parser parser(plant);
  quadruped = parser.AddModels("urdf/olympus_drake.urdf").at(0);
  plant->RenameModelInstance(quadruped,robot_name);
  get_joint_mapping(plant->GetJointIndices(quadruped),robot_config);

  add_base_joint( plant->GetBodyByName(torso_name) );

  for (int i=0;i<num_legs;i++){
    legs[i] = new leg_system(builder, plant, leg_prefix[i]);
    legs[i]->handle_shanks_collisions(scene_graph);
  }
  
  //2. Set plant simulation parameters
  plant->mutable_gravity_field().set_gravity_vector(custom_gravity);
  if (joint_type == olympus_base_joint::floating){
    std::cout << "[drake simulation]: With floating joint, gravity is always set to zero" <<std::endl;
    plant->mutable_gravity_field().set_gravity_vector(Eigen::Vector3d().Zero());
  }
  plant->set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
  plant->Finalize();

  //3. Add rest of systems in the simulation (PID, Multiplexer, "Demultiplexer")
  state_info_struct state_info(joint_type);

  auto mux   = builder.AddSystem<drake::systems::Multiplexer>(std::vector<int>(num_legs, actuators_per_leg));
  auto demux   = builder.AddSystem<drake::systems::Demultiplexer>(num_actuators,actuators_per_leg);
  outputer   = builder.AddSystem<olympus_state_demultiplexer>(state_info);
  outputer->set_name("olympus_state_demultiplexer");

  #pragma endregion

  //C. connect systems
  builder.Connect(mux->get_output_port(),plant->get_actuation_input_port());
  builder.Connect(plant->get_state_output_port(),outputer->get_input_port());
  builder.Connect(plant->get_net_actuation_output_port(quadruped),demux->get_input_port());

  //Add "encoders"
  for (int i=0;i<num_legs;i++){
    builder.ExportOutput(outputer->get_output_port(i));
    builder.Connect(legs[i]->get_controller_saturation_ptr()->get_output_port(),mux->get_input_port(i));
    // builder.Connect(plant->get_state_output_port(quadruped)      ,legs[i]->get_controller_ptr()->get_input_port_estimated_state());
    builder.Connect(outputer->get_output_port(i)      ,legs[i]->get_controller_ptr()->get_input_port_estimated_state());

  }
  builder.ExportOutput(outputer->get_output_port(4)); //Body state output
  //Add actuation ports afterwards
  for (int i=0;i<num_legs;i++){      
    builder.ExportOutput(demux->get_output_port(i),leg_prefix[i]+"_actuation_port"); 
  }

  //Debugging:
  // for (auto str: plant->GetStateNames() ){
  //   std::cout<< str <<std::endl;
  // } 

}

drake_plant &  olympus_plant::get_plant(){
  return *plant;
}

double  olympus_plant::get_timestep() const{
  return time_step;
} 

double  olympus_plant::get_simulation_update_rate() const {
    return simulation_update_rate;
}

double  olympus_plant::get_realtime_rate_target() const {
    return realtime_rate_target;
}

bool  olympus_plant::publish_every_time_step() const {
    return publish_every_time_step_;
}

leg_system&  olympus_plant::get_leg(const int & index){
  return *(legs[index]); 
}

olympus_state_demultiplexer&  olympus_plant::get_state_demux(){
  return *outputer;
}

void olympus_plant::add_base_joint( const drake_rigidBody & torso_body ){

  const drake::multibody::Joint< double >* world_joint_ptr =nullptr; 
  const drake_rigidBody*  testbed_body_ptr =nullptr; 

  switch (joint_type){
    case olympus_base_joint::revolute_x :
      testbed_body_ptr = add_testbed_body();

      world_joint_ptr = & plant->AddJoint<drake::multibody::RevoluteJoint>(
        "world_joint",
        plant->world_body(),
        drake_tfd(Eigen::Translation3d{0,0,1}),
        *testbed_body_ptr,
        drake_tfd(),
        Eigen::Vector3d{0,0,1}, // z-axis
        testbed_damping
      );

      joint_ptr = & plant->AddJoint< drake::multibody::WeldJoint >(
        "body_joint",
        *testbed_body_ptr,
        drake_tfd(drake::math::RollPitchYawd(0,-M_PI_2,0), drake::Vector3<double>(testbed_DZ,0,0)), //z-translation
        torso_body,
        drake_tfd(),
        drake_tfd::Identity()
      ); 

      break;

    case olympus_base_joint::revolute_y :
      testbed_body_ptr = add_testbed_body();

      world_joint_ptr = & plant->AddJoint<drake::multibody::RevoluteJoint>(
        "world_joint",
        plant->world_body(),
        drake_tfd(Eigen::Translation3d{0,0,1}),
        *testbed_body_ptr,
        drake_tfd(),
        Eigen::Vector3d{0,0,1}, // z-axis
        testbed_damping
      );

      joint_ptr = & plant->AddJoint< drake::multibody::WeldJoint >(
        "body_joint",
        *testbed_body_ptr,
        drake_tfd(drake::math::RollPitchYawd(-M_PI_2,0,0), drake::Vector3<double>(0,testbed_DZ,0)), //z-translation
        torso_body,
        drake_tfd(),
        drake_tfd::Identity()
      ); 
      break;

    case olympus_base_joint::revolute_z :
      testbed_body_ptr = add_testbed_body();

      world_joint_ptr = & plant->AddJoint<drake::multibody::RevoluteJoint>(
        "world_joint",
        plant->world_body(),
        drake_tfd(Eigen::Translation3d{0,0,1}),
        *testbed_body_ptr,
        drake_tfd(),
        Eigen::Vector3d{0,0,1}, // z-axis
        testbed_damping
      );

      joint_ptr = & plant->AddJoint< drake::multibody::WeldJoint >(
        "body_joint",
        *testbed_body_ptr,
        drake_tfd( Eigen::Translation3d{0,0,testbed_DZ}), //z-translation
        torso_body,
        drake_tfd(),
        drake_tfd::Identity()
      );
      break;

    case olympus_base_joint::floating :
      joint_ptr = & plant->AddJoint<drake::multibody::QuaternionFloatingJoint>(
        "body_joint",
        plant->world_body(),
        drake_tfd( Eigen::Translation3d{0,0,1}), //z-translation
        torso_body,
        drake_tfd() // x-axis
      );
      break;
    
    default: //weld
      joint_ptr = & plant->AddJoint< drake::multibody::WeldJoint >(
      "body_joint",
      plant->world_body(),
      drake_tfd( Eigen::Translation3d{0,0,1}), //z-translation
      torso_body,
      drake_tfd(),
      drake_tfd::Identity());
  }

}

const drake_rigidBody*  olympus_plant::add_testbed_body(){
  drake::multibody::Parser parser(plant);
  auto base = parser.AddModels("urdf/testbed.urdf").at(0);
  plant->RenameModelInstance(base,"testbed");

  return &plant->GetBodyByName("testbed");

}

void olympus_plant::get_joint_mapping(
    const std::vector<drake::multibody::JointIndex> & joint_index_vector,
    const YAML::Node & robot_config){
    
  //1.  Get a joint(first) of each leg system and add it in the `elements` vector
  std::vector<std::string> joint_elements; 
  for (auto str:leg_prefix){
    joint_elements.push_back(robot_config["joint_names"][str].as<std::vector<std::string>>()[0] );
  }

  //2. Get the `joint_index` of the first joint of each leg 
  std::vector<int> joint_elements_ids;
  for( auto joint_elem:joint_elements){
    auto& joint = plant->GetJointByName(joint_elem,quadruped);

    // std::cout<< int( joint.index()   ) <<std::endl;
    // std::cout<< joint_elem             <<std::endl;

    joint_elements_ids.push_back( int( joint.index()   ) );
  }

  //3. Map urdf leg system order with configuration order
  auto ascending_ind = ascending_indices<int>(joint_elements_ids);

  //4. Map configuration order with urdf order (to get states). (Its like inversing)
  auto map_ind_ = ascending_indices<int>(ascending_ind);
  std::copy(map_ind_.begin(),map_ind_.end(),leg_sim_order.begin());
}

std::array<std::string,4> olympus_plant::leg_prefix{"FrontLeft","FrontRight","BackLeft","BackRight"}; 
std::array<int,4> olympus_plant::leg_sim_order{0,1,2,3}; 


