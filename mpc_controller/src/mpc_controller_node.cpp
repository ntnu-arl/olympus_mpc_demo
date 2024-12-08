#include <ros/ros.h>
#include "leg_kinematics.hpp"
#include <eigen3/Eigen/Dense>

//message_types
#include "olympus_ros_msgs/SetpointArray.h"
#include "sensor_msgs/JointState.h"

#define corresponding_kinematic_configuration(leg_id) ( (i==0 || i==3)? 1: 2 ) 
#define traj_stages 4

#include <stdlib.h>
// acados
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "c_generated_code/acados_solver_fr_leg_pos.h" 

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

//OCP defines:
#define NX     FR_LEG_POS_NX 
#define NU     FR_LEG_POS_NU 
#define NBX0   FR_LEG_POS_NBX0 

#define TH    0.3

//This are read from a config in WBC class
struct leg_controller_opt{
  std::array<int,5> joint_state_id{0,1,2,3,4};
  std::array<int,3> motor_id{6,5,4};
  int kinematic_config = 1; //[1,2]
  int leg_id = 0; 
};

class leg_controller {
  public:		
  leg_controller(const leg_controller_opt & opts):
  joint_state_id(opts.joint_state_id),
  kinematics(opts.kinematic_config),
  id(opts.leg_id),
  setpoint_arr(init_setpoint_array(opts.motor_id)),
  qd(setpoint_arr.values.data()) 
  {
      ros::NodeHandle nh;
      std::string leg_node_name = "leg"+ std::to_string(id+1)+"_node/";
      command_pub     = nh.advertise<olympus_ros_msgs::SetpointArray>(leg_node_name+"command_position",10);
      motor_state_sub = nh.subscribe("/joint_states",10,&leg_controller::status_callback,this);

      full_state.setZero();
      ros::spinOnce(); //Populate message with current state
      qd[0] = q[0];
      qd[1] = q[1];
      qd[2] = q[3];

      //Debugging:
      std::cout << "Current id is: " << id << std::endl;
      for (auto id:joint_state_id){std::cout << id << std::endl;}
      std::cout <<"Kinematic config is:" << opts.kinematic_config << std::endl;
      std::cout << "====" << std::endl;
      for (auto id:setpoint_arr.ids){std::cout << int(id) << std::endl;}

  }

  void setCommand(const Eigen::Vector3f & qd_){
      qd = qd_;
      Publish();
  }

  void setCommand(const std::vector<float> & qd_vec){
      if (qd_vec.size()!=3){
        ROS_ERROR_STREAM("[mpc_controller]: wrong vector size. Abort command");
      }else{
        setpoint_arr.values = qd_vec;
        Publish();
      }
  }

  Eigen::Matrix<double,5,1> leg_state_estimation(const Eigen::Vector3d & qd){
    return kinematics.state_estimation(qd);
  }

  Eigen::Matrix<double,10,1>  get_full_state() const {
    return full_state; 
  }

  const Eigen::Matrix<double,10,1> *  get_full_state_ptr() const {
    return &full_state; 
  }

  private:

  void Publish(){
    setpoint_arr.header.stamp = ros::Time::now();
    command_pub.publish(setpoint_arr);
  }

  void status_callback(const sensor_msgs::JointStateConstPtr & joint_state ){
    for (int i=0;i<5;i++){
      q[i]  = joint_state->position[ joint_state_id[i] ];
      qt[i] = joint_state->velocity[ joint_state_id[i] ];
    }

  //Debuggin:
  // ROS_INFO_STREAM("Current state for FR: "<< full_state );

  }

  //This function is used to initialize `setpoint_arr` in the initializer list.
  //
  // --- 
  //
  // - It is needed so qd which is `Eigen::Map<Eigen::Vector3f>` can share the same memory with `setpoint_arr_.values`
  olympus_ros_msgs::SetpointArray init_setpoint_array(const std::array<int,3> & motor_id) const {

    olympus_ros_msgs::SetpointArray setpoint_arr_;
    for (int id:motor_id){
      setpoint_arr_.ids.push_back(id);
      setpoint_arr_.values.push_back(0);
    }

    return setpoint_arr_; 

  }

  const int id; 
  const std::array<int,5> joint_state_id; 
  leg_kinematics kinematics;

  ros::Publisher command_pub;
  ros::Subscriber motor_state_sub;
  olympus_ros_msgs::SetpointArray setpoint_arr;

  Eigen::Matrix<double,10,1> full_state;
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  q = full_state.head<5>() ;  //alias for position part
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  qt = full_state.tail<5>(); //alias for velocity part

  ///Desired joint position.
  ///@note Map takes a pointer to the data container to construct. Size specified by using Vector3f.
  ///This way efficiently uses eigen to interface with controller and the `setpoint_arr.position` vector to publish the msg
  Eigen::Map<Eigen::Vector3f> qd; 

};

class whole_body_controller {
  public:		
    whole_body_controller() {
      ros::NodeHandle nh;
      ros::NodeHandle nh_param("~");

      //1. get parameters:
      std::string param_name;

      std::array<std::string,4> leg_prefix{ "fr","rr","fl","rl"};
      std::array< std::vector<int>,4 > joint_state_ids;
      std::array< std::vector<int>,4 > motor_ids;

      for (int i=0; i<4; i++){
        param_name = leg_prefix[i]+"_joint_state_ids";
        if ( !nh_param.getParam(param_name,joint_state_ids[i]) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }

        param_name = leg_prefix[i]+"_motor_ids";
        if ( !nh_param.getParam(param_name,motor_ids[i]) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }
      }

      //TODO: leg_order from config
      std::array<int,4> leg_order{1,3,0,2};
      //2. create corresponding leg controllers
      for (int i =0; i<4; i++){
        leg_controller_opt leg_opts;
        leg_opts.leg_id = leg_order[i];
        leg_opts.kinematic_config = corresponding_kinematic_configuration(i) ;

        std::copy(motor_ids[i].begin(),motor_ids[i].end(),leg_opts.motor_id.begin());
        std::copy(joint_state_ids[i].begin(),joint_state_ids[i].end(),leg_opts.joint_state_id.begin());

        leg_controllers[i] = std::make_unique<leg_controller>(leg_opts);
      }

      //3. trajectory timer
      trajectory_timer = nh.createTimer(5,&whole_body_controller::trajectory_publish,this);
      trajectory_timer.setPeriod( ros::Duration(TH/FR_LEG_POS_N) );
      trajectory_timer.stop();
      // trajectory_timer.start();
      
      xtraj.fill(0);
      utraj.fill(0);
      dt_traj.fill( TH/FR_LEG_POS_N );

      //MPC initialization:
      mpc_solver_init();
    }

  private:
  //MPC specific:
  void mpc_solver_init(){
    acados_ocp_capsule = fr_leg_pos_acados_create_capsule();
    double* new_time_steps = NULL;
    int status = fr_leg_pos_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("fr_leg_pos_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    nlp_config = fr_leg_pos_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = fr_leg_pos_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = fr_leg_pos_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = fr_leg_pos_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = fr_leg_pos_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = fr_leg_pos_acados_get_nlp_opts(acados_ocp_capsule);

    //constraint ids:
    for (int id=0; id< NBX0 ; id++){ idxbx[id]=id; }
  }

  void mpc_update_constraints(double *x0, double * xf){
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", xf);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", xf);
  }

  void initialize_mpc(Eigen::Matrix<double,10,1> & x0, Eigen::Matrix<double,10,1> & xf ){
    Eigen::Vector3d u0;
    u0.setZero();

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0.data());
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0.data());
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0.data());
  }

  public:
  void mpc_update_ocp(const Eigen::Vector3d & qdesired){
    auto qd = leg_controllers[0]->leg_state_estimation(qdesired);

    //TODO: fix kinematics to use setpoint based on command:
    // qd <<  0,1.8000,-1.1703,-0.7000,0.4087;
    qd << 0.7854,    0.3491,   -0.5528,   0.3491,    0.5590;

    Eigen::Matrix<double,10,1> xd ;
    xd.setZero();
    xd.head(5) = qd;

    ROS_INFO_STREAM("setpoint is:" <<qd); //devugging:

    Eigen::Matrix<double,10,1> x0 ;
    x0 = leg_controllers[0]->get_full_state();
    x0[3] *=-1;

    mpc_update_constraints(x0.data(),xd.data());
    initialize_mpc(x0,xd);

    //update config
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &mpc_counter);

    //Solve: 
    int status = fr_leg_pos_acados_solve(acados_ocp_capsule);

    double elapsed_time;
    //Get data:
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
    elapsed_time = MIN(elapsed_time, 1e12);

    //get solution
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    ROS_INFO("[mpc_controller]: MPC exited with status:  %d. Elapsed time: %.1f [ms]",status,1e3*elapsed_time);
    trajectory_indexer = 0;
    trajectory_timer.start();

  }

  private:

  //Controller support functions:
  void allocation(const Eigen::Vector3f & qd_fr){
    //initialize
    Eigen::Vector3f qd_rr;
    Eigen::Vector3f qd_fl;
    Eigen::Vector3f qd_rl;

    //matlab roll
    if (cancel_roll){ 
      qd_fl[0] = -qd_fr[0];
    } else { 
      qd_fl[0] = qd_fr[0] - 45;
    }

    //pitch-yaw
    if (pitch_mode){
      qd_fl[1] =  qd_fr[1];
      qd_fl[2] =  qd_fr[2];
    }else{ //yaw mode
      qd_fl[1] = -qd_fr[2]-40;
      qd_fl[2] = -(qd_fr[1]+40);
    }

    //Same side mimic:
    qd_rr[0] = -qd_fr[0]; //ok
    qd_rr[1] =  qd_fr[2];
    qd_rr[2] =  qd_fr[1];

    qd_rl[0] = -qd_fl[0]; //ok
    qd_rl[1] = -qd_fl[2];
    qd_rl[2] = -qd_fl[1];

    //update signs:
    qd_fl[1] *= -1;
    qd_fl[2] *= -1;


    std::array<const Eigen::Vector3f *, 4> qd{&qd_fr,&qd_rr,&qd_fl,&qd_rl};
    for (int i=0; i<4; i++){ 
      leg_controllers[i] -> setCommand( *qd[i] ); 
    }

  }

  void trajectory_publish(const ros::TimerEvent&){
    int & i = trajectory_indexer ;
    i++;

    // Eigen::Vector3f pos(qMH_t[i],qHI_t[i],qHO_t[i]);
    double & qMH = xtraj[i*NX    ];
    double & qHI = xtraj[i*NX +1 ];
    double & qHO = xtraj[i*NX +3 ];

    Eigen::Vector3f pos(qMH,qHI,qHO);

    pos[2] *= -1;

    allocation(pos*180/M_PI);
    trajectory_timer.setPeriod( ros::Duration( dt_traj[i-1]) );
    
    if (i == FR_LEG_POS_N){
      i=0;
      trajectory_timer.stop();
    }
  }

  bool cancel_roll = true;
  bool pitch_mode  = true;

  ros::Timer trajectory_timer;
  std::array< std::unique_ptr< leg_controller>,4> leg_controllers;

  //Saved trajectory:
  int trajectory_indexer = 0;

  std::array<double,NX*(FR_LEG_POS_N+1)> xtraj;
  std::array<double,NU* FR_LEG_POS_N   > utraj;
  std::array<double,NX* FR_LEG_POS_N   > dt_traj;

  //MPC specific:
  fr_leg_pos_solver_capsule *acados_ocp_capsule;
  ocp_nlp_config *nlp_config ;
  ocp_nlp_dims *nlp_dims ;
  ocp_nlp_in *nlp_in ;
  ocp_nlp_out *nlp_out ;
  ocp_nlp_solver *nlp_solver ;
  void *nlp_opts ;

  std::array<int,NBX0> idxbx;
  int N = FR_LEG_POS_N; //Number of shooting intervals

  //stats:
  int mpc_counter = 0;

};

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;

double loop_freq = 100 ;
ros::Rate loop_rate(loop_freq); //HZ
#pragma endregion

whole_body_controller wbc;

Eigen::Vector3d qd(0,1.8000,-0.7000);
auto timer  = nh.createTimer(ros::Duration(5),
[&wbc, qd](const ros::TimerEvent&) { wbc.mpc_update_ocp(qd); },true,true);


while(ros::ok()){
  ros::spinOnce(); // Process any callbacks
  loop_rate.sleep();
}
return 0;
}