//Drake pre-reqs:
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h" 

//Drake Multibody
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h> // Add spring and bushing
#include <drake/multibody/tree/linear_spring_damper.h>
#include <drake/math/rigid_transform.h> //to set up helper frames 
#include <drake/systems/primitives/saturation.h>

#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"

//diagram design:
#include "drake/systems/controllers/pid_controller.h"//Pid Controller
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/framework/leaf_system.h>

//other helpers:
#include <functional> //std::bind
#include <algorithm> //std::sort
#include <iostream>   
#include "drake_helpers.hpp"
#include <yaml-cpp/yaml.h>

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 


/*!
 * @brief A structure to hold the dynamic parameters of the bushing joint
 * used to close the kinematic chain of the legs. 
 * 
 * For more info see [LinearBushingRollPitchYaw](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_bushing_roll_pitch_yaw.html)
 */
struct BushingParamsStruct {
  drake::Vector3<double> torque_stiffness_constants{100,0,100};  /*!<  Torque stiffness. */
  drake::Vector3<double> torque_damping_constants{20,0,20};      /*!<  Torque damping.  */
  drake::Vector3<double> force_stiffness_constants{100,0,100};   /*!<  Force stiffness. */
  drake::Vector3<double> force_damping_constant{20,0,20};        /*!<  Force damping. */

  /*!
   * @brief Constructor. 
   */
  BushingParamsStruct();
};

/*!
 * @brief A structure to hold the dynamic parameters of the knee spring. 
 * 
 * For more info see [LinearSpringDamper](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_spring_damper.html)
 */
struct SpringParamsStruct {
  double free_length = 0.175;    /*!<  Free length of linear spring. */
  double stiffness   = 1;        /*!<  Spring stiffness. */
  double damping     = 10;       /*!<  Spring Damping. */

  /*!
   * @brief Constructor. 
   */
  SpringParamsStruct();

};

/*!
 * @brief A structure to hold the PID controller gains for olympus.
 * 
 * They are ordered as [MotorHousing, HipInner, HipOuter]
 */
struct controllerGains {
  Eigen::Vector3d Kp = {10,10,10}; /*!<  Proportional gain for MH,HI,HO */
  Eigen::Vector3d Kd = {1,1,1};    /*!<  Derivative gain for MH,HI,HO   */
  Eigen::Vector3d Ki = {1,1,1};    /*!<  Integral gain for MH,HI,HO     */

  /*!
   * @brief Constructor. 
   */
  controllerGains();
  
};

/*!
 * Enum representing the types of base joints for the Olympus robot.
 */
enum olympus_base_joint {
    weld,        /*!< A fixed joint with no degrees of freedom. */
    revolute_x,  /*!< A revolute joint around the x-axis. State is [q,dq/dt] */
    revolute_y,  /*!< A revolute joint around the y-axis. State is [q,dq/dt] */
    revolute_z,  /*!< A revolute joint around the z-axis. State is [q,dq/dt] */

    /*!
     * @brief A floating joint with 6 degrees of freedom. 
     * 
     * State is [orientation, pos, angular velocity, linear velocity].
     * Orientation uses quaternions, thus there are:
     * 7 generalized positions, 6 generalized velocities.
     * See [this](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_quaternion_floating_joint.html)
     */
    floating     
};

/*!
 * @brief A structure to hold state information for the Olympus robot in order
 * to create @ref olympus_state_demultiplexer class. 
 * 
 * This structure contains information about the base joint type, the number
 * of base positions and velocities, the number of leg joints, and the number
 * of legs. 
 */
struct state_info_struct {

  /*!
   * @brief The type of the base joint.
   *
   * This can be one of the values defined in @ref olympus_base_joint.
   * Default is `olympus_base_joint::weld`.
   */
  int base_joint_type = olympus_base_joint::weld; 

  //! Number of generalized position states for the base
  int num_base_pos = 0;

  //! Number of generalized velocity states for the base
  int num_base_vel = 0;

  //! Number of joints in one leg
  int num_leg_joints = 5;

  //! Number of legs
  int num_legs = 4; 

  /*!
   * @brief Constructor. 
   * 
   * @param[in] base_type The type of the base joint.
   */
  state_info_struct(const int & base_type);
};

/*!
 * @brief This class implements a "smart demultiplexer" that gets the quadrupeds 
 * full state and exports 5 different ports, for the state of the base and each 
 * leg. 
 */
class olympus_state_demultiplexer : public drake::systems::LeafSystem<double> {
  public:

    /*!
    * @brief Constructor.
    * @param [in]  state_struct Struct to help identify leg and body states. @see state_info_struct
    */  
    olympus_state_demultiplexer(const state_info_struct & state_struct );
  
  private:

    /*!
    * @brief Assigns the body related states to the "olympus_body_state" output 
    * port depending on the base joint. 
    * 
    * The output is a vector of size 13 no matter the base joint. 
    * Output = [quat(qw qx qy qz), pos, ang_vel, lin_vel]
    * The function handles different types of base joints from @ref olympus_base_joint
    * 
    * @param[in] context The context of the system. It is passed internally
    * @param[out] output The output vector where the calculated body state values will be stored.
    * 
    * @see [DeclareVectorOutputPort](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_leaf_system.html#a11479a5cb19f65f35f1a7f5530fb42a0)
    */
    void CalcBodyStateOutput(
      const drake::systems::Context<double>& context, 
      drake::systems::BasicVector<double>* output) const ;

    /*!
    * @brief Assigns the leg related states to the "<leg-prefix>_leg_state" output port
    * 
    * @param[in] context The context of the system. It is passed internally
    * @param[out] output The output vector where the calculated body state values will be stored.
    * @param[in] leg_index The index of the leg
    * 
    * @see [DeclareVectorOutputPort](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_leaf_system.html#a11479a5cb19f65f35f1a7f5530fb42a0)
    */
    void CalcLegStateOutput(
      const drake::systems::Context<double>& context, 
      drake::systems::BasicVector<double>* output, 
      const int & leg_index) const ;
    
    /*!
    * @brief Vector holding the starting indices for the position states of the body and each leg.
    * 
    * The `pos_ids` vector is used to map and extract the position states from the full state vector
    * in a specific order. The full state vector contains the states in the order [body, order of legs in URDF],
    * and `pos_ids` rearranges them to the order [B, FR, BR, FL, BL] using @ref olympus_plant::leg_sim_order
    * 
    * \image html doc_images/pos_id.png "`pos_id` explanation"
    */
    std::vector<int> pos_ids;

    /*!
    * @brief Vector holding the starting indices for the velocity states of the body and each leg.
    * @see pos_ids
    */
    std::vector<int> vel_ids;

    const int base_joint_type;  /*!<  The type of the base joint, from @ref olympus_base_joint */
    const int num_base_pos;     /*!<  Number of generalized position states for the base */
    const int num_base_vel;     /*!<  Number of generalized velocity states for the base */
    const int num_leg_joints;   /*!<  Number of leg joints */
    const int num_legs;         /*!<  Number of legs */
    const int num_states;       /*!<  Number of total states in the system */
    const int num_pos_states;   /*!<  Number of total position states in the system */

};

//TODO: get/set motor limit, attachment points, load attachment points
//TODO: More accurate PID controller or even Torque/Velocity control module

/*! 
 * @brief This class manages a single leg of Olympus
 *
 * This class containts the physical and simulation parameters for a single leg
 * of the quadruped. It is used to set up leg specific elements (eg. bushing 
 * joint to close chain, or leg PID controller) as well as to get access to 
 * these elements (controller ports).
 * 
 * This class organises the leg with the following order of links (and joints)
 * [MotorHousing, HipInner, ShankInner, HipOuter, ShankOuter, Paw] 
 * 
 */
class leg_system {
  public:		

    /*!
     * @brief Constructor.
     * @param [in]  builder The current diagram builder 
     * @param [out] plant_  The multibody plant which containts Olympus
     * @param [in]  leg_prefix_ The prefix of the current leg (same as the URDF)
     */    
    leg_system(drake_builder & builder, drake_plant* plant_, const std::string & leg_prefix_);

    /*!
     * @brief Disable shank collisions, as they form a closed kinematic chain. 
     * @param [out] scene_graph current scene_graph instance
     */
    void handle_shanks_collisions(drake::geometry::SceneGraph<double> & scene_graph);
    
    /*!
     * @brief Returns the reference/desired port for the PID controller of the leg.
     * @return @ref controller_desired_state_port.
     */
    drake::systems::InputPortIndex & get_controller_port();

    /*!
     * @brief Returns a pointer to the current PID controller.  
     * @returns @ref controller
     */
    drake::systems::controllers::PidController<double>* get_controller_ptr();

    /*!
     * @brief Returns a pointer to the system that saturates the output of the 
     * PID controller.
     * @returns @ref controller_saturation
     */
    drake::systems::Saturation<double>* get_controller_saturation_ptr();
    
  private:

    /*!
     * @brief Sets the indices of the actuated joints states in @ref  state_ids.
     *
     * Sets the @ref state_ids variable. It requires that the bodies are passed
     * in the correct order in the `olympus_param.yaml`, which is (Olympus v2):
     * [MotorHousing, HipInner, ShankInner, HipOuter, ShankOuter, Paw] 
     */
    void set_state_ids();

    /*!
     * @brief Add a linear spring connecting the two knees. 
     * 
     * The spring parameters are defined in \ref SpringParams 
     */
    void add_linear_spring();

    /*!
     * @brief Add a bushing joint to close the kinematic chain. 
     * 
     * The bushing parameters are defined in \ref BushingParams 
     */
    void add_bushing_joint();

    /*!
     * @brief This function adds an actuator in the actuated legs of Olympus.
     */
    void add_actuators();

    /*!
     * @brief This function adds a PID controller in the actuated legs of Olympus.
     *
     * Controller gains are defined in \ref Gains. 
     * 
     * @param [in] builder The current diagram builder 
     * @note
     * Currently, the whole input is saturated. A more accurate model to the one
     * used in olympus, would be to saturate P,D,I gains seperately.
     */
    void add_PID_system(drake_builder & builder);   

    // ============= Member Variables: Multibody Elements =======================

    //! Current multibody plant that containts olympus
    drake_plant * plant ; 

    //! Pointer to the PID controller of the current leg
    drake::systems::controllers::PidController<double>* controller =nullptr;

    //! Pointer to the saturation of the  PID controller of the current leg
    drake::systems::Saturation<double>* controller_saturation =nullptr;

    //! The reference/desired port of the PID controller
    drake::systems::InputPortIndex controller_desired_state_port;

    /*!
     * @brief Pointer array to leg bodies.
     *
     * The joint names used to extract the pointers are defined explicitely 
     * in the config file `olympus_param.yaml`.
     */
    std::array< const drake_rigidBody *,5> leg_bodies;

    /*!
     * @brief Pointer array to leg joints.
     *
     * The joint names used to extract the pointers are defined explicitely 
     * in the config file `olympus_param.yaml`.
     */
    std::array< const drake::multibody::Joint<double> *,5> leg_joints;

    // ============= Member Variables: Dynamic parameters =======================
    
    //! Bushing parameters.
    const BushingParamsStruct BushingParams;   

    //! Leg linear spring parameters.
    const SpringParamsStruct  SpringParams; 

    //! PID gains.
    controllerGains     Gains;        

    //! Symmetric motor limits. Each motor has torque `t`:  `-motor_limit[i]< t[i] < motor_limit[i]`.
    std::array<double,3> motor_limit {20,20,20};

    // ============= Member Variables: Internal Elements ========================

    //! number of joints in a single leg (actuated + unactuated).
    const int num_joints = 5;

    //! Prefix of leg. One of the @ref olympus_plant::leg_prefix
    std::string leg_prefix; 

    /*!
     * @brief Indices that point out to position and velocity state of each leg.
     * 
     * @note
     * The states are order as: \n
     * [qMH,qHI,qHO,vMH,vHI,vHO] \n
     * where:
     * - v = dq/dt
     * - MotorHousing (MH), Hip Inner(HI) and HipOuter(HO)
     * 
     */
    std::array<int,6> state_ids{0,3,1,5,8,6} ; 
  
};


/*! 
 * @brief This is the main class that manages the drake simulation of olympus. 
 *
 * This class is responsible to set the drake simulation of Olympus. It loads 
 * the URDF and creates and connects the necessary systems. Also, it exports 
 * the necessary ports to read and control the system. It must be instanciated
 * before calling `builder.Build()`.
 * 
 * @note
 * The user specifies a custom order for the legs in the `olympus_param.yaml` 
 * configuration file, independent of the order in the URDF. This ensures 
 * consistent indexing of leg systems, inputs, controllers,etc., regardless 
 * of the URDF's joint/body order. 
 * See @ref get_joint_mapping, @ref leg_sim_order
 * 
 */
class olympus_plant {
  public:		

    /*!
     * @brief Constructor.
     * @param [in]  builder The current diagram builder. 
     */   
    olympus_plant(drake_builder & builder) ;

    /*!
     * @brief Returns a reference to the current multibody plant.
     * @returns @ref plant
     */   
    drake_plant &  get_plant();

    /*!
     * @brief Returns the update timestep of the multibody plant.
     * @return @ref time_step
     */   
    double  get_timestep() const;

    /*!
     * @brief Returns the update rate of the simulation.
     * @return @ref simulation_update_rate
     */   
    double  get_simulation_update_rate() const;

    /*!
      * @brief Returns the target rate at which the simulation should run in real-time.
      * @return @ref realtime_rate_target
      */   
    double get_realtime_rate_target() const;

    /*!
     * @brief Returns whether to update drake events (mainly  visualization) every simulation time step.
     * @return @ref publish_every_time_step
     */   
    bool  publish_every_time_step() const;

    /*!
     * @brief Returns a reference to the specified leg_system.
     * @param[in] index index of leg.
     * @return the specified leg_system from @ref legs.
     */   
    leg_system&  get_leg(const int & index);

    /*!
     * @brief Returns a reference to the state demultiplexer. 
     * @returns @ref outputer
     */   
    olympus_state_demultiplexer&  get_state_demux();

    /*!
     * @brief Returns the joint_type of the base. 
     *
     * Returns @ref joint_type.
     * One of the enums of  @ref  olympus_base_joint.
     */
    int get_base_joint_type() const {   return joint_type;  }

    /*!
     * @brief  The prefixes of the legs in the desired order.
     *
     * This order is determined by the order of legs in `olympus_param.yaml`.
     * @see leg_sim_order
     */
    static  std::array<std::string,4> leg_prefix;

    /*!
     * @brief  An array that maps the desired leg order to the URDF order.
     *
     * leg_sim_order[i] is the ordering in the URDF of the i-th leg in the
     * `olympus_param.yaml`. 
     * **Usage:** Use this to access the drake elements with the Custom Order.
     * 
     * Input: index based on CO. Output: index on URDF/drake.
     * 
     * @note
     * A distinction should be made between the Custom Order of the legs (CO)
     * and the URDF order (UO) which can change. The legs are passed in olympus
     * class according to the CO. So this variable matches every leg in the (CO)
     * to the corresponding leg in the URDF.
     * 
     * @see get_joint_mapping
     * 
     */
    static  std::array<int,4> leg_sim_order; // The leg order mapping between config file-ros and urdf 

  private:

    /*!
     * @brief Connects the quadruped to the simulation world.
     *
     * The connection depends on the `base_joint_type` from `olympus_param.yaml`.
     * @param[in] torso_body The torso body.
     */
    void add_base_joint(const drake_rigidBody & torso_body);

    /*!
     * @brief Adds the testbed geometry.
     * @returns a reference to the testbed body.
     */
    const drake_rigidBody* add_testbed_body();

    /*!
     * @brief This function finds the mapping from the custom order to the urdf 
     * body order.
     *
     * The custom order is defined in `olympus_param.yaml`. The urdf order 
     * depends on the export. 
     * @see leg_sim_order
     */
    void get_joint_mapping(
      const std::vector<drake::multibody::JointIndex> & joint_index_vector,
      const YAML::Node & robot_config);


    /*!
     * @brief An array containing pointers to the leg systems. 
     */
    std::array< leg_system *,4> legs;   

    /*!
     * @brief A pointer to the olympus demultiplexer.
     */ 
    olympus_state_demultiplexer * outputer;  

    //! Current multibody plant that contains olympus.
    drake_plant * plant;

    //! Model instance of the quadruped.
    drake::multibody::ModelInstanceIndex quadruped;

    //! base joint type.
    int joint_type = olympus_base_joint::weld;

    //! Pointer to the base joint.
    const drake::multibody::Joint<double> * joint_ptr; 

    //! Multibody simulation timestep [s]. 
    double time_step = 0.001;
    
    //! Update rate of the simulation quantities (states, visualization etc) [s]. 
    double simulation_update_rate = 0.01; 

    /*!
     * @brief The target rate at which the simulation should run in real-time. 
     *
     * See [this](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html#abda3671ab5939691e7e39fcba568174b)
     */ 
    double realtime_rate_target = 1 ; 
    
    /*!
     * @brief Whether to update visualization (mainly) every simulation time step.
     *
     * See [this](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html#aef1dc6aeb821503379ab1dd8c6044562)
     */ 
    bool publish_every_time_step_ = false; 
    
    //! Tespbed connection height [m].
    double testbed_DZ = 0;

    //! Testbed damping parameter [Nm / (rad/s) ].
    double testbed_damping = 0;
    
};