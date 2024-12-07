#ifndef OLYMPUS_DRAKE_ROS_HPP
#define OLYMPUS_DRAKE_ROS_HPP

#include <ros/ros.h>
#include "rosgraph_msgs/Clock.h"

#include "olympus_drake.hpp"
// #include "drake_helpers.hpp"

#include "olympus_ros_msgs/MotorStatusArray.h"
#include "olympus_ros_msgs/SetpointArray.h"
#include "olympus_ros_msgs/MotorStatus.h"
#include "nav_msgs/Odometry.h"

#include <functional>
#define NUM_LEGS 4
#define NUM_DOFS_LEG 3
#define NUM_JOINTS_LEG 5
#define TOTAL_DOFS (NUM_LEGS*NUM_DOFS_LEG)


using drake_OutputPortd  = drake::systems::OutputPort<double> ;
using drake_InputPortd   = drake::systems::InputPort<double> ;

/*!
 * @brief Publishes the current simulation time to a ROS topic.
 * 
 * This function creates a ROS clock message with the current simulation time
 * from the Drake context and publishes it to the specified ROS topic.
 * 
 * @param[in] pub The ROS publisher to publish the clock message.
 * @param[in] context The Drake context containing the current simulation time.
 * @return drake::systems::EventStatus::Succeeded() if the event is successfully handled.
 */
drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context );

/*!
 * @brief A structure to hold elements related to the Drake ROS integration.
 * 
 * This structure contains references to the Drake simulator and various input/output ports
 * for the leg and body systems. It also stores the type of the base joint.
 */
struct drake_ros_elements {

  //! Reference to the Drake simulator. 
  drake::systems::Simulator<double> & simulator;

  //! Vector of pointers to leg input ports.
  std::vector <const drake_InputPortd *>  leg_input_ports ;

  //! Vector of pointers to leg output ports.
  std::vector <const drake_OutputPortd *> leg_output_ports ;

  //! Vector of pointers to leg actuation output ports.
  std::vector <const drake_OutputPortd *> leg_actuation_output_ports ;

  //! Pointer to the body output port.
  const drake_OutputPortd * body_output_port; 

  //! Type of the base joint, default is weld. The type is one of the enum of @ref olympus_base_joint
  int base_joint_type = olympus_base_joint::weld ;

  /*!
   * @brief Constructs a drake_ros_elements object with the given simulator.
   * 
   * @param[in] deployed_simulator Reference to the Drake simulator.
   */
  drake_ros_elements( drake::systems::Simulator<double> & deployed_simulator) : simulator(deployed_simulator) {;};

  /*!
   * @brief Constructs a drake_ros_elements object with the given simulator and base joint type.
   * 
   * @param[in] deployed_simulator Reference to the Drake simulator.
   * @param[in] base_joint_type_ Type of the base joint.
   */
  drake_ros_elements( drake::systems::Simulator<double> & deployed_simulator, const int & base_joint_type_)
      : simulator(deployed_simulator), base_joint_type(base_joint_type_) {;};
};

/*!
 * @brief Interface class for integrating Drake simulation with ROS.
 * 
 * This class handles the integration between the Drake simulation environment 
 * and ROS, including publishing simulation data to ROS topics and subscribing 
 * to ROS messages to control the simulation. 
 * 
 * @note The interface emulates the real hardware by publishing messages in the 
 * same manner, allowing a working controller in the simulation to be 
 * immediately tested on the actual hardware.
 */
class olympus_ros_interface {
  public:		

    /*!
     * @brief Constructs an olympus_ros_interface object.
     * 
     * Initializes the ROS publishers and subscribers, and sets up the necessary
     * connections between the Drake simulation and ROS.
     * 
     * @param[in] drake_elements Reference to a drake_ros_elements object containing
     *                           the Drake simulator and various I/O port pointers.
     */
    olympus_ros_interface(drake_ros_elements & drake_elements); 

    /*!
     * @brief Publishes the current state of Olympus to ROS topics.
     * 
     * This function publishes the state of each leg and the body orientation to
     * the corresponding ROS topics.
     */
    void publish();

  private:

    /*!
     * @brief Sets a monitor function in the Drake simulator to publish the simulation clock to ROS.
     * 
     * This function binds the publishClock function to the ROS clock publisher and sets it
     * as a monitor function in the Drake simulator.
     */
    void set_monitor();

    /*!
     * @brief Callback function for the ROS subscriber to set reference positions for the motors.
     * 
     * This function receives a setpoint array message from ROS and updates the reference positions
     * for the motors in the Drake simulation.
     * 
     * @param[in] setpoint_arr Pointer to the received setpoint array message.
     */ 
    void reference_callback(const olympus_ros_msgs::SetpointArray::ConstPtr& setpoint_arr);

    /*!
     * @brief Returns the index (custom order) of the leg system [0,3] based on the motor index [1,12].
     * 
     * @param[in] index The motor id, using 1-based indexing. [1-12]
     * @return The custom ordering index of the corresponding leg.
     */
    int inline get_leg_id(const int & index) const { return leg_order[ (div(index-1,3).quot) ] ; }
  
    /*!
     * @brief Mapping of Custom Leg order (CO) to hardware leg order (HO).
     * 
     * leg_order[i] = is the H0 (actual robot) of the i-th leg in the `olympus_param.yaml` \n
     * **Input**: leg index in the CO. \n **Output**: The HO of the leg .
     */
    std::array<int,NUM_LEGS> leg_order;

    /*!
     * @brief Mapping of Custom Motor Order to the order of the Motors in the Hardware
     *
     * This order is hardcoded.
     * 
     * Custom Order: [Mh,HI,HO]
     * Hardware Order: [HO,HI,Mh]
     */
    std::array<int,NUM_DOFS_LEG> motor_order{2,1,0};  

    //! Same as @ref state_ids from @ref leg_system. //TODO: Must be initialized from it
    const std::array<int,2*NUM_DOFS_LEG> state_ids{0,3,1,5,8,6};


    // ===== INTERNAL ELEMENTS ================================================
    //-------------------------------------------------------------------------

    //! Number of legs for olympus
    const int num_legs =4;

    //! Number of independent degrees of freedom ~ actuators in each leg.
    const int num_DOF_leg = 3;

    //! Type of the base joint.
    const int base_joint_type ;

    // ===== ROS ELEMENTS =====================================================
    //-------------------------------------------------------------------------

    //! ROS publisher for the body orientation.
    ros::Publisher body_orientation_pub; 

    //! ROS publishers for the motor statuses.
    std::vector <ros::Publisher> encoder_pub;

    //! ROS subscribers for the motor commands.
    std::vector <ros::Subscriber> commander_sub;

    //! ROS publisher for the simulation clock.
    ros::Publisher clock_publisher;

    //! Odometry message for the body orientation.
    nav_msgs::Odometry body_odom;

    //! Motor status messages for each leg.
    std::array<olympus_ros_msgs::MotorStatusArray,4> motor_statuses;

    // ===== DRAKE SIMULATION INTERFACE ELEMENTS ==============================
    //-------------------------------------------------------------------------

    //! Reference to the Drake simulator.
    drake::systems::Simulator<double> & simulator;

    //! Vector of pointers to leg input ports.
    std::vector <const drake_InputPortd *>  input_ports;

    //! Vector of pointers to leg output ports.
    std::vector <const drake_OutputPortd *> output_ports;

    //! Vector of pointers to leg actuation output ports.
    std::vector <const drake_OutputPortd *> actuation_output_ports;

    //! Pointer to the body output port.
    const drake_OutputPortd * body_output_port;
};

#endif //OLYMPUS_DRAKE_ROS_HPP
