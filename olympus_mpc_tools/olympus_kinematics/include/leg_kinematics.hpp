#include <eigen3/Eigen/Dense>
#include "kinematic_defines.hpp"

#define SQUARE(x) ((x)*(x))
#define ANLGE_DIST(x1,x2,x3,x4)  SQUARE(x1)+SQUARE(x2)+SQUARE(x3)+SQUARE(x4)
#define LEG_CONFIGURATION(leg_id) ( (leg_id==0 || leg_id==3 ) ?1 :2 ) //zero based indexing
#define CONFIG_SIGN(leg_id)  (LEG_CONFIGURATION(leg_id)==1?1:-1) 

// This library handles the kinematics of olympus leg. 
// All angles in this library are in  [RADIANS]
// 
// The kinematics were analyzed with in a different frame than that of the URDF. And thus there is an internal conversion 
// from the *Kinematic Frame* to the *URDF Frame*. This convention can be updated by changing parameters in the `kinematic_defines.hpp`
// 
//
// API functions: angles in robot frame
// Internal Functions: angles in kinematic frame

//TODO: RETURN Cartesian positions in urdf frame in `DK`and `state_estimation_AND_DK`
//TODO: Handle Robot frame in IK
//TODO: Handle feasibility checks better in IK
//TODO: [optimization] maybe using constexpr some member variables can be global across all instances of this class 

enum joint_id {jMH,j11,j21,j12,j22};

class leg_kinematics {
  public:		
  leg_kinematics(int leg_id_):
  leg_id(leg_id),
  config_param( LEG_CONFIGURATION(leg_id_)),
  Pj11{j11_Dx, CONFIG_SIGN(leg_id_)*j_Dy,z_MH_j11j21},
  Pj12{j12_Dx, CONFIG_SIGN(leg_id_)*j_Dy,z_MH_j11j21},
  config_sign( CONFIG_SIGN(leg_id_)),
  hip_link_margin(1 + shank_margin/(2*l11) ),
  conversion_projection( get_correct_conversion_matrix(leg_id_) ),
  conversion_offsets ( get_conversion_offsets() )  {}
    

  // Forward Kinematics:
  // ===================
  public:	
  /// @brief Return the position of the end effector in the {MH} frame [atm in kinematic frame]. 
  /// @param[in] JointPositionVector_robot The measured joint positions `[qMH,q11,q12]`, in the URDF Frame
  /// @return p_EE in {MH} frame  //TODO: return in URDF Frame (returns in Kinematic frame atm)
  /// @note The leg doesn't know its position in the world frame. The user is responsible for further transformations.
  /// 
  /// @details
  /// It calculates the end_effector position in {MHr} frame using `end_effector_2D` and then transforms it to the {MH}
  /// using `end_effector_MH`.
  ///
  Eigen::Vector3d DK(const Eigen::Vector3d &JointPositionVector_robot);

  // helper functions:
  private: 
  /// @brief Calculates the end_effector position in the {MHr} frame and saves it in `p_EE_2d` field. Also, saves the positions of j21,j22 
  ///(`pc1`,`pc2`) in {MHr} frame  that is used for state estimation by `state_estimation()`. 
  ///
  /// @param[in] JointPositionVector The measured joint positions `[qMH,q11,q12]` , in the kinematic frame
  /// @note
  /// - Internal function: Kinematic frame as input - output. 
  /// - Depending on `config_param` it handles the 2 configurations of the leg. 
  /// 
  void end_effector_2D(const Eigen::Vector3d &JointPositionVector);
  
  /// @brief Calculates the end_effector position in the {MH} frame using the current `p_EE_2d` that is calculated
  /// from `end_effector_2D`
  ///
  /// @param[in] JointPositionVector The measured motor housing joint position `qMH` in the kinematic frame
  /// @return  p_EE in {MH} frame
  ///
  /// @note Internal function: Kinematic frame as input - output
  /// 
  Eigen::Vector3d end_effector_MH(const double &qMH);
  

  // State Estimation:
  // ===================
  public:

  /// @brief Calculates the full state of the leg `[qMH,q11,q21,q12,22]`. 
  ///
  /// @param[in] JointPositionVector_robot The measured joint positions `[qMH,q11,q12]`, using the URDF Frame
  /// @return 5x1 joint_position_vector in URDF Frame 
  /// @note API function: Robot frame as input
  /// @details It calls the `end_effector_2D` to update `pc1`,`pc2`,`p_EE_2d`. If we want to update the current 
  /// EE position and estimate the state we must use `state_estimation_AND_DK`.
  /// 
  Eigen::Matrix<double,5,1> state_estimation(const Eigen::Vector3d &JointPositionVector_robot);

  /// @brief Calculates the full state of the leg `[qMH,q11,q21,q12,22]` and the end effector position in {MH} frame in an efficient way
  ///
  /// @param[in] JointPositionVector_robot The measured joint positions `[qMH,q11,q12]`, in the ROBOT FRAME
  /// @return a pair of the full joint positions [URDF FRAME] and EE position [Kinematic Frame] //TODO: return pEE in URDF FRAME
  /// @note It uses the dedicated functions  `DK`  and  `state_estimation`
  /// 
  std::pair< Eigen::Matrix<double,5,1>,Eigen::Vector3d> state_estimation_AND_DK(const Eigen::Vector3d &JointPositionVector_robot);
      

  // Helpers
  // ===================
  private:    
  /// @param motor housing angle in kinematic convetion
  /// @return rotation matrix about x-axis
  Eigen::Matrix3d rotate_x(const double & th);

  //Conversions between URDF frames and Kinematic frames

  /// @brief Converts joint angles from URDF Frame to Kinematic frame
  ///
  /// @param[in] q_robot The measured joint positions `[qMh,qHI,qHO]=[qMH,q11,q12]` in URDF frame
  /// @return  The measured joint positions  in Kinematic frame
  /// @note Input must be aldready in radians.
  /// 
  Eigen::Vector3d robotToCustom(const Eigen::Vector3d & q_robot);

  /// @brief Converts joint angles from Kinematic frame to URDF frame. 
  ///
  /// @param[in] q_robot The full joint positions `[qMh,qHI,qSI,qHO,qSO]=[qMH,q11,q21,q12,q22]` in Kinematic frame
  /// @return The full joint positions in URDF frame
  /// @note Input must be in radians.
  /// 
  Eigen::Matrix<double,5,1> customToRobot(const Eigen::Matrix<double,5,1> & q_kinematics);

  /// @brief Initialization helper function to populate `conversion_projection` member according to leg id. 
  /// @param leg_id Leg index: [0: FR, 1: RR, 2: FL, 3: RL]
  /// @return an 5x1 eigen projeciton matrix
  Eigen::Matrix<double,5,1> get_correct_conversion_matrix(const int & leg_id) ;

  /// @brief Initialization helper function to populate `conversion_offsets`
  Eigen::Matrix<double,5,1> get_conversion_offsets() ;

public:
    // IK

    /// TODO: handle  infeasible commands <-> never send nan to RR_IK


    /// @brief Return the input in`motor3` frame. 
    ///
    /// Returns [joint1,joint2,motor3]
    /// @brief Return the joint angles for a desired position of the end effector in the `motor3` frame. 
    ///
    /// [joint1,joint2,motor3]
    /// @param[in] p_M3 an `Eigen::Vector3d` containing the desired position of the leg in `motor3` frame
    /// @return Desired joint angles (MUST ADD SECOND CONFIGURATION)
    /// @note The leg doesn't know its position in the world frame. The user is responsible for further transformations. 
    /// 
    /// TO DO: 
    /// 1. add second configuration.
    /// 2. Add checks for valid desired positions. 
    ///
    Eigen::Vector3d IK(const Eigen::Vector3d & pD_MH  );

private:
    /// TODO: handle  infeasible commands <-> never send theta2 as nan to RR_IK_system

    /// @brief Returns the solutions of the inverse kinematics for an RR manipulator
    ///
    /// @param[in] chain [template] integer {1,2} to denote which chain we are working on.
    /// @param[in] Pd a 2D vector with the [x;y] position of the leg in the {MHr} frame.
    /// @return An `std::array` with 2 arrays for the 2 different solutions. Each array contains 2 angles `q1i` and `q2i` where i=1,2 for each chain.
    /// @note The functions does the inverse kinematics for a planar RR manipulator, thus it works in 2D vectors. The returned angles are joint positions, 
    /// meaning offsets are already compensated. This function uses `constexpr if` to create two versions for each chain, and thus requires std++17
    /// 
    template <int chain>
    std::array<std::array<double,2>,2> RR_IK(const Eigen::Vector2d & Pd);

    /// @brief This function solves the system that arises in the RR IK problem after finding `theta2` solutions.
    ///
    /// @param[in] Pd Desired Position of the end effector
    /// @param[in] r1 Length of link 1 
    /// @param[in] r2 Length of link 2 
    /// @param[in] theta2 Solution of `theta2` 
    /// @return The value of `theta1`, or `NaN` if `theta2` is `NaN`. 
    ///
    /// @note There is always a solution in this system if theta2 is not nan.
    ///
    inline double RR_IK_system(const Eigen::Vector2d & Pd,const double r1,const double r2,const double & theta2);

    void feasibility_check();
     
    /// @brief This function checks if an RR_IK respects the joint limits of the RR manipulator
    ///
    /// @param[in] sol A `std::array<double,2>` array containing [q1i,q2i]
    /// @param[in] chain_n Which chain is being checked -> i subscript
    /// @return true if both joint angles respect joint limits
    /// 
    inline bool solution_in_limits(const std::array<double,2> & sol, const  int & chain_n);

    /// @brief This function checks if a joint angle is in the specified limits.
    ///
    /// @param[in] q joint angle value
    /// @param[in] joint_index which joint is being checked
    /// @return true if in joint angle limits false otherwise 
    inline bool joint_angle_in_limits(const double & q, const  int & joint_index);
    
    bool avoid_self_collisions(const  int & c1_id,const  int & c2_id);    


    std::array<int,2> RR_best_sol( 
    const std::array<std::array<double,2>,2> & SOLS_1,
    const std::array<std::array<double,2>,2> & SOLS_2 );
    inline double angle_distance(const std::array<double,2>& sol_chain1,const std::array<double,2>& sol_chain2,const Eigen::Matrix<double,5,1> qpos );
    


// private:
public:
//Geometrical Quantites:
const double l11    = LENGTH_HIP_INNER;
const double l21    = LENGTH_SHANK_INNER;

const double l12    = LENGTH_HIP_OUTER;
const double l22    = LENGTH_SHANK_OUTER;

const Eigen::Vector3d Pj11{j11_Dx, j_Dy,z_MH_j11j21} ; // `pj11` is the center of joint11 in the rotated {MH} frame
const Eigen::Vector3d Pj12{j12_Dx, j_Dy,z_MH_j11j21} ; // `pj12` is the center of joint12 in the rotated {MH} frame

//limits
const std::array<double,2> qMH_lim { qMH_lb,qMH_ub};
const std::array<double,2> q11_lim { q11_lb,q11_ub};
const std::array<double,2> q21_lim { q21_lb,q21_ub};
const std::array<double,2> q12_lim { q12_lb,q12_ub};
const std::array<double,2> q22_lim { q22_lb,q22_ub};
const std::array< const std::array<double,2> * , 5> qlim {&qMH_lim,&q11_lim,&q21_lim,&q12_lim,&q22_lim }; 

//Kinematic Quantities
const int leg_id;
const int config_param; // [config 1: {FR,RL}, config2: {FL,RR}] 
const double config_sign;

//DK specific:
Eigen::Vector2d p1c,p2c;      //`p1c`,`p2c` are circle centers in the rotated {MH} frame. Used in `DK`, and saved for state estimation
Eigen::Vector2d p_EE_2d;      //End Effector position vector in the rotated {MH} frame -> in the projected plane -> 2d.

// Current state (may be deleted):
Eigen::Matrix<double,5,1> q;  //Joint Angles <- update before calling IK

//IK specific:
std::array<std::array<double,2>,2> SOLS_c1;
std::array<std::array<double,2>,2> SOLS_c2;

// Boolean array for the feasibility checks. (s_ci = solution of chain)
// { 
//   [s_c1[0] + s_c2[0],  s_c1[0] + s_c2[1] ], 
//   [s_c1[1] + s_c2[0],  s_c1[1] + s_c2[1] ] 
// }
// s.t `feasibility_matrix[i][j]` -> is imidiatelly `SOLS_c1[i]`, `SOLS_c2[j]` 
std::array<std::array<bool,2>,2> feasibility_matrix;

const double shank_margin    = 0.03; //experimentally
const double hip_link_margin; //experimentally

//Correction matrices:
const Eigen::Matrix<double,5,1>  conversion_projection; 
const Eigen::Matrix<double,5,1>  conversion_offsets; 




const std::array<double,5> offsets ={qMH_offset,q11_offset ,q21_offset,q12_offset ,q22_offset}; //Kinematic modelling offsets -> see pdf
};

