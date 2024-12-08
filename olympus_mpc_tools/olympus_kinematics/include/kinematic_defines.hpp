#ifndef KINEMATIC_CONFIGS_H
#define KINEMATIC_CONFIGS_H
#include <array>

// Frame correction quantities: To match the URDF
namespace kinematic_correction{
  //This matrices are the direction matrices between kinematic frame and robot frame
  // 1: Same direction in kinematic analysis and hardware
  //-1: Opposite direction in kinematic analysis and hardware
  constexpr std::array<double,5> projMFL_{-1, 1,-1, 1,-1}; 
  constexpr std::array<double,5> projMFR_{-1,-1,-1,-1,-1}; 
  constexpr std::array<double,5> projMRL_{-1,-1, 1,-1, 1}; 
  constexpr std::array<double,5> projMRR_{-1, 1, 1, 1, 1}; 

  #define ROBOT_CONVERSION_FACTOR (M_PI/180.)

  //This offsets is how much each joint [qMH,q11,q12,q21,q22] should rotate in the kinematic frame
  //to reach the zero position of the URDF
  // Transformation is:
  // Proj*( q_kinematic + q_offset ) = q_robot 
  constexpr std::array<double,5> conversion_offsets_{0, 0.738704,-1.17042, -0.735976,1.17777}; 
}

// leg lentghs:
#define LENGTH_HIP_INNER 0.175
#define LENGTH_SHANK_INNER 0.29977

#define LENGTH_HIP_OUTER 0.175
#define LENGTH_SHANK_OUTER 0.29929

// Quantities based on the kinematic analysis: KINEMATIC FRAME:
// This quantities are analyzed in the thesis pdf
#define j11_Dx 0.0517
#define j12_Dx 0.1417
#define j_Dy   -1.1338e-05
#define z_MH_j11j21 -0.05945

#define qMH_offset -M_PI_2
#define q11_offset  2.3095
#define q21_offset  1.3265
#define q12_offset  0.83482
#define q22_offset  -1.3233

// ============
// Joint Limits
// ============

// qMH:
#define qMH_lb -M_PI
#define qMH_ub  M_PI
// q11:
#define q11_lb -M_PI
#define q11_ub  1.65
// q21:
#define q21_lb -2.5
#define q21_ub  1.58
// q12:
#define q12_lb -1.65
#define q12_ub  M_PI
// q22:
#define q22_lb -1.51
#define q22_ub  2.5

#endif // KINEMATIC_CONFIGS_H