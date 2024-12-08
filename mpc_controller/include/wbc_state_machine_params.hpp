#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>

enum controller_mode { roll,pitch,yaw,stabilizing };
enum phase {mid, reset, ext, torque };




namespace resetting{
#define TORQUE_MAGNITUDE_THRESH 0.2 
#define ANGULAR_STABILIZATION_THRESH (5 *M_PI/180)
#define ANGULAR_STABILIZATION_THRESH_EXIT (10 *M_PI/180)
constexpr int NUMBER_OF_PHASES = 4;

//u is saved in a large array
//To avoid passing the define, we use this
//TODO: maybe implement with array, if sizes of mpcs are defined here
inline int get_max_u_id(const double*  u ){
  int id_max;
  id_max = ( abs( u[0]      )  > abs( u[1] ) ) ? 0      : 1 ; 
  return   ( abs( u[id_max] )  > abs( u[2] ) ) ? id_max : 1 ;
}

void populate_diagonal(Eigen::Matrix<double,5,5> & vec, const std::array<double,5> arr){
  vec.diagonal() << arr[0],arr[1],arr[2],arr[3],arr[4];
}

void populate_vector(Eigen::Matrix<double,5,1> & vec, const std::array<double,5> arr){
  vec << arr[0],arr[1],arr[2],arr[3],arr[4];
}

constexpr std::array<double, 12> populateCollectiveWeightMatrices( 
  const double (&Wtrack)[3], 
  const double & Wu, 
  const double (&Wstate)[4], 
  const double (&Q)[4]){

    // double arr[12];  
    std::array<double,12> arr = {
      Wtrack[0],
      Wtrack[1],
      Wtrack[2],
      Wu,
      Wstate[0],
      Wstate[1],
      Wstate[2],
      Wstate[3],
      Q[0],
      Q[1],
      Q[2],
      Q[3],
    };

    return arr;
  }

// ===== ROLL: ======== 
#pragma region 

#define ROLL_MAX_POS_TRACKING_TORQUE  4
#define ROLL_MAX_NEG_TRACKING_TORQUE -4
#define SATURATE_ROLL(x)  (   MAX( MIN(x, ROLL_MAX_POS_TRACKING_TORQUE), ROLL_MAX_NEG_TRACKING_TORQUE) )

constexpr std::array<double, 5> roll_pos   {  1.85,        0,        0,        0,        0 };
constexpr std::array<double, 5> roll_neg   {  -1.2,   0.3500,  -0.5542,   0.3500,   0.5605 };
constexpr std::array<double, 5> roll_ext   {     0,        0,        0,        0,        0 };
constexpr std::array<double, 5> roll_mid   {     0,   1.3500,  -2.0695,   1.5000,   2.1095 };

constexpr std::array<double, 5> roll_CG    {0.1625,   0.4250,  -0.6951,   0.4625,   0.7161 } ;

constexpr std::array<double, 5> roll_W_mid {     1,     5,        0,        5,        0 };
constexpr std::array<double, 5> roll_W_res {     1,     0,        0,        0,        0 };
constexpr std::array<double, 5> roll_W_ext {     1,     0,        0,        0,        0 };
constexpr std::array<double, 5> roll_W_tor {     1,     0,        0,        0,        0 };

#define roll_Th_mid 0.25
#define roll_Th_res 0.25
#define roll_Th_ext 0.1
#define roll_Th_tor 0.25

constexpr std::array<const std::array<double, 5> * , 4> roll_setpoints  {  &roll_mid,   &roll_neg,   &roll_ext,   &roll_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> roll_W          {&roll_W_mid, &roll_W_res, &roll_W_ext, &roll_W_tor } ;
constexpr std::array<double  , 4>                       roll_thresholds {roll_Th_mid, roll_Th_res, roll_Th_ext, roll_Th_tor } ;

// mid weights:
constexpr double roll_Wtrack_mid[3] = {5,0,1};
constexpr double roll_Wu_mid        = 2;
constexpr double roll_Wstate_mid[4] = {   1, 100, 1e-2, 1e-2};     //W(1),W(2:5),W(6),W(7:10)
constexpr double roll_Q_mid[4]      = {1000, 200, 5e-3, 5e-3}; //Q(1),Q(2:5),Q(6),Q(7:10)

// res weights:
constexpr double roll_Wtrack_res[3] = {5,0,0};
constexpr double roll_Wu_res        = 1;
constexpr double roll_Wstate_res[4] = {1e-2,    5,  2.5,  2.5};     //W(1),W(2:5),W(6),W(7:10)
constexpr double roll_Q_res[4]      = { 100, 1e-2, 5e-2, 5e-2}; //Q(1),Q(2:5),Q(6),Q(7:10)

// ext weights:
constexpr double roll_Wtrack_ext[3] = {120,5,5};
constexpr double roll_Wu_ext        = 2;
constexpr double roll_Wstate_ext[4] = {1e-2, 100,  2.5,  10};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double roll_Q_ext[4]      = {   1, 100, 5e-3, 10}; //Q(1),Q(2:5),Q(6),Q(7:10)

// tor weights:
constexpr double roll_Wtrack_tor[3] = {120,5,5};
constexpr double roll_Wu_tor        = 0.5;
constexpr double roll_Wstate_tor[4] = {1e-2,  100,  2.5,   10};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double roll_Q_tor[4]      = {1e-2, 1e-2, 5e-2,   10}; //Q(1),Q(2:5),Q(6),Q(7:10)

constexpr std::array<double, 12> roll_weights_mid = populateCollectiveWeightMatrices(roll_Wtrack_mid,roll_Wu_mid,roll_Wstate_mid,roll_Q_mid)  ;
constexpr std::array<double, 12> roll_weights_res = populateCollectiveWeightMatrices(roll_Wtrack_res,roll_Wu_res,roll_Wstate_res,roll_Q_res)  ;
constexpr std::array<double, 12> roll_weights_ext = populateCollectiveWeightMatrices(roll_Wtrack_ext,roll_Wu_ext,roll_Wstate_ext,roll_Q_ext)  ;
constexpr std::array<double, 12> roll_weights_tor = populateCollectiveWeightMatrices(roll_Wtrack_tor,roll_Wu_tor,roll_Wstate_tor,roll_Q_tor)  ;

constexpr std::array<const std::array<double, 12> * , 4> roll_weights  {  &roll_weights_mid,   &roll_weights_res,   &roll_weights_ext,   &roll_weights_tor } ;


#pragma endregion

// ===== PITCH: ======== 
#pragma region 

#define PITCH_MAX_POS_TRACKING_TORQUE  2.5
#define PITCH_MAX_NEG_TRACKING_TORQUE -2.5
#define SATURATE_PITCH(x)  (   MAX( MIN(x, PITCH_MAX_POS_TRACKING_TORQUE), PITCH_MAX_NEG_TRACKING_TORQUE) )

constexpr std::array<double, 5> pitch_pos   {      0,  2.0900,  -1.5456,  -0.3330,   1.0078 };
constexpr std::array<double, 5> pitch_neg   {      0, -1.0000,  -1.3412,   3.5000,   1.8464 };
constexpr std::array<double, 5> pitch_ext   {      0,       0,        0,        0,        0 };
constexpr std::array<double, 5> pitch_mid   { M_PI/2,       0,        0,        0,        0 };

constexpr std::array<double, 5> pitch_CG    { 0.3927,  0.2725,  -0.7498,   0.7917,   0.9314 } ;

constexpr std::array<double, 5> pitch_W_mid {   0.5,     1,        0,        1,        0 };
constexpr std::array<double, 5> pitch_W_res {   0.5,     1,        0,        1,        0 };
constexpr std::array<double, 5> pitch_W_ext {     0,   0.4,        0,        2,        0 };
constexpr std::array<double, 5> pitch_W_tor {     0,   0.4,        0,        2,        0 };

#define pitch_Th_mid 10
#define pitch_Th_res 2
#define pitch_Th_ext 7.5
#define pitch_Th_tor 0.2
#define pitch_inject_1_3 -2 //Inject this in the matrix in the pitch mode for ext-tor phases


constexpr std::array<const std::array<double, 5> * , 4> pitch_setpoints  {  &pitch_mid,   &pitch_neg,   &pitch_ext,   &pitch_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> pitch_W          {&pitch_W_mid, &pitch_W_res, &pitch_W_ext, &pitch_W_tor } ;
constexpr std::array<double  , 4>                       pitch_thresholds {pitch_Th_mid, pitch_Th_res, pitch_Th_ext, pitch_Th_tor } ;

// mid weights:
constexpr double pitch_Wtrack_mid[3] = {1, 10, 1};
constexpr double pitch_Wu_mid        = 1;
constexpr double pitch_Wstate_mid[4] = {  50, 1e-1, 1e-1, 1e-1};     //W(1),W(2:5),W(6),W(7:10)
constexpr double pitch_Q_mid[4]      = {  20,   20, 1e-1, 1e-1}; //Q(1),Q(2:5),Q(6),Q(7:10)

// res weights:
constexpr double pitch_Wtrack_res[3] = {1, 50, 1};
constexpr double pitch_Wu_res        = 5;
constexpr double pitch_Wstate_res[4] = {  50, 1e-1, 1e-1, 1e-1};     //W(1),W(2:5),W(6),W(7:10)
constexpr double pitch_Q_res[4]      = {  20,   20, 1e-1, 1e-1}; //Q(1),Q(2:5),Q(6),Q(7:10)

// ext weights:
constexpr double pitch_Wtrack_ext[3] = {5, 160, 5};
constexpr double pitch_Wu_ext        = 5;
constexpr double pitch_Wstate_ext[4] = { 200,  200, 2.5e-1, 2.5e-1};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double pitch_Q_ext[4]      = {1e-2, 1e-2,   2e-2,   2e-2}; //Q(1),Q(2:5),Q(6),Q(7:10)

// tor weights:
constexpr double pitch_Wtrack_tor[3] = {5, 160, 5};
constexpr double pitch_Wu_tor        = 5;
constexpr double pitch_Wstate_tor[4] = {200, 1e-2,    2,    2};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double pitch_Q_tor[4]      = {1e-2, 1e-2, 2e-2, 2e-2}; //Q(1),Q(2:5),Q(6),Q(7:10)

constexpr std::array<double, 12> pitch_weights_mid = populateCollectiveWeightMatrices(pitch_Wtrack_mid,pitch_Wu_mid,pitch_Wstate_mid,pitch_Q_mid)  ;
constexpr std::array<double, 12> pitch_weights_res = populateCollectiveWeightMatrices(pitch_Wtrack_res,pitch_Wu_res,pitch_Wstate_res,pitch_Q_res)  ;
constexpr std::array<double, 12> pitch_weights_ext = populateCollectiveWeightMatrices(pitch_Wtrack_ext,pitch_Wu_ext,pitch_Wstate_ext,pitch_Q_ext)  ;
constexpr std::array<double, 12> pitch_weights_tor = populateCollectiveWeightMatrices(pitch_Wtrack_tor,pitch_Wu_tor,pitch_Wstate_tor,pitch_Q_tor)  ;

constexpr std::array<const std::array<double, 12> * , 4> pitch_weights  {  &pitch_weights_mid,   &pitch_weights_res,   &pitch_weights_ext,   &pitch_weights_tor } ;

#pragma endregion

// ===== YAW: ======== 
#pragma region 

#define YAW_MAX_POS_TRACKING_TORQUE 2
#define YAW_MAX_NEG_TRACKING_TORQUE -2
#define SATURATE_YAW(x)  (   MAX( MIN(x, YAW_MAX_POS_TRACKING_TORQUE), YAW_MAX_NEG_TRACKING_TORQUE) )

constexpr std::array<double,5> yaw_pos {1.4500,   -1.1000,    0.0816,    1.8500,    1.0829 } ;
constexpr std::array<double,5> yaw_neg {1.4700,    1.0400,   -0.7702,   -0.6000,   -0.1164 } ;
constexpr std::array<double,5> yaw_mid {     0,    0.6800,   -1.2675,    1.0000,    1.3578 } ;
constexpr std::array<double,5> yaw_ext { 1.300,    0.1500,   -0.4166,    0.4700,    0.5689 } ;

constexpr std::array<double,5> yaw_CG {1.0550,    0.1925,   -0.5953,    0.6800,    0.7863 } ;
// constexpr std::array<double,5> yaw_CG {0.7,    0.1925,   -0.5953,    0.6800,    0.7863 } ;


constexpr std::array<double,5> yaw_W_mid {   1,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_res { 0.5,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_ext {   1,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_tor { 0.4,1,0,1,0 } ;

#define yaw_Th_mid 0.5
#define yaw_Th_res 0.2
// #define yaw_Th_res 0.5
#define yaw_Th_ext 0.2
#define yaw_Th_tor 0.5

constexpr std::array<const std::array<double, 5> * , 4> yaw_setpoints  {  &yaw_mid,   &yaw_neg,   &yaw_ext,   &yaw_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> yaw_W          {&yaw_W_mid, &yaw_W_res, &yaw_W_ext, &yaw_W_tor } ;
constexpr std::array<double  , 4>                       yaw_thresholds {yaw_Th_mid, yaw_Th_res, yaw_Th_ext, yaw_Th_tor } ;

// mid weights:
constexpr double yaw_Wtrack_mid[3] = {1,0,1};
constexpr double yaw_Wu_mid        = 2;
constexpr double yaw_Wstate_mid[4] = {100,10,1e-2,1e-2};     //W(1),W(2:5),W(6),W(7:10)
constexpr double yaw_Q_mid[4]      = {500,20,5e-3,5e-3}; //Q(1),Q(2:5),Q(6),Q(7:10)

// res weights:
constexpr double yaw_Wtrack_res[3] = {1,0,0};
constexpr double yaw_Wu_res        = 1;
constexpr double yaw_Wstate_res[4] = {100,1,1e-2,1e-2};     //W(1),W(2:5),W(6),W(7:10)
constexpr double yaw_Q_res[4]      = {100,10,5e-3,5e-3}; //Q(1),Q(2:5),Q(6),Q(7:10)

// ext weights:
constexpr double yaw_Wtrack_ext[3] = {1,0,5};
constexpr double yaw_Wu_ext        = 5e-1;
constexpr double yaw_Wstate_ext[4] = {1e-2,1e-2,2,2};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double yaw_Q_ext[4]      = {100,100,200,1}; //Q(1),Q(2:5),Q(6),Q(7:10)

// tor weights:
constexpr double yaw_Wtrack_tor[3] = {.5,0,120};
constexpr double yaw_Wu_tor        = 5;
constexpr double yaw_Wstate_tor[4] = {500,1e-2,2,2};     //W(1),W(2:5),W(6),W(7:10) 
constexpr double yaw_Q_tor[4]      = {1e3,1e-1,2e-2,2e-2}; //Q(1),Q(2:5),Q(6),Q(7:10)

constexpr std::array<double, 12> yaw_weights_mid = populateCollectiveWeightMatrices(yaw_Wtrack_mid,yaw_Wu_mid,yaw_Wstate_mid,yaw_Q_mid)  ;
constexpr std::array<double, 12> yaw_weights_res = populateCollectiveWeightMatrices(yaw_Wtrack_res,yaw_Wu_res,yaw_Wstate_res,yaw_Q_res)  ;
constexpr std::array<double, 12> yaw_weights_ext = populateCollectiveWeightMatrices(yaw_Wtrack_ext,yaw_Wu_ext,yaw_Wstate_ext,yaw_Q_ext)  ;
constexpr std::array<double, 12> yaw_weights_tor = populateCollectiveWeightMatrices(yaw_Wtrack_tor,yaw_Wu_tor,yaw_Wstate_tor,yaw_Q_tor)  ;

constexpr std::array<const std::array<double, 12> * , 4> yaw_weights  {  &yaw_weights_mid,   &yaw_weights_res,   &yaw_weights_ext,   &yaw_weights_tor } ;




#pragma endregion

// ===== COMPLERE ====
constexpr std::array< const  std::array<const std::array<double, 12> * , 4> *,3> mode_weights {& roll_weights, &pitch_weights, &yaw_weights };

//resetting params
constexpr std::array< const  std::array<const std::array<double, 5> * , 4> *,3>  mode_setpoints  { & roll_setpoints,  &pitch_setpoints,  &yaw_setpoints };
constexpr std::array< const  std::array<const std::array<double, 5> * , 4> *,3>  mode_W          {         & roll_W,          &pitch_W,          &yaw_W };
constexpr std::array< const  std::array<double  , 4> *,3>                        mode_thresholds {& roll_thresholds, &pitch_thresholds, &yaw_thresholds };

//contracting
constexpr std::array< const  std::array<double,5>  *,3>                          mode_CG{& roll_CG, &pitch_CG, &yaw_CG };



//TODO: organize roll,yaw,pitch setpoints and handle stabilization

};
