#include <ros/ros.h>
//link with generated library:
//with ACADOS_INCLUDE_DIRS
// #include "acados/utils/types.h"
#include "eigen3/Eigen/Dense"

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "c_generated_code_leg/acados_solver_fr_leg_torque.h" 

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// #define NX     FR_LEG_TORQUE_NX
// #define NZ     FR_LEG_TORQUE_NZ
// #define NU     FR_LEG_TORQUE_NU
// #define NP     FR_LEG_TORQUE_NP
// #define NBX    FR_LEG_TORQUE_NBX
// #define NBX0   FR_LEG_TORQUE_NBX0
// #define NBU    FR_LEG_TORQUE_NBU
// #define NSBX   FR_LEG_TORQUE_NSBX
// #define NSBU   FR_LEG_TORQUE_NSBU
// #define NSH    FR_LEG_TORQUE_NSH
// #define NSG    FR_LEG_TORQUE_NSG
// #define NSPHI  FR_LEG_TORQUE_NSPHI
// #define NSHN   FR_LEG_TORQUE_NSHN
// #define NSGN   FR_LEG_TORQUE_NSGN
// #define NSPHIN FR_LEG_TORQUE_NSPHIN
// #define NSBXN  FR_LEG_TORQUE_NSBXN
// #define NS     FR_LEG_TORQUE_NS
// #define NSN    FR_LEG_TORQUE_NSN
// #define NG     FR_LEG_TORQUE_NG
// #define NBXN   FR_LEG_TORQUE_NBXN
// #define NGN    FR_LEG_TORQUE_NGN
// #define NY0    FR_LEG_TORQUE_NY0
// #define NY     FR_LEG_TORQUE_NY
// #define NYN    FR_LEG_TORQUE_NYN
// #define NH     FR_LEG_TORQUE_NH
// #define NPHI   FR_LEG_TORQUE_NPHI
// #define NHN    FR_LEG_TORQUE_NHN
// #define NPHIN  FR_LEG_TORQUE_NPHIN
// #define NR     FR_LEG_TORQUE_NR


int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "test_acados");
ros::NodeHandle nh;


    fr_leg_torque_solver_capsule *acados_ocp_capsule_leg = fr_leg_torque_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N_l = FR_LEG_TORQUE_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = fr_leg_torque_acados_create_with_discretization(acados_ocp_capsule_leg, N_l, new_time_steps);

    if (status)
    {
        printf("fr_leg_torque_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config_leg = fr_leg_torque_acados_get_nlp_config(acados_ocp_capsule_leg);
    ocp_nlp_dims *nlp_dims_leg = fr_leg_torque_acados_get_nlp_dims(acados_ocp_capsule_leg);
    ocp_nlp_in *nlp_in_leg = fr_leg_torque_acados_get_nlp_in(acados_ocp_capsule_leg);
    ocp_nlp_out *nlp_out_leg = fr_leg_torque_acados_get_nlp_out(acados_ocp_capsule_leg);
    ocp_nlp_solver *nlp_solver_leg = fr_leg_torque_acados_get_nlp_solver(acados_ocp_capsule_leg);
    void *nlp_opts_leg = fr_leg_torque_acados_get_nlp_opts(acados_ocp_capsule_leg);

    // initial condition
    #pragma region
    std::array<int,FR_LEG_TORQUE_NBX0> idxbx_l;
    for (int i=0; i < FR_LEG_TORQUE_NBX0 ; i++) { idxbx_l[i] = i;}
    
    Eigen::Matrix<double,10,1> x0;
    x0.setZero();
    Eigen::Vector3d u0;
    u0.setZero();

    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "idxbx", idxbx_l.data());
    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "ubx", x0.data());

    #pragma endregion

   
    //Update weights:
    #pragma region
    
    //Class variables
    Eigen::Matrix<double,17,1> W;
    Eigen::Matrix<double,10,1> Q;

    Eigen::Matrix<double,10,1> Wstate;
    Eigen::Matrix<double,3,1>  Wtrack;
    Eigen::Matrix<double,2,1>  Wu;
    Eigen::Matrix<double,2,1>  Wclosure; 

    //1. Update new phase constraints (7_values Wtrack:2, Wu, Wpos, Wvel, Qpos, Qvel) -> save in reset_defines
    Wclosure.setZero() ;
    Wtrack << 5,5,160;
    Wu     << 5,5;
    Wstate.head(5) << 0.01* Eigen::Matrix<double,5,1>::Ones();
    Wstate.tail(5) << 2   * Eigen::Matrix<double,5,1>::Ones();
    
    Q.head(5) << 0.01* Eigen::Matrix<double,5,1>::Ones();
    Q.tail(5) << 0.02* Eigen::Matrix<double,5,1>::Ones();

    //2. Populate Consecutive Vector
    W.segment(0,3)  = Wtrack;
    W.segment(3,2)  = Wu;
    W.segment(5,2)  = Wclosure;
    W.segment(7,10) = Wstate; 

    //3. Flatten diagonal arrays:
    std::array<double,17*17> Wflatt; //Class Variable -> so i do not fill with 0 all the time
    std::array<double,10*10> Wflatt_e; //Class Variable

    Wflatt.fill(0); //constructor
    Wflatt_e.fill(0);
    

    for (int i=0;i<17 ; i++){
        Wflatt[i*17+i] = W[i] ;
    }

    for (int i=0;i<10 ; i++){
        Wflatt_e[i*10+i] = Q[i] ;
    }

    //4. Update ocp structure
    for (int i = 0; i < N_l; i++){
        ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, i, "W", Wflatt.data());
    }
    ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, N_l, "W", Wflatt_e.data());
    #pragma endregion

    
    //Update reference
    #pragma region
    //Class vars
    Eigen::Vector3d tref_mh( 0,0,1 ) ; //Only this updates
    Eigen::Vector2d closure_ref(0,0);
    Eigen::Vector2d input_ref(0,0);

    Eigen::Matrix<double,FR_LEG_TORQUE_NX,1> xref;
    xref.tail(5).setZero();
    //q_interrupt
    xref.head(5) << 1.4500,   -1.1000,    0.0816,    1.8500,    1.0829;

    Eigen::Matrix<double,17,1> y_ref ;
    y_ref.segment(0,3) = tref_mh; //This updates
    y_ref.segment(3,2) = input_ref;
    y_ref.segment(5,2) = closure_ref;
    y_ref.tail(10)     = xref; //This updates y_ref.segment(7,5) = q_interrupt


    for  (int i = 0; i < N_l; i++) {
        ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, i, "y_ref", y_ref.data());
    }
    ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, N_l, "y_ref", xref.data());
    #pragma endregion


    for (int i = 0; i < N_l; i++)
    {
        ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "x", x0.data());
        ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "u", u0.data());
    }

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    std::array<double, FR_LEG_TORQUE_NX * (FR_LEG_TORQUE_N+1)> xtraj_l;
    std::array<double, FR_LEG_TORQUE_NU * (FR_LEG_TORQUE_N  )> utraj_l;


    // solve ocp in loop
    int leg_planner_counter = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N_l; i++)
        {
            ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "x", x0.data());
            ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "u", u0.data());
        }
        ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, N_l, "x", x0.data());
        ocp_nlp_solver_opts_set(nlp_config_leg, nlp_opts_leg, "rti_phase", &leg_planner_counter);
        status = fr_leg_torque_acados_solve(acados_ocp_capsule_leg);
        ocp_nlp_get(nlp_config_leg, nlp_solver_leg, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims_leg->N; ii++)
        ocp_nlp_out_get(nlp_config_leg, nlp_dims_leg, nlp_out_leg, ii, "x", &xtraj_l[ii*FR_LEG_TORQUE_NX]);
    for (int ii = 0; ii < nlp_dims_leg->N; ii++)
        ocp_nlp_out_get(nlp_config_leg, nlp_dims_leg, nlp_out_leg, ii, "u", &utraj_l[ii*FR_LEG_TORQUE_NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( FR_LEG_TORQUE_NX, N_l+1, xtraj_l.data(), FR_LEG_TORQUE_NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( FR_LEG_TORQUE_NU, N_l, utraj_l.data(), FR_LEG_TORQUE_NU );
    // ocp_nlp_out_print(nlp_solver_leg->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("fr_leg_torque_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("fr_leg_torque_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config_leg, nlp_dims_leg, nlp_out_leg, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config_leg, nlp_solver_leg, "sqp_iter", &sqp_iter);

    fr_leg_torque_acados_print_stats(acados_ocp_capsule_leg);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = fr_leg_torque_acados_free(acados_ocp_capsule_leg);
    if (status) {
        printf("fr_leg_torque_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = fr_leg_torque_acados_free_capsule(acados_ocp_capsule_leg);
    if (status) {
        printf("fr_leg_torque_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}