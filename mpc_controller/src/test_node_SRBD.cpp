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
#include "c_generated_code_SRBD/acados_solver_SRBD.h"


// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     SRBD_NX
#define NZ     SRBD_NZ
#define NU     SRBD_NU
#define NP     SRBD_NP
#define NBX    SRBD_NBX
#define NBX0   SRBD_NBX0
#define NBU    SRBD_NBU
#define NSBX   SRBD_NSBX
#define NSBU   SRBD_NSBU
#define NSH    SRBD_NSH
#define NSG    SRBD_NSG
#define NSPHI  SRBD_NSPHI
#define NSHN   SRBD_NSHN
#define NSGN   SRBD_NSGN
#define NSPHIN SRBD_NSPHIN
#define NSBXN  SRBD_NSBXN
#define NS     SRBD_NS
#define NSN    SRBD_NSN
#define NG     SRBD_NG
#define NBXN   SRBD_NBXN
#define NGN    SRBD_NGN
#define NY0    SRBD_NY0
#define NY     SRBD_NY
#define NYN    SRBD_NYN
#define NH     SRBD_NH
#define NPHI   SRBD_NPHI
#define NHN    SRBD_NHN
#define NPHIN  SRBD_NPHIN
#define NR     SRBD_NR


int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "test_acados");
ros::NodeHandle nh;

SRBD_solver_capsule *acados_ocp_capsule = SRBD_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = SRBD_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = SRBD_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("SRBD_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = SRBD_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = SRBD_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = SRBD_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = SRBD_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = SRBD_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = SRBD_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;

    #pragma region 
    std::array<int,NBX0> idxbx;
    for (int id=0; id< NBX0 ; id++){ idxbx[id]=id; }
    Eigen::Matrix<double,NBX0,1> x0 ;
    x0.setZero();   
    x0.head(4) << 0.8839,   -0.3062,    0.1768,    0.3062;

    Eigen::Vector3d u0;
    u0.setZero();

    Eigen::Quaterniond q_ref;
    q_ref.setIdentity();
    Eigen::Matrix<double,4,1> q_conv; 
    q_conv[0] = q_ref.w();
    q_conv.tail(3) = q_ref.vec();

    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0.data());

    // set parameters
    for (int ii = 0; ii <= N; ii++)
    {
        SRBD_acados_update_params(acados_ocp_capsule, ii, q_conv.data(), NP);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0.data());
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0.data());
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0.data());
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = SRBD_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("SRBD_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("SRBD_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    SRBD_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = SRBD_acados_free(acados_ocp_capsule);
    if (status) {
        printf("SRBD_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = SRBD_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("SRBD_acados_free_capsule() returned status %d. \n", status);
    }

    return status;



}