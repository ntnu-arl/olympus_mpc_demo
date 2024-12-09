/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) SRBD_constr_h_e_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s3[18] = {7, 3, 0, 4, 8, 12, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

/* SRBD_constr_h_e_fun_jac_uxt_zt:(i0[7],i1[4])->(o0[3],o1[7x3,12nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a5, a6, a7, a8, a9;
  a0=5.7295779513082323e+01;
  a1=2.;
  a2=arg[1]? arg[1][0] : 0;
  a3=arg[0]? arg[0][0] : 0;
  a4=casadi_sq(a3);
  a5=arg[0]? arg[0][1] : 0;
  a6=casadi_sq(a5);
  a4=(a4+a6);
  a6=arg[0]? arg[0][2] : 0;
  a7=casadi_sq(a6);
  a4=(a4+a7);
  a7=arg[0]? arg[0][3] : 0;
  a8=casadi_sq(a7);
  a4=(a4+a8);
  a8=1.0000000000000001e-09;
  a4=(a4+a8);
  a4=sqrt(a4);
  a9=(a3/a4);
  a10=(a2*a9);
  a11=arg[1]? arg[1][1] : 0;
  a12=(a5/a4);
  a13=(a11*a12);
  a10=(a10+a13);
  a13=arg[1]? arg[1][2] : 0;
  a14=(a6/a4);
  a15=(a13*a14);
  a10=(a10+a15);
  a15=arg[1]? arg[1][3] : 0;
  a16=(a7/a4);
  a17=(a15*a16);
  a10=(a10+a17);
  a17=casadi_sq(a10);
  a17=(a1*a17);
  a18=1.;
  a17=(a17-a18);
  a19=acos(a17);
  a20=(a9*a11);
  a21=(a12*a2);
  a20=(a20-a21);
  a21=(a14*a15);
  a20=(a20+a21);
  a21=(a16*a13);
  a20=(a20-a21);
  a21=casadi_sq(a20);
  a22=(a9*a13);
  a23=(a14*a2);
  a22=(a22-a23);
  a23=(a12*a15);
  a22=(a22-a23);
  a23=(a16*a11);
  a22=(a22+a23);
  a23=casadi_sq(a22);
  a21=(a21+a23);
  a23=(a9*a15);
  a24=(a2*a16);
  a23=(a23-a24);
  a24=(a12*a13);
  a23=(a23+a24);
  a24=(a14*a11);
  a23=(a23-a24);
  a24=casadi_sq(a23);
  a21=(a21+a24);
  a21=(a21+a8);
  a21=sqrt(a21);
  a8=(a20/a21);
  a24=(a19*a8);
  a24=(a0*a24);
  if (res[0]!=0) res[0][0]=a24;
  a24=(a22/a21);
  a25=(a19*a24);
  a25=(a0*a25);
  if (res[0]!=0) res[0][1]=a25;
  a25=(a23/a21);
  a26=(a19*a25);
  a26=(a0*a26);
  if (res[0]!=0) res[0][2]=a26;
  a26=-1.;
  a17=casadi_sq(a17);
  a18=(a18-a17);
  a18=sqrt(a18);
  a26=(a26/a18);
  a10=(a10+a10);
  a18=(1./a4);
  a9=(a9/a4);
  a3=(a3/a4);
  a17=(a9*a3);
  a18=(a18-a17);
  a17=(a2*a18);
  a12=(a12/a4);
  a27=(a12*a3);
  a28=(a11*a27);
  a17=(a17-a28);
  a14=(a14/a4);
  a28=(a14*a3);
  a29=(a13*a28);
  a17=(a17-a29);
  a16=(a16/a4);
  a3=(a16*a3);
  a29=(a15*a3);
  a17=(a17-a29);
  a17=(a10*a17);
  a17=(a1*a17);
  a17=(a26*a17);
  a29=(a8*a17);
  a30=(a11*a18);
  a31=(a2*a27);
  a30=(a30+a31);
  a31=(a15*a28);
  a30=(a30-a31);
  a31=(a13*a3);
  a30=(a30+a31);
  a31=(a30/a21);
  a32=(a8/a21);
  a20=(a20+a20);
  a30=(a20*a30);
  a22=(a22+a22);
  a33=(a13*a18);
  a34=(a2*a28);
  a33=(a33+a34);
  a34=(a15*a27);
  a33=(a33+a34);
  a34=(a11*a3);
  a33=(a33-a34);
  a34=(a22*a33);
  a30=(a30+a34);
  a23=(a23+a23);
  a18=(a15*a18);
  a3=(a2*a3);
  a18=(a18+a3);
  a27=(a13*a27);
  a18=(a18-a27);
  a28=(a11*a28);
  a18=(a18+a28);
  a28=(a23*a18);
  a30=(a30+a28);
  a28=(a21+a21);
  a30=(a30/a28);
  a27=(a32*a30);
  a31=(a31-a27);
  a31=(a19*a31);
  a29=(a29+a31);
  a29=(a0*a29);
  if (res[1]!=0) res[1][0]=a29;
  a29=(1./a4);
  a5=(a5/a4);
  a31=(a12*a5);
  a29=(a29-a31);
  a31=(a11*a29);
  a27=(a9*a5);
  a3=(a2*a27);
  a31=(a31-a3);
  a3=(a14*a5);
  a34=(a13*a3);
  a31=(a31-a34);
  a5=(a16*a5);
  a34=(a15*a5);
  a31=(a31-a34);
  a31=(a10*a31);
  a31=(a1*a31);
  a31=(a26*a31);
  a34=(a8*a31);
  a35=(a13*a5);
  a36=(a11*a27);
  a37=(a2*a29);
  a36=(a36+a37);
  a37=(a15*a3);
  a36=(a36+a37);
  a35=(a35-a36);
  a36=(a35/a21);
  a35=(a20*a35);
  a37=(a2*a3);
  a38=(a13*a27);
  a37=(a37-a38);
  a38=(a15*a29);
  a37=(a37-a38);
  a38=(a11*a5);
  a37=(a37-a38);
  a38=(a22*a37);
  a35=(a35+a38);
  a5=(a2*a5);
  a27=(a15*a27);
  a5=(a5-a27);
  a29=(a13*a29);
  a5=(a5+a29);
  a3=(a11*a3);
  a5=(a5+a3);
  a3=(a23*a5);
  a35=(a35+a3);
  a35=(a35/a28);
  a3=(a32*a35);
  a36=(a36-a3);
  a36=(a19*a36);
  a34=(a34+a36);
  a34=(a0*a34);
  if (res[1]!=0) res[1][1]=a34;
  a34=(1./a4);
  a6=(a6/a4);
  a36=(a14*a6);
  a34=(a34-a36);
  a36=(a13*a34);
  a3=(a9*a6);
  a29=(a2*a3);
  a27=(a12*a6);
  a38=(a11*a27);
  a29=(a29+a38);
  a36=(a36-a29);
  a6=(a16*a6);
  a29=(a15*a6);
  a36=(a36-a29);
  a36=(a10*a36);
  a36=(a1*a36);
  a36=(a26*a36);
  a29=(a8*a36);
  a38=(a2*a27);
  a39=(a11*a3);
  a38=(a38-a39);
  a39=(a15*a34);
  a38=(a38+a39);
  a39=(a13*a6);
  a38=(a38+a39);
  a39=(a38/a21);
  a38=(a20*a38);
  a40=(a15*a27);
  a41=(a13*a3);
  a42=(a2*a34);
  a41=(a41+a42);
  a40=(a40-a41);
  a41=(a11*a6);
  a40=(a40-a41);
  a41=(a22*a40);
  a38=(a38+a41);
  a6=(a2*a6);
  a3=(a15*a3);
  a6=(a6-a3);
  a27=(a13*a27);
  a6=(a6-a27);
  a34=(a11*a34);
  a6=(a6-a34);
  a34=(a23*a6);
  a38=(a38+a34);
  a38=(a38/a28);
  a34=(a32*a38);
  a39=(a39-a34);
  a39=(a19*a39);
  a29=(a29+a39);
  a29=(a0*a29);
  if (res[1]!=0) res[1][2]=a29;
  a29=(1./a4);
  a7=(a7/a4);
  a16=(a16*a7);
  a29=(a29-a16);
  a16=(a15*a29);
  a9=(a9*a7);
  a4=(a2*a9);
  a12=(a12*a7);
  a39=(a11*a12);
  a4=(a4+a39);
  a14=(a14*a7);
  a7=(a13*a14);
  a4=(a4+a7);
  a16=(a16-a4);
  a10=(a10*a16);
  a1=(a1*a10);
  a26=(a26*a1);
  a8=(a8*a26);
  a1=(a2*a12);
  a10=(a11*a9);
  a1=(a1-a10);
  a10=(a15*a14);
  a1=(a1-a10);
  a10=(a13*a29);
  a1=(a1-a10);
  a10=(a1/a21);
  a20=(a20*a1);
  a1=(a2*a14);
  a16=(a13*a9);
  a1=(a1-a16);
  a16=(a15*a12);
  a1=(a1+a16);
  a16=(a11*a29);
  a1=(a1+a16);
  a22=(a22*a1);
  a20=(a20+a22);
  a11=(a11*a14);
  a15=(a15*a9);
  a2=(a2*a29);
  a15=(a15+a2);
  a13=(a13*a12);
  a15=(a15+a13);
  a11=(a11-a15);
  a23=(a23*a11);
  a20=(a20+a23);
  a20=(a20/a28);
  a32=(a32*a20);
  a10=(a10-a32);
  a10=(a19*a10);
  a8=(a8+a10);
  a8=(a0*a8);
  if (res[1]!=0) res[1][3]=a8;
  a8=(a24*a17);
  a33=(a33/a21);
  a10=(a24/a21);
  a32=(a10*a30);
  a33=(a33-a32);
  a33=(a19*a33);
  a8=(a8+a33);
  a8=(a0*a8);
  if (res[1]!=0) res[1][4]=a8;
  a8=(a24*a31);
  a37=(a37/a21);
  a33=(a10*a35);
  a37=(a37-a33);
  a37=(a19*a37);
  a8=(a8+a37);
  a8=(a0*a8);
  if (res[1]!=0) res[1][5]=a8;
  a8=(a24*a36);
  a40=(a40/a21);
  a37=(a10*a38);
  a40=(a40-a37);
  a40=(a19*a40);
  a8=(a8+a40);
  a8=(a0*a8);
  if (res[1]!=0) res[1][6]=a8;
  a24=(a24*a26);
  a1=(a1/a21);
  a10=(a10*a20);
  a1=(a1-a10);
  a1=(a19*a1);
  a24=(a24+a1);
  a24=(a0*a24);
  if (res[1]!=0) res[1][7]=a24;
  a17=(a25*a17);
  a18=(a18/a21);
  a24=(a25/a21);
  a30=(a24*a30);
  a18=(a18-a30);
  a18=(a19*a18);
  a17=(a17+a18);
  a17=(a0*a17);
  if (res[1]!=0) res[1][8]=a17;
  a31=(a25*a31);
  a5=(a5/a21);
  a35=(a24*a35);
  a5=(a5-a35);
  a5=(a19*a5);
  a31=(a31+a5);
  a31=(a0*a31);
  if (res[1]!=0) res[1][9]=a31;
  a36=(a25*a36);
  a6=(a6/a21);
  a38=(a24*a38);
  a6=(a6-a38);
  a6=(a19*a6);
  a36=(a36+a6);
  a36=(a0*a36);
  if (res[1]!=0) res[1][10]=a36;
  a25=(a25*a26);
  a11=(a11/a21);
  a24=(a24*a20);
  a11=(a11-a24);
  a19=(a19*a11);
  a25=(a25+a19);
  a0=(a0*a25);
  if (res[1]!=0) res[1][11]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int SRBD_constr_h_e_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int SRBD_constr_h_e_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int SRBD_constr_h_e_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void SRBD_constr_h_e_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int SRBD_constr_h_e_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void SRBD_constr_h_e_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void SRBD_constr_h_e_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void SRBD_constr_h_e_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int SRBD_constr_h_e_fun_jac_uxt_zt_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int SRBD_constr_h_e_fun_jac_uxt_zt_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real SRBD_constr_h_e_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* SRBD_constr_h_e_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* SRBD_constr_h_e_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* SRBD_constr_h_e_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* SRBD_constr_h_e_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int SRBD_constr_h_e_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
