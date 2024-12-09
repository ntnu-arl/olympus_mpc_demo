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
  #define CASADI_PREFIX(ID) fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_ ## ID
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
#define casadi_s4 CASADI_PREFIX(s4)
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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[19] = {10, 4, 0, 2, 4, 8, 12, 0, 1, 0, 3, 1, 2, 3, 4, 1, 2, 3, 4};
static const casadi_int casadi_s4[22] = {10, 10, 0, 1, 3, 5, 7, 9, 9, 9, 9, 9, 9, 0, 1, 2, 1, 2, 3, 4, 3, 4};

/* fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess:(i0[10],i1[4],i2[])->(o0[4],o1[10x4,12nz],o2[10x10,9nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a6, a7, a8, a9;
  a0=-4.7499999999999998e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=(a0*a1);
  a3=1.8999999999999999e+00;
  a2=(a2+a3);
  a3=1.;
  a4=-4.9399999999999999e+01;
  a5=9.0000000000000002e-01;
  a5=(a1-a5);
  a5=(a4*a5);
  a5=exp(a5);
  a6=(a3+a5);
  a2=(a2/a6);
  a7=-30.;
  a8=9.4999999999999996e-01;
  a8=(a1-a8);
  a8=(a7*a8);
  a8=exp(a8);
  a9=(a3+a8);
  a10=(1./a9);
  a3=(a3-a10);
  a11=-1.9399999999999999e+01;
  a12=(a11*a1);
  a13=2.1500000000000000e+01;
  a12=(a12+a13);
  a13=(a3*a12);
  a13=(a2+a13);
  a14=arg[0]? arg[0][1] : 0;
  a13=(a13-a14);
  if (res[0]!=0) res[0][0]=a13;
  a13=-8.1159999999999999e-01;
  a15=casadi_sq(a1);
  a16=casadi_sq(a15);
  a17=(a1*a16);
  a17=(a13*a17);
  a18=8.9614999999999991e+00;
  a19=casadi_sq(a1);
  a20=casadi_sq(a19);
  a20=(a18*a20);
  a17=(a17+a20);
  a20=-3.8647399999999998e+01;
  a21=casadi_sq(a1);
  a22=(a1*a21);
  a22=(a20*a22);
  a17=(a17+a22);
  a22=8.1128100000000003e+01;
  a23=casadi_sq(a1);
  a23=(a22*a23);
  a17=(a17+a23);
  a23=-8.3697000000000003e+01;
  a24=(a23*a1);
  a17=(a17+a24);
  a24=3.6456400000000002e+01;
  a17=(a17+a24);
  a24=arg[0]? arg[0][3] : 0;
  a17=(a17-a24);
  if (res[0]!=0) res[0][1]=a17;
  a17=2.9929000000000000e-01;
  a25=cos(a24);
  a25=(a17*a25);
  a26=1.4501633819223289e-01;
  a27=arg[0]? arg[0][4] : 0;
  a28=cos(a27);
  a28=(a26*a28);
  a29=9.8942926056253055e-01;
  a30=sin(a27);
  a30=(a29*a30);
  a28=(a28+a30);
  a30=(a25*a28);
  a31=1.7499999999999399e-01;
  a32=sin(a24);
  a32=(a31*a32);
  a30=(a30-a32);
  a32=1.7499999999999100e-01;
  a33=sin(a14);
  a33=(a32*a33);
  a30=(a30-a33);
  a33=2.9976999999999998e-01;
  a34=cos(a14);
  a34=(a33*a34);
  a35=1.5547417496984137e-01;
  a36=arg[0]? arg[0][2] : 0;
  a37=cos(a36);
  a37=(a35*a37);
  a38=9.8783995713751482e-01;
  a39=sin(a36);
  a39=(a38*a39);
  a37=(a37-a39);
  a39=(a34*a37);
  a30=(a30+a39);
  a39=sin(a24);
  a39=(a17*a39);
  a40=cos(a27);
  a40=(a29*a40);
  a41=sin(a27);
  a41=(a26*a41);
  a40=(a40-a41);
  a41=(a39*a40);
  a30=(a30-a41);
  a41=sin(a14);
  a41=(a33*a41);
  a42=cos(a36);
  a42=(a38*a42);
  a43=sin(a36);
  a43=(a35*a43);
  a42=(a42+a43);
  a43=(a41*a42);
  a30=(a30-a43);
  a43=8.9999999999999997e-02;
  a30=(a30-a43);
  if (res[0]!=0) res[0][2]=a30;
  a30=cos(a24);
  a30=(a31*a30);
  a43=cos(a14);
  a43=(a32*a43);
  a30=(a30-a43);
  a43=cos(a24);
  a43=(a17*a43);
  a44=cos(a27);
  a44=(a29*a44);
  a45=sin(a27);
  a45=(a26*a45);
  a44=(a44-a45);
  a45=(a43*a44);
  a30=(a30+a45);
  a45=cos(a14);
  a45=(a33*a45);
  a46=cos(a36);
  a46=(a38*a46);
  a47=sin(a36);
  a47=(a35*a47);
  a46=(a46+a47);
  a47=(a45*a46);
  a30=(a30-a47);
  a47=sin(a24);
  a47=(a17*a47);
  a48=cos(a27);
  a48=(a26*a48);
  a49=sin(a27);
  a49=(a29*a49);
  a48=(a48+a49);
  a49=(a47*a48);
  a30=(a30+a49);
  a49=sin(a14);
  a49=(a33*a49);
  a50=cos(a36);
  a50=(a35*a50);
  a51=sin(a36);
  a51=(a38*a51);
  a50=(a50-a51);
  a51=(a49*a50);
  a30=(a30-a51);
  if (res[0]!=0) res[0][3]=a30;
  a30=(a0/a6);
  a51=(a2/a6);
  a52=(a4*a5);
  a51=(a51*a52);
  a30=(a30-a51);
  a51=(a10/a9);
  a52=(a7*a8);
  a51=(a51*a52);
  a51=(a12*a51);
  a3=(a11*a3);
  a51=(a51+a3);
  a30=(a30+a51);
  if (res[1]!=0) res[1][0]=a30;
  a30=-1.;
  if (res[1]!=0) res[1][1]=a30;
  a51=(a15+a15);
  a3=(a1+a1);
  a51=(a51*a3);
  a51=(a1*a51);
  a16=(a16+a51);
  a16=(a13*a16);
  a51=(a19+a19);
  a3=(a1+a1);
  a51=(a51*a3);
  a51=(a18*a51);
  a16=(a16+a51);
  a51=(a1+a1);
  a51=(a1*a51);
  a21=(a21+a51);
  a21=(a20*a21);
  a16=(a16+a21);
  a21=(a1+a1);
  a21=(a22*a21);
  a16=(a16+a21);
  a16=(a16+a23);
  if (res[1]!=0) res[1][2]=a16;
  if (res[1]!=0) res[1][3]=a30;
  a30=cos(a14);
  a30=(a32*a30);
  a16=sin(a14);
  a16=(a33*a16);
  a16=(a37*a16);
  a30=(a30+a16);
  a16=cos(a14);
  a16=(a33*a16);
  a16=(a42*a16);
  a30=(a30+a16);
  a30=(-a30);
  if (res[1]!=0) res[1][4]=a30;
  a30=sin(a36);
  a30=(a35*a30);
  a16=cos(a36);
  a16=(a38*a16);
  a30=(a30+a16);
  a30=(a34*a30);
  a16=cos(a36);
  a16=(a35*a16);
  a23=sin(a36);
  a23=(a38*a23);
  a16=(a16-a23);
  a16=(a41*a16);
  a30=(a30+a16);
  a30=(-a30);
  if (res[1]!=0) res[1][5]=a30;
  a30=sin(a24);
  a30=(a17*a30);
  a30=(a28*a30);
  a16=cos(a24);
  a16=(a31*a16);
  a30=(a30+a16);
  a16=cos(a24);
  a16=(a17*a16);
  a16=(a40*a16);
  a30=(a30+a16);
  a30=(-a30);
  if (res[1]!=0) res[1][6]=a30;
  a30=cos(a27);
  a30=(a29*a30);
  a16=sin(a27);
  a16=(a26*a16);
  a30=(a30-a16);
  a30=(a25*a30);
  a16=sin(a27);
  a16=(a29*a16);
  a23=cos(a27);
  a23=(a26*a23);
  a16=(a16+a23);
  a16=(a39*a16);
  a30=(a30+a16);
  if (res[1]!=0) res[1][7]=a30;
  a30=sin(a14);
  a30=(a32*a30);
  a16=sin(a14);
  a16=(a33*a16);
  a16=(a46*a16);
  a30=(a30+a16);
  a16=cos(a14);
  a16=(a33*a16);
  a16=(a50*a16);
  a30=(a30-a16);
  if (res[1]!=0) res[1][8]=a30;
  a30=sin(a36);
  a30=(a35*a30);
  a16=cos(a36);
  a16=(a38*a16);
  a30=(a30+a16);
  a30=(a49*a30);
  a16=cos(a36);
  a16=(a35*a16);
  a23=sin(a36);
  a23=(a38*a23);
  a16=(a16-a23);
  a16=(a45*a16);
  a30=(a30-a16);
  if (res[1]!=0) res[1][9]=a30;
  a30=cos(a24);
  a30=(a17*a30);
  a30=(a48*a30);
  a16=sin(a24);
  a16=(a31*a16);
  a23=sin(a24);
  a23=(a17*a23);
  a23=(a44*a23);
  a16=(a16+a23);
  a30=(a30-a16);
  if (res[1]!=0) res[1][10]=a30;
  a30=cos(a27);
  a30=(a29*a30);
  a16=sin(a27);
  a16=(a26*a16);
  a30=(a30-a16);
  a30=(a47*a30);
  a16=sin(a27);
  a16=(a29*a16);
  a23=cos(a27);
  a23=(a26*a23);
  a16=(a16+a23);
  a16=(a43*a16);
  a30=(a30-a16);
  if (res[1]!=0) res[1][11]=a30;
  a30=2.;
  a16=arg[1]? arg[1][1] : 0;
  a22=(a22*a16);
  a22=(a30*a22);
  a20=(a20*a16);
  a23=(a1+a1);
  a23=(a20*a23);
  a22=(a22+a23);
  a23=(a1*a20);
  a23=(a30*a23);
  a21=(a1+a1);
  a21=(a21*a20);
  a23=(a23+a21);
  a22=(a22+a23);
  a19=(a19+a19);
  a18=(a18*a16);
  a19=(a19*a18);
  a19=(a30*a19);
  a23=(a1+a1);
  a21=(a1+a1);
  a21=(a21+a21);
  a18=(a18*a21);
  a23=(a23*a18);
  a19=(a19+a23);
  a22=(a22+a19);
  a13=(a13*a16);
  a16=(a15+a15);
  a19=(a1+a1);
  a16=(a16*a19);
  a16=(a13*a16);
  a22=(a22+a16);
  a15=(a15+a15);
  a16=(a1*a13);
  a23=(a15*a16);
  a30=(a30*a23);
  a1=(a1+a1);
  a19=(a19+a19);
  a16=(a16*a19);
  a15=(a15*a13);
  a16=(a16+a15);
  a1=(a1*a16);
  a30=(a30+a1);
  a22=(a22+a30);
  a30=arg[1]? arg[1][0] : 0;
  a1=(a10/a9);
  a16=(a7*a8);
  a1=(a1*a16);
  a15=(a30*a1);
  a15=(a11*a15);
  a22=(a22+a15);
  a10=(a10/a9);
  a12=(a12*a30);
  a15=(a10*a12);
  a15=(a15*a16);
  a11=(a11*a30);
  a11=(a10*a11);
  a1=(a1/a9);
  a10=(a10/a9);
  a10=(a10*a16);
  a1=(a1+a10);
  a12=(a12*a1);
  a11=(a11-a12);
  a8=(a8*a11);
  a15=(a15+a8);
  a7=(a7*a15);
  a22=(a22+a7);
  a7=(a2/a6);
  a15=(a7*a30);
  a8=(a4*a5);
  a15=(a15*a8);
  a11=(a0/a6);
  a2=(a2/a6);
  a2=(a2*a8);
  a11=(a11-a2);
  a11=(a11/a6);
  a7=(a7/a6);
  a7=(a7*a8);
  a11=(a11-a7);
  a11=(a30*a11);
  a5=(a5*a11);
  a15=(a15+a5);
  a4=(a4*a15);
  a22=(a22-a4);
  a30=(a30/a6);
  a30=(a30/a6);
  a30=(a30*a8);
  a0=(a0*a30);
  a22=(a22-a0);
  if (res[2]!=0) res[2][0]=a22;
  a22=arg[1]? arg[1][3] : 0;
  a46=(a46*a22);
  a46=(a33*a46);
  a0=cos(a14);
  a46=(a46*a0);
  a50=(a50*a22);
  a50=(a33*a50);
  a0=sin(a14);
  a50=(a50*a0);
  a46=(a46+a50);
  a50=(a32*a22);
  a0=cos(a14);
  a50=(a50*a0);
  a46=(a46+a50);
  a50=arg[1]? arg[1][2] : 0;
  a42=(a42*a50);
  a42=(a33*a42);
  a0=sin(a14);
  a42=(a42*a0);
  a46=(a46+a42);
  a37=(a37*a50);
  a37=(a33*a37);
  a42=cos(a14);
  a37=(a37*a42);
  a46=(a46-a37);
  a32=(a32*a50);
  a37=sin(a14);
  a32=(a32*a37);
  a46=(a46+a32);
  if (res[2]!=0) res[2][1]=a46;
  a46=cos(a36);
  a32=cos(a14);
  a32=(a33*a32);
  a32=(a22*a32);
  a37=(a38*a32);
  a46=(a46*a37);
  a37=sin(a36);
  a32=(a35*a32);
  a37=(a37*a32);
  a46=(a46+a37);
  a37=cos(a36);
  a32=sin(a14);
  a32=(a33*a32);
  a32=(a22*a32);
  a42=(a35*a32);
  a37=(a37*a42);
  a46=(a46+a37);
  a37=sin(a36);
  a32=(a38*a32);
  a37=(a37*a32);
  a46=(a46-a37);
  a37=cos(a36);
  a32=cos(a14);
  a32=(a33*a32);
  a32=(a50*a32);
  a42=(a35*a32);
  a37=(a37*a42);
  a46=(a46-a37);
  a37=sin(a36);
  a32=(a38*a32);
  a37=(a37*a32);
  a46=(a46+a37);
  a37=cos(a36);
  a32=sin(a14);
  a32=(a33*a32);
  a32=(a50*a32);
  a42=(a38*a32);
  a37=(a37*a42);
  a46=(a46+a37);
  a37=sin(a36);
  a32=(a35*a32);
  a37=(a37*a32);
  a46=(a46+a37);
  if (res[2]!=0) res[2][2]=a46;
  a46=sin(a14);
  a37=cos(a36);
  a37=(a35*a37);
  a32=sin(a36);
  a32=(a38*a32);
  a37=(a37-a32);
  a37=(a22*a37);
  a37=(a33*a37);
  a46=(a46*a37);
  a37=cos(a14);
  a32=sin(a36);
  a32=(a35*a32);
  a42=cos(a36);
  a42=(a38*a42);
  a32=(a32+a42);
  a32=(a22*a32);
  a32=(a33*a32);
  a37=(a37*a32);
  a46=(a46+a37);
  a37=cos(a14);
  a32=cos(a36);
  a32=(a35*a32);
  a42=sin(a36);
  a42=(a38*a42);
  a32=(a32-a42);
  a32=(a50*a32);
  a32=(a33*a32);
  a37=(a37*a32);
  a46=(a46-a37);
  a14=sin(a14);
  a37=sin(a36);
  a37=(a35*a37);
  a32=cos(a36);
  a32=(a38*a32);
  a37=(a37+a32);
  a37=(a50*a37);
  a33=(a33*a37);
  a14=(a14*a33);
  a46=(a46+a14);
  if (res[2]!=0) res[2][3]=a46;
  a49=(a49*a22);
  a46=(a35*a49);
  a14=cos(a36);
  a46=(a46*a14);
  a49=(a38*a49);
  a14=sin(a36);
  a49=(a49*a14);
  a46=(a46-a49);
  a45=(a45*a22);
  a49=(a35*a45);
  a14=sin(a36);
  a49=(a49*a14);
  a46=(a46+a49);
  a45=(a38*a45);
  a49=cos(a36);
  a45=(a45*a49);
  a46=(a46+a45);
  a41=(a41*a50);
  a45=(a35*a41);
  a49=sin(a36);
  a45=(a45*a49);
  a46=(a46+a45);
  a41=(a38*a41);
  a45=cos(a36);
  a41=(a41*a45);
  a46=(a46+a41);
  a34=(a34*a50);
  a38=(a38*a34);
  a41=sin(a36);
  a38=(a38*a41);
  a46=(a46+a38);
  a35=(a35*a34);
  a36=cos(a36);
  a35=(a35*a36);
  a46=(a46-a35);
  if (res[2]!=0) res[2][4]=a46;
  a40=(a40*a50);
  a40=(a17*a40);
  a46=sin(a24);
  a40=(a40*a46);
  a48=(a48*a22);
  a48=(a17*a48);
  a46=sin(a24);
  a48=(a48*a46);
  a44=(a44*a22);
  a44=(a17*a44);
  a46=cos(a24);
  a44=(a44*a46);
  a48=(a48+a44);
  a44=(a31*a22);
  a46=cos(a24);
  a44=(a44*a46);
  a48=(a48+a44);
  a40=(a40-a48);
  a31=(a31*a50);
  a48=sin(a24);
  a31=(a31*a48);
  a40=(a40+a31);
  a28=(a28*a50);
  a28=(a17*a28);
  a31=cos(a24);
  a28=(a28*a31);
  a40=(a40-a28);
  if (res[2]!=0) res[2][5]=a40;
  a40=cos(a27);
  a28=cos(a24);
  a28=(a17*a28);
  a28=(a22*a28);
  a31=(a29*a28);
  a40=(a40*a31);
  a31=sin(a27);
  a28=(a26*a28);
  a31=(a31*a28);
  a40=(a40-a31);
  a31=cos(a27);
  a28=sin(a24);
  a28=(a17*a28);
  a28=(a22*a28);
  a48=(a26*a28);
  a31=(a31*a48);
  a40=(a40+a31);
  a31=sin(a27);
  a28=(a29*a28);
  a31=(a31*a28);
  a40=(a40+a31);
  a31=cos(a27);
  a28=cos(a24);
  a28=(a17*a28);
  a28=(a50*a28);
  a48=(a26*a28);
  a31=(a31*a48);
  a40=(a40+a31);
  a31=sin(a27);
  a28=(a29*a28);
  a31=(a31*a28);
  a40=(a40+a31);
  a31=cos(a27);
  a28=sin(a24);
  a28=(a17*a28);
  a28=(a50*a28);
  a48=(a29*a28);
  a31=(a31*a48);
  a40=(a40-a31);
  a31=sin(a27);
  a28=(a26*a28);
  a31=(a31*a28);
  a40=(a40+a31);
  if (res[2]!=0) res[2][6]=a40;
  a40=cos(a24);
  a31=cos(a27);
  a31=(a29*a31);
  a28=sin(a27);
  a28=(a26*a28);
  a31=(a31-a28);
  a31=(a22*a31);
  a31=(a17*a31);
  a40=(a40*a31);
  a31=sin(a24);
  a28=sin(a27);
  a28=(a29*a28);
  a48=cos(a27);
  a48=(a26*a48);
  a28=(a28+a48);
  a28=(a22*a28);
  a28=(a17*a28);
  a31=(a31*a28);
  a40=(a40+a31);
  a31=cos(a24);
  a28=sin(a27);
  a28=(a29*a28);
  a48=cos(a27);
  a48=(a26*a48);
  a28=(a28+a48);
  a28=(a50*a28);
  a28=(a17*a28);
  a31=(a31*a28);
  a40=(a40+a31);
  a24=sin(a24);
  a31=cos(a27);
  a31=(a29*a31);
  a28=sin(a27);
  a28=(a26*a28);
  a31=(a31-a28);
  a31=(a50*a31);
  a17=(a17*a31);
  a24=(a24*a17);
  a40=(a40-a24);
  if (res[2]!=0) res[2][7]=a40;
  a43=(a43*a22);
  a40=(a26*a43);
  a24=sin(a27);
  a40=(a40*a24);
  a47=(a47*a22);
  a22=(a29*a47);
  a24=sin(a27);
  a22=(a22*a24);
  a47=(a26*a47);
  a24=cos(a27);
  a47=(a47*a24);
  a22=(a22+a47);
  a40=(a40-a22);
  a43=(a29*a43);
  a22=cos(a27);
  a43=(a43*a22);
  a40=(a40-a43);
  a39=(a39*a50);
  a43=(a26*a39);
  a22=sin(a27);
  a43=(a43*a22);
  a40=(a40-a43);
  a39=(a29*a39);
  a43=cos(a27);
  a39=(a39*a43);
  a40=(a40+a39);
  a25=(a25*a50);
  a29=(a29*a25);
  a50=sin(a27);
  a29=(a29*a50);
  a40=(a40-a29);
  a26=(a26*a25);
  a27=cos(a27);
  a26=(a26*a27);
  a40=(a40-a26);
  if (res[2]!=0) res[2][8]=a40;
  return 0;
}

CASADI_SYMBOL_EXPORT int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fr_leg_torque_constr_h_e_fun_jac_uxt_zt_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
