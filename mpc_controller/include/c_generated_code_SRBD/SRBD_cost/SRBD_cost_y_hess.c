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
  #define CASADI_PREFIX(ID) SRBD_cost_y_hess_ ## ID
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
#define casadi_s5 CASADI_PREFIX(s5)
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
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s4[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s5[29] = {10, 10, 0, 0, 0, 0, 4, 8, 12, 16, 16, 16, 16, 3, 4, 5, 6, 3, 4, 5, 6, 3, 4, 5, 6, 3, 4, 5, 6};

/* SRBD_cost_y_hess:(i0[7],i1[3],i2[],i3[9],i4[4])->(o0[10x10,16nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92;
  a0=arg[4]? arg[4][3] : 0;
  a1=arg[3]? arg[3][2] : 0;
  a2=-1.;
  a3=1.;
  a4=2.;
  a5=arg[4]? arg[4][0] : 0;
  a6=arg[0]? arg[0][0] : 0;
  a7=casadi_sq(a6);
  a8=arg[0]? arg[0][1] : 0;
  a9=casadi_sq(a8);
  a7=(a7+a9);
  a9=arg[0]? arg[0][2] : 0;
  a10=casadi_sq(a9);
  a7=(a7+a10);
  a10=arg[0]? arg[0][3] : 0;
  a11=casadi_sq(a10);
  a7=(a7+a11);
  a11=1.0000000000000001e-09;
  a7=(a7+a11);
  a7=sqrt(a7);
  a12=(a6/a7);
  a13=(a5*a12);
  a14=arg[4]? arg[4][1] : 0;
  a15=(a8/a7);
  a16=(a14*a15);
  a13=(a13+a16);
  a16=arg[4]? arg[4][2] : 0;
  a17=(a9/a7);
  a18=(a16*a17);
  a13=(a13+a18);
  a18=(a10/a7);
  a19=(a0*a18);
  a13=(a13+a19);
  a19=casadi_sq(a13);
  a19=(a4*a19);
  a19=(a19-a3);
  a20=casadi_sq(a19);
  a20=(a3-a20);
  a20=sqrt(a20);
  a20=(a2/a20);
  a21=(a13+a13);
  a22=(1./a7);
  a23=(a12/a7);
  a24=(a6/a7);
  a25=(a23*a24);
  a22=(a22-a25);
  a25=(a5*a22);
  a26=(a15/a7);
  a27=(a26*a24);
  a28=(a14*a27);
  a25=(a25-a28);
  a28=(a17/a7);
  a29=(a28*a24);
  a30=(a16*a29);
  a25=(a25-a30);
  a30=(a18/a7);
  a31=(a30*a24);
  a32=(a0*a31);
  a25=(a25-a32);
  a32=(a21*a25);
  a32=(a4*a32);
  a33=(a20*a32);
  a34=(a1*a33);
  a35=(a12*a14);
  a36=(a15*a5);
  a35=(a35-a36);
  a36=(a17*a0);
  a35=(a35+a36);
  a36=(a18*a16);
  a35=(a35-a36);
  a36=casadi_sq(a35);
  a37=(a12*a16);
  a38=(a17*a5);
  a37=(a37-a38);
  a38=(a15*a0);
  a37=(a37-a38);
  a38=(a18*a14);
  a37=(a37+a38);
  a38=casadi_sq(a37);
  a36=(a36+a38);
  a38=(a12*a0);
  a39=(a5*a18);
  a38=(a38-a39);
  a39=(a15*a16);
  a38=(a38+a39);
  a39=(a17*a14);
  a38=(a38-a39);
  a39=casadi_sq(a38);
  a36=(a36+a39);
  a36=(a36+a11);
  a36=sqrt(a36);
  a11=(a34/a36);
  a39=acos(a19);
  a40=(a39*a1);
  a41=(a40/a36);
  a42=(a41/a36);
  a43=(a35+a35);
  a44=(a14*a22);
  a45=(a5*a27);
  a44=(a44+a45);
  a45=(a0*a29);
  a44=(a44-a45);
  a45=(a16*a31);
  a44=(a44+a45);
  a45=(a43*a44);
  a46=(a37+a37);
  a47=(a16*a22);
  a48=(a5*a29);
  a47=(a47+a48);
  a48=(a0*a27);
  a47=(a47+a48);
  a48=(a14*a31);
  a47=(a47-a48);
  a48=(a46*a47);
  a45=(a45+a48);
  a48=(a38+a38);
  a49=(a0*a22);
  a50=(a5*a31);
  a49=(a49+a50);
  a50=(a16*a27);
  a49=(a49-a50);
  a50=(a14*a29);
  a49=(a49+a50);
  a50=(a48*a49);
  a45=(a45+a50);
  a50=(a36+a36);
  a45=(a45/a50);
  a51=(a42*a45);
  a11=(a11-a51);
  a51=(a38/a36);
  a52=(a51/a36);
  a53=(a52*a40);
  a54=(a37/a36);
  a55=(a54/a36);
  a56=arg[3]? arg[3][1] : 0;
  a57=(a39*a56);
  a58=(a55*a57);
  a53=(a53+a58);
  a58=(a35/a36);
  a59=(a58/a36);
  a60=arg[3]? arg[3][0] : 0;
  a39=(a39*a60);
  a61=(a59*a39);
  a53=(a53+a61);
  a61=(a36+a36);
  a53=(a53/a61);
  a62=(a49+a49);
  a62=(a53*a62);
  a38=(a38+a38);
  a49=(a49/a36);
  a63=(a51/a36);
  a64=(a63*a45);
  a49=(a49-a64);
  a64=(a49/a36);
  a65=(a52/a36);
  a66=(a65*a45);
  a64=(a64-a66);
  a64=(a40*a64);
  a34=(a52*a34);
  a64=(a64+a34);
  a34=(a47/a36);
  a66=(a54/a36);
  a67=(a66*a45);
  a34=(a34-a67);
  a67=(a34/a36);
  a68=(a55/a36);
  a69=(a68*a45);
  a67=(a67-a69);
  a67=(a57*a67);
  a69=(a56*a33);
  a70=(a55*a69);
  a67=(a67+a70);
  a64=(a64+a67);
  a67=(a44/a36);
  a70=(a58/a36);
  a71=(a70*a45);
  a67=(a67-a71);
  a71=(a67/a36);
  a72=(a59/a36);
  a73=(a72*a45);
  a71=(a71-a73);
  a71=(a39*a71);
  a33=(a60*a33);
  a73=(a59*a33);
  a71=(a71+a73);
  a64=(a64+a71);
  a64=(a64/a61);
  a71=(a53/a61);
  a73=(a45+a45);
  a73=(a71*a73);
  a64=(a64-a73);
  a73=(a38*a64);
  a62=(a62+a73);
  a11=(a11-a62);
  a62=(a0*a11);
  a69=(a69/a36);
  a73=(a57/a36);
  a74=(a73/a36);
  a75=(a74*a45);
  a69=(a69-a75);
  a47=(a47+a47);
  a47=(a53*a47);
  a37=(a37+a37);
  a75=(a37*a64);
  a47=(a47+a75);
  a69=(a69-a47);
  a47=(a16*a69);
  a62=(a62+a47);
  a33=(a33/a36);
  a47=(a39/a36);
  a75=(a47/a36);
  a45=(a75*a45);
  a33=(a33-a45);
  a44=(a44+a44);
  a44=(a53*a44);
  a35=(a35+a35);
  a64=(a35*a64);
  a44=(a44+a64);
  a33=(a33-a44);
  a44=(a14*a33);
  a62=(a62+a44);
  a44=casadi_sq(a19);
  a3=(a3-a44);
  a3=sqrt(a3);
  a2=(a2/a3);
  a51=(a51*a1);
  a54=(a54*a56);
  a51=(a51+a54);
  a58=(a58*a60);
  a51=(a51+a58);
  a58=(a2*a51);
  a58=(a4*a58);
  a25=(a25+a25);
  a25=(a58*a25);
  a13=(a13+a13);
  a54=(a2/a3);
  a19=(a19+a19);
  a32=(a19*a32);
  a3=(a3+a3);
  a32=(a32/a3);
  a32=(a54*a32);
  a32=(a51*a32);
  a49=(a1*a49);
  a34=(a56*a34);
  a49=(a49+a34);
  a67=(a60*a67);
  a49=(a49+a67);
  a49=(a2*a49);
  a32=(a32+a49);
  a32=(a4*a32);
  a32=(a13*a32);
  a25=(a25+a32);
  a32=(a5*a25);
  a62=(a62+a32);
  a32=(a62/a7);
  a49=(a38*a53);
  a41=(a41-a49);
  a49=(a0*a41);
  a67=(a37*a53);
  a73=(a73-a67);
  a67=(a16*a73);
  a49=(a49+a67);
  a67=(a35*a53);
  a47=(a47-a67);
  a67=(a14*a47);
  a49=(a49+a67);
  a67=(a13*a58);
  a34=(a5*a67);
  a49=(a49+a34);
  a34=(a49/a7);
  a34=(a34/a7);
  a44=(a34*a24);
  a32=(a32-a44);
  a18=(a18/a7);
  a44=(a14*a73);
  a64=(a5*a41);
  a44=(a44-a64);
  a64=(a16*a47);
  a44=(a44-a64);
  a64=(a0*a67);
  a44=(a44+a64);
  a64=(a18*a44);
  a17=(a17/a7);
  a45=(a0*a47);
  a76=(a14*a41);
  a77=(a5*a73);
  a76=(a76+a77);
  a45=(a45-a76);
  a76=(a16*a67);
  a45=(a45+a76);
  a76=(a17*a45);
  a64=(a64+a76);
  a15=(a15/a7);
  a41=(a16*a41);
  a73=(a0*a73);
  a41=(a41-a73);
  a47=(a5*a47);
  a41=(a41-a47);
  a67=(a14*a67);
  a41=(a41+a67);
  a67=(a15*a41);
  a64=(a64+a67);
  a12=(a12/a7);
  a67=(a12*a49);
  a64=(a64+a67);
  a67=(a7+a7);
  a64=(a64/a67);
  a47=(a4*a64);
  a6=(a6+a6);
  a73=(a14*a69);
  a76=(a5*a11);
  a73=(a73-a76);
  a76=(a16*a33);
  a73=(a73-a76);
  a76=(a0*a25);
  a73=(a73+a76);
  a76=(a18*a73);
  a31=(a31/a7);
  a77=(a18/a7);
  a78=(a77*a24);
  a31=(a31+a78);
  a31=(a44*a31);
  a76=(a76-a31);
  a31=(a0*a33);
  a78=(a14*a11);
  a79=(a5*a69);
  a78=(a78+a79);
  a31=(a31-a78);
  a78=(a16*a25);
  a31=(a31+a78);
  a78=(a17*a31);
  a29=(a29/a7);
  a79=(a17/a7);
  a80=(a79*a24);
  a29=(a29+a80);
  a29=(a45*a29);
  a78=(a78-a29);
  a76=(a76+a78);
  a11=(a16*a11);
  a69=(a0*a69);
  a11=(a11-a69);
  a33=(a5*a33);
  a11=(a11-a33);
  a25=(a14*a25);
  a11=(a11+a25);
  a25=(a15*a11);
  a27=(a27/a7);
  a33=(a15/a7);
  a69=(a33*a24);
  a27=(a27+a69);
  a27=(a41*a27);
  a25=(a25-a27);
  a76=(a76+a25);
  a22=(a22/a7);
  a25=(a12/a7);
  a27=(a25*a24);
  a22=(a22-a27);
  a22=(a49*a22);
  a62=(a12*a62);
  a22=(a22+a62);
  a76=(a76+a22);
  a76=(a76/a67);
  a22=(a64/a67);
  a62=(a24+a24);
  a62=(a22*a62);
  a76=(a76-a62);
  a62=(a6*a76);
  a47=(a47+a62);
  a32=(a32-a47);
  if (res[0]!=0) res[0][0]=a32;
  a11=(a11/a7);
  a32=(a41/a7);
  a32=(a32/a7);
  a47=(a32*a24);
  a11=(a11-a47);
  a47=(a8+a8);
  a62=(a47*a76);
  a11=(a11-a62);
  if (res[0]!=0) res[0][1]=a11;
  a31=(a31/a7);
  a11=(a45/a7);
  a11=(a11/a7);
  a62=(a11*a24);
  a31=(a31-a62);
  a62=(a9+a9);
  a27=(a62*a76);
  a31=(a31-a27);
  if (res[0]!=0) res[0][2]=a31;
  a73=(a73/a7);
  a31=(a44/a7);
  a31=(a31/a7);
  a24=(a31*a24);
  a73=(a73-a24);
  a24=(a10+a10);
  a76=(a24*a76);
  a73=(a73-a76);
  if (res[0]!=0) res[0][3]=a73;
  a73=(1./a7);
  a8=(a8/a7);
  a76=(a26*a8);
  a73=(a73-a76);
  a76=(a14*a73);
  a27=(a23*a8);
  a69=(a5*a27);
  a76=(a76-a69);
  a69=(a28*a8);
  a78=(a16*a69);
  a76=(a76-a78);
  a78=(a30*a8);
  a29=(a0*a78);
  a76=(a76-a29);
  a29=(a21*a76);
  a29=(a4*a29);
  a80=(a20*a29);
  a81=(a1*a80);
  a82=(a81/a36);
  a83=(a16*a78);
  a84=(a14*a27);
  a85=(a5*a73);
  a84=(a84+a85);
  a85=(a0*a69);
  a84=(a84+a85);
  a83=(a83-a84);
  a84=(a43*a83);
  a85=(a5*a69);
  a86=(a16*a27);
  a85=(a85-a86);
  a86=(a0*a73);
  a85=(a85-a86);
  a86=(a14*a78);
  a85=(a85-a86);
  a86=(a46*a85);
  a84=(a84+a86);
  a86=(a5*a78);
  a87=(a0*a27);
  a86=(a86-a87);
  a87=(a16*a73);
  a86=(a86+a87);
  a87=(a14*a69);
  a86=(a86+a87);
  a87=(a48*a86);
  a84=(a84+a87);
  a84=(a84/a50);
  a87=(a42*a84);
  a82=(a82-a87);
  a87=(a86+a86);
  a87=(a53*a87);
  a86=(a86/a36);
  a88=(a63*a84);
  a86=(a86-a88);
  a88=(a86/a36);
  a89=(a65*a84);
  a88=(a88-a89);
  a88=(a40*a88);
  a81=(a52*a81);
  a88=(a88+a81);
  a81=(a85/a36);
  a89=(a66*a84);
  a81=(a81-a89);
  a89=(a81/a36);
  a90=(a68*a84);
  a89=(a89-a90);
  a89=(a57*a89);
  a90=(a56*a80);
  a91=(a55*a90);
  a89=(a89+a91);
  a88=(a88+a89);
  a89=(a83/a36);
  a91=(a70*a84);
  a89=(a89-a91);
  a91=(a89/a36);
  a92=(a72*a84);
  a91=(a91-a92);
  a91=(a39*a91);
  a80=(a60*a80);
  a92=(a59*a80);
  a91=(a91+a92);
  a88=(a88+a91);
  a88=(a88/a61);
  a91=(a84+a84);
  a91=(a71*a91);
  a88=(a88-a91);
  a91=(a38*a88);
  a87=(a87+a91);
  a82=(a82-a87);
  a87=(a0*a82);
  a90=(a90/a36);
  a91=(a74*a84);
  a90=(a90-a91);
  a85=(a85+a85);
  a85=(a53*a85);
  a91=(a37*a88);
  a85=(a85+a91);
  a90=(a90-a85);
  a85=(a16*a90);
  a87=(a87+a85);
  a80=(a80/a36);
  a84=(a75*a84);
  a80=(a80-a84);
  a83=(a83+a83);
  a83=(a53*a83);
  a88=(a35*a88);
  a83=(a83+a88);
  a80=(a80-a83);
  a83=(a14*a80);
  a87=(a87+a83);
  a76=(a76+a76);
  a76=(a58*a76);
  a29=(a19*a29);
  a29=(a29/a3);
  a29=(a54*a29);
  a29=(a51*a29);
  a86=(a1*a86);
  a81=(a56*a81);
  a86=(a86+a81);
  a89=(a60*a89);
  a86=(a86+a89);
  a86=(a2*a86);
  a29=(a29+a86);
  a29=(a4*a29);
  a29=(a13*a29);
  a76=(a76+a29);
  a29=(a5*a76);
  a87=(a87+a29);
  a29=(a87/a7);
  a86=(a34*a8);
  a29=(a29-a86);
  a86=(a14*a90);
  a89=(a5*a82);
  a86=(a86-a89);
  a89=(a16*a80);
  a86=(a86-a89);
  a89=(a0*a76);
  a86=(a86+a89);
  a89=(a18*a86);
  a78=(a78/a7);
  a81=(a77*a8);
  a78=(a78+a81);
  a78=(a44*a78);
  a89=(a89-a78);
  a78=(a0*a80);
  a81=(a14*a82);
  a83=(a5*a90);
  a81=(a81+a83);
  a78=(a78-a81);
  a81=(a16*a76);
  a78=(a78+a81);
  a81=(a17*a78);
  a69=(a69/a7);
  a83=(a79*a8);
  a69=(a69+a83);
  a69=(a45*a69);
  a81=(a81-a69);
  a89=(a89+a81);
  a73=(a73/a7);
  a81=(a33*a8);
  a73=(a73-a81);
  a73=(a41*a73);
  a82=(a16*a82);
  a90=(a0*a90);
  a82=(a82-a90);
  a80=(a5*a80);
  a82=(a82-a80);
  a76=(a14*a76);
  a82=(a82+a76);
  a76=(a15*a82);
  a73=(a73+a76);
  a89=(a89+a73);
  a87=(a12*a87);
  a27=(a27/a7);
  a73=(a25*a8);
  a27=(a27+a73);
  a27=(a49*a27);
  a87=(a87-a27);
  a89=(a89+a87);
  a89=(a89/a67);
  a87=(a8+a8);
  a87=(a22*a87);
  a89=(a89-a87);
  a87=(a6*a89);
  a29=(a29-a87);
  if (res[0]!=0) res[0][4]=a29;
  a82=(a82/a7);
  a29=(a32*a8);
  a82=(a82-a29);
  a29=(a4*a64);
  a87=(a47*a89);
  a29=(a29+a87);
  a82=(a82-a29);
  if (res[0]!=0) res[0][5]=a82;
  a78=(a78/a7);
  a82=(a11*a8);
  a78=(a78-a82);
  a82=(a62*a89);
  a78=(a78-a82);
  if (res[0]!=0) res[0][6]=a78;
  a86=(a86/a7);
  a8=(a31*a8);
  a86=(a86-a8);
  a89=(a24*a89);
  a86=(a86-a89);
  if (res[0]!=0) res[0][7]=a86;
  a86=(1./a7);
  a9=(a9/a7);
  a89=(a28*a9);
  a86=(a86-a89);
  a89=(a16*a86);
  a8=(a23*a9);
  a78=(a5*a8);
  a82=(a26*a9);
  a29=(a14*a82);
  a78=(a78+a29);
  a89=(a89-a78);
  a78=(a30*a9);
  a29=(a0*a78);
  a89=(a89-a29);
  a29=(a21*a89);
  a29=(a4*a29);
  a87=(a20*a29);
  a27=(a1*a87);
  a73=(a27/a36);
  a76=(a5*a82);
  a80=(a14*a8);
  a76=(a76-a80);
  a80=(a0*a86);
  a76=(a76+a80);
  a80=(a16*a78);
  a76=(a76+a80);
  a80=(a43*a76);
  a90=(a0*a82);
  a81=(a16*a8);
  a69=(a5*a86);
  a81=(a81+a69);
  a90=(a90-a81);
  a81=(a14*a78);
  a90=(a90-a81);
  a81=(a46*a90);
  a80=(a80+a81);
  a81=(a5*a78);
  a69=(a0*a8);
  a81=(a81-a69);
  a69=(a16*a82);
  a81=(a81-a69);
  a69=(a14*a86);
  a81=(a81-a69);
  a69=(a48*a81);
  a80=(a80+a69);
  a80=(a80/a50);
  a69=(a42*a80);
  a73=(a73-a69);
  a69=(a81+a81);
  a69=(a53*a69);
  a81=(a81/a36);
  a83=(a63*a80);
  a81=(a81-a83);
  a83=(a81/a36);
  a88=(a65*a80);
  a83=(a83-a88);
  a83=(a40*a83);
  a27=(a52*a27);
  a83=(a83+a27);
  a27=(a90/a36);
  a88=(a66*a80);
  a27=(a27-a88);
  a88=(a27/a36);
  a84=(a68*a80);
  a88=(a88-a84);
  a88=(a57*a88);
  a84=(a56*a87);
  a85=(a55*a84);
  a88=(a88+a85);
  a83=(a83+a88);
  a88=(a76/a36);
  a85=(a70*a80);
  a88=(a88-a85);
  a85=(a88/a36);
  a91=(a72*a80);
  a85=(a85-a91);
  a85=(a39*a85);
  a87=(a60*a87);
  a91=(a59*a87);
  a85=(a85+a91);
  a83=(a83+a85);
  a83=(a83/a61);
  a85=(a80+a80);
  a85=(a71*a85);
  a83=(a83-a85);
  a85=(a38*a83);
  a69=(a69+a85);
  a73=(a73-a69);
  a69=(a0*a73);
  a84=(a84/a36);
  a85=(a74*a80);
  a84=(a84-a85);
  a90=(a90+a90);
  a90=(a53*a90);
  a85=(a37*a83);
  a90=(a90+a85);
  a84=(a84-a90);
  a90=(a16*a84);
  a69=(a69+a90);
  a87=(a87/a36);
  a80=(a75*a80);
  a87=(a87-a80);
  a76=(a76+a76);
  a76=(a53*a76);
  a83=(a35*a83);
  a76=(a76+a83);
  a87=(a87-a76);
  a76=(a14*a87);
  a69=(a69+a76);
  a89=(a89+a89);
  a89=(a58*a89);
  a29=(a19*a29);
  a29=(a29/a3);
  a29=(a54*a29);
  a29=(a51*a29);
  a81=(a1*a81);
  a27=(a56*a27);
  a81=(a81+a27);
  a88=(a60*a88);
  a81=(a81+a88);
  a81=(a2*a81);
  a29=(a29+a81);
  a29=(a4*a29);
  a29=(a13*a29);
  a89=(a89+a29);
  a29=(a5*a89);
  a69=(a69+a29);
  a29=(a69/a7);
  a81=(a34*a9);
  a29=(a29-a81);
  a81=(a14*a84);
  a88=(a5*a73);
  a81=(a81-a88);
  a88=(a16*a87);
  a81=(a81-a88);
  a88=(a0*a89);
  a81=(a81+a88);
  a88=(a18*a81);
  a78=(a78/a7);
  a27=(a77*a9);
  a78=(a78+a27);
  a78=(a44*a78);
  a88=(a88-a78);
  a86=(a86/a7);
  a78=(a79*a9);
  a86=(a86-a78);
  a86=(a45*a86);
  a78=(a0*a87);
  a27=(a14*a73);
  a76=(a5*a84);
  a27=(a27+a76);
  a78=(a78-a27);
  a27=(a16*a89);
  a78=(a78+a27);
  a27=(a17*a78);
  a86=(a86+a27);
  a88=(a88+a86);
  a73=(a16*a73);
  a84=(a0*a84);
  a73=(a73-a84);
  a87=(a5*a87);
  a73=(a73-a87);
  a89=(a14*a89);
  a73=(a73+a89);
  a89=(a15*a73);
  a82=(a82/a7);
  a87=(a33*a9);
  a82=(a82+a87);
  a82=(a41*a82);
  a89=(a89-a82);
  a88=(a88+a89);
  a69=(a12*a69);
  a8=(a8/a7);
  a89=(a25*a9);
  a8=(a8+a89);
  a8=(a49*a8);
  a69=(a69-a8);
  a88=(a88+a69);
  a88=(a88/a67);
  a69=(a9+a9);
  a69=(a22*a69);
  a88=(a88-a69);
  a69=(a6*a88);
  a29=(a29-a69);
  if (res[0]!=0) res[0][8]=a29;
  a73=(a73/a7);
  a29=(a32*a9);
  a73=(a73-a29);
  a29=(a47*a88);
  a73=(a73-a29);
  if (res[0]!=0) res[0][9]=a73;
  a78=(a78/a7);
  a73=(a11*a9);
  a78=(a78-a73);
  a73=(a4*a64);
  a29=(a62*a88);
  a73=(a73+a29);
  a78=(a78-a73);
  if (res[0]!=0) res[0][10]=a78;
  a81=(a81/a7);
  a9=(a31*a9);
  a81=(a81-a9);
  a88=(a24*a88);
  a81=(a81-a88);
  if (res[0]!=0) res[0][11]=a81;
  a81=(1./a7);
  a10=(a10/a7);
  a30=(a30*a10);
  a81=(a81-a30);
  a30=(a0*a81);
  a23=(a23*a10);
  a88=(a5*a23);
  a26=(a26*a10);
  a9=(a14*a26);
  a88=(a88+a9);
  a28=(a28*a10);
  a9=(a16*a28);
  a88=(a88+a9);
  a30=(a30-a88);
  a21=(a21*a30);
  a21=(a4*a21);
  a20=(a20*a21);
  a88=(a1*a20);
  a9=(a88/a36);
  a78=(a5*a26);
  a73=(a14*a23);
  a78=(a78-a73);
  a73=(a0*a28);
  a78=(a78-a73);
  a73=(a16*a81);
  a78=(a78-a73);
  a43=(a43*a78);
  a73=(a5*a28);
  a29=(a16*a23);
  a73=(a73-a29);
  a29=(a0*a26);
  a73=(a73+a29);
  a29=(a14*a81);
  a73=(a73+a29);
  a46=(a46*a73);
  a43=(a43+a46);
  a46=(a14*a28);
  a29=(a0*a23);
  a69=(a5*a81);
  a29=(a29+a69);
  a69=(a16*a26);
  a29=(a29+a69);
  a46=(a46-a29);
  a48=(a48*a46);
  a43=(a43+a48);
  a43=(a43/a50);
  a42=(a42*a43);
  a9=(a9-a42);
  a42=(a46+a46);
  a42=(a53*a42);
  a46=(a46/a36);
  a63=(a63*a43);
  a46=(a46-a63);
  a63=(a46/a36);
  a65=(a65*a43);
  a63=(a63-a65);
  a40=(a40*a63);
  a52=(a52*a88);
  a40=(a40+a52);
  a52=(a73/a36);
  a66=(a66*a43);
  a52=(a52-a66);
  a66=(a52/a36);
  a68=(a68*a43);
  a66=(a66-a68);
  a57=(a57*a66);
  a66=(a56*a20);
  a55=(a55*a66);
  a57=(a57+a55);
  a40=(a40+a57);
  a57=(a78/a36);
  a70=(a70*a43);
  a57=(a57-a70);
  a70=(a57/a36);
  a72=(a72*a43);
  a70=(a70-a72);
  a39=(a39*a70);
  a20=(a60*a20);
  a59=(a59*a20);
  a39=(a39+a59);
  a40=(a40+a39);
  a40=(a40/a61);
  a61=(a43+a43);
  a71=(a71*a61);
  a40=(a40-a71);
  a38=(a38*a40);
  a42=(a42+a38);
  a9=(a9-a42);
  a42=(a0*a9);
  a66=(a66/a36);
  a74=(a74*a43);
  a66=(a66-a74);
  a73=(a73+a73);
  a73=(a53*a73);
  a37=(a37*a40);
  a73=(a73+a37);
  a66=(a66-a73);
  a73=(a16*a66);
  a42=(a42+a73);
  a20=(a20/a36);
  a75=(a75*a43);
  a20=(a20-a75);
  a78=(a78+a78);
  a53=(a53*a78);
  a35=(a35*a40);
  a53=(a53+a35);
  a20=(a20-a53);
  a53=(a14*a20);
  a42=(a42+a53);
  a30=(a30+a30);
  a58=(a58*a30);
  a19=(a19*a21);
  a19=(a19/a3);
  a54=(a54*a19);
  a51=(a51*a54);
  a1=(a1*a46);
  a56=(a56*a52);
  a1=(a1+a56);
  a60=(a60*a57);
  a1=(a1+a60);
  a2=(a2*a1);
  a51=(a51+a2);
  a51=(a4*a51);
  a13=(a13*a51);
  a58=(a58+a13);
  a13=(a5*a58);
  a42=(a42+a13);
  a13=(a42/a7);
  a34=(a34*a10);
  a13=(a13-a34);
  a81=(a81/a7);
  a77=(a77*a10);
  a81=(a81-a77);
  a44=(a44*a81);
  a81=(a14*a66);
  a77=(a5*a9);
  a81=(a81-a77);
  a77=(a16*a20);
  a81=(a81-a77);
  a77=(a0*a58);
  a81=(a81+a77);
  a18=(a18*a81);
  a44=(a44+a18);
  a18=(a0*a20);
  a77=(a14*a9);
  a34=(a5*a66);
  a77=(a77+a34);
  a18=(a18-a77);
  a77=(a16*a58);
  a18=(a18+a77);
  a17=(a17*a18);
  a28=(a28/a7);
  a79=(a79*a10);
  a28=(a28+a79);
  a45=(a45*a28);
  a17=(a17-a45);
  a44=(a44+a17);
  a16=(a16*a9);
  a0=(a0*a66);
  a16=(a16-a0);
  a5=(a5*a20);
  a16=(a16-a5);
  a14=(a14*a58);
  a16=(a16+a14);
  a15=(a15*a16);
  a26=(a26/a7);
  a33=(a33*a10);
  a26=(a26+a33);
  a41=(a41*a26);
  a15=(a15-a41);
  a44=(a44+a15);
  a12=(a12*a42);
  a23=(a23/a7);
  a25=(a25*a10);
  a23=(a23+a25);
  a49=(a49*a23);
  a12=(a12-a49);
  a44=(a44+a12);
  a44=(a44/a67);
  a67=(a10+a10);
  a22=(a22*a67);
  a44=(a44-a22);
  a6=(a6*a44);
  a13=(a13-a6);
  if (res[0]!=0) res[0][12]=a13;
  a16=(a16/a7);
  a32=(a32*a10);
  a16=(a16-a32);
  a47=(a47*a44);
  a16=(a16-a47);
  if (res[0]!=0) res[0][13]=a16;
  a18=(a18/a7);
  a11=(a11*a10);
  a18=(a18-a11);
  a62=(a62*a44);
  a18=(a18-a62);
  if (res[0]!=0) res[0][14]=a18;
  a81=(a81/a7);
  a31=(a31*a10);
  a81=(a81-a31);
  a4=(a4*a64);
  a24=(a24*a44);
  a4=(a4+a24);
  a81=(a81-a4);
  if (res[0]!=0) res[0][15]=a81;
  return 0;
}

CASADI_SYMBOL_EXPORT int SRBD_cost_y_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int SRBD_cost_y_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int SRBD_cost_y_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void SRBD_cost_y_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int SRBD_cost_y_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void SRBD_cost_y_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void SRBD_cost_y_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void SRBD_cost_y_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int SRBD_cost_y_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int SRBD_cost_y_hess_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real SRBD_cost_y_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* SRBD_cost_y_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* SRBD_cost_y_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* SRBD_cost_y_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* SRBD_cost_y_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int SRBD_cost_y_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
