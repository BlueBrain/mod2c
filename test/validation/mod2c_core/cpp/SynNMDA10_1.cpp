/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#undef PI
 
#include "coreneuron/utils/randoms/nrnran123.h"
#include "coreneuron/nrnoc/md1redef.h"
#include "coreneuron/nrnconf.h"
#include "coreneuron/sim/multicore.hpp"
#include "coreneuron/nrniv/nrniv_decl.h"
#include "coreneuron/utils/ivocvect.hpp"
#include "coreneuron/utils/nrnoc_aux.hpp"
#include "coreneuron/gpu/nrn_acc_manager.hpp"
#include "coreneuron/sim/scopmath/newton_struct.h"
#include "coreneuron/sim/scopmath/newton_thread.hpp"
#include "coreneuron/sim/scopmath/sparse_thread.hpp"
#include "coreneuron/sim/scopmath/ssimplic_thread.hpp"
#include "coreneuron/nrnoc/md2redef.h"
#include "coreneuron/mechanism/register_mech.hpp"
#if !NRNGPU
#if !defined(DISABLE_HOC_EXP)
#undef exp
#define exp hoc_Exp
#endif
#endif
 namespace coreneuron {
 
#define _thread_present_ /**/ , _thread[0:2] , _slist1[0:10], _dlist1[0:10] 
 
#if defined(_OPENACC) && !defined(DISABLE_OPENACC)
#include <openacc.h>
#define _PRAGMA_FOR_INIT_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], nrn_ion_global_map[0:nrn_ion_global_map_size][0:ion_global_map_member_size], _nt[0:1] _thread_present_) if(_nt->compute_gpu)")
#define _PRAGMA_FOR_STATE_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _nt[0:1], _ml[0:1] _thread_present_) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_CUR_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _vec_d[0:_nt->end], _vec_rhs[0:_nt->end], _nt[0:1] _thread_present_) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_CUR_SYN_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _vec_shadow_rhs[0:_nt->shadow_rhs_cnt], _vec_shadow_d[0:_nt->shadow_rhs_cnt], _vec_d[0:_nt->end], _vec_rhs[0:_nt->end], _nt[0:1]) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_NETRECV_ACC_LOOP_ _Pragma("acc parallel loop present(_pnt[0:_pnt_length], _nrb[0:1], _nt[0:1], nrn_threads[0:nrn_nthread]) if(_nt->compute_gpu) async(stream_id)")
#else
#define _PRAGMA_FOR_INIT_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_STATE_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_CUR_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_CUR_SYN_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_NETRECV_ACC_LOOP_ _Pragma("")
#endif
 
#if defined(__ICC) || defined(__INTEL_COMPILER)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ivdep")
#elif defined(__IBMC__) || defined(__IBMCPP__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ibm independent_loop")
#elif defined(__PGI)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("vector")
#elif defined(_CRAYC)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("_CRI ivdep")
#elif defined(__clang__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("clang loop vectorize(enable)")
#elif defined(__GNUC__) || defined(__GNUG__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("GCC ivdep")
#else
#define _PRAGMA_FOR_VECTOR_LOOP_
#endif // _PRAGMA_FOR_VECTOR_LOOP_
 
#if !defined(LAYOUT)
/* 1 means AoS, >1 means AoSoA, <= 0 means SOA */
#define LAYOUT 1
#endif
#if LAYOUT >= 1
#define _STRIDE LAYOUT
#else
#define _STRIDE _cntml_padded + _iml
#endif
 

#if !defined(NET_RECEIVE_BUFFERING)
#define NET_RECEIVE_BUFFERING 1
#endif
 
#define nrn_init _nrn_init__NMDA10_1
#define nrn_cur _nrn_cur__NMDA10_1
#define _nrn_current _nrn_current__NMDA10_1
#define nrn_jacob _nrn_jacob__NMDA10_1
#define nrn_state _nrn_state__NMDA10_1
#define initmodel initmodel__NMDA10_1
#define _net_receive _net_receive__NMDA10_1
#define _net_init _net_init__NMDA10_1
#define nrn_state_launcher nrn_state_NMDA10_1_launcher
#define nrn_cur_launcher nrn_cur_NMDA10_1_launcher
#define nrn_jacob_launcher nrn_jacob_NMDA10_1_launcher 
#if NET_RECEIVE_BUFFERING
#define _net_buf_receive _net_buf_receive_NMDA10_1
void _net_buf_receive(NrnThread*);
#endif
 
#define kstates kstates_NMDA10_1 
#define release release_NMDA10_1 
 
#undef _threadargscomma_
#undef _threadargsprotocomma_
#undef _threadargs_
#undef _threadargsproto_
 
#define _threadargscomma_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, v,
#define _threadargsprotocomma_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, NrnThread* _nt, double v,
#define _threadargs_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, v
#define _threadargsproto_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, NrnThread* _nt, double v
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Erev _p[0*_STRIDE]
#define gmax _p[1*_STRIDE]
#define tau _p[2*_STRIDE]
#define T_max _p[3*_STRIDE]
#define i _p[4*_STRIDE]
#define g _p[5*_STRIDE]
#define T _p[6*_STRIDE]
#define tRel _p[7*_STRIDE]
#define synon _p[8*_STRIDE]
#define rb _p[9*_STRIDE]
#define rmb _p[10*_STRIDE]
#define rmu _p[11*_STRIDE]
#define rbMg _p[12*_STRIDE]
#define rmc1b _p[13*_STRIDE]
#define rmc1u _p[14*_STRIDE]
#define rmc2b _p[15*_STRIDE]
#define rmc2u _p[16*_STRIDE]
#define rmd1b _p[17*_STRIDE]
#define rmd1u _p[18*_STRIDE]
#define rmd2b _p[19*_STRIDE]
#define rmd2u _p[20*_STRIDE]
#define U _p[21*_STRIDE]
#define Cl _p[22*_STRIDE]
#define D1 _p[23*_STRIDE]
#define D2 _p[24*_STRIDE]
#define O _p[25*_STRIDE]
#define UMg _p[26*_STRIDE]
#define ClMg _p[27*_STRIDE]
#define D1Mg _p[28*_STRIDE]
#define D2Mg _p[29*_STRIDE]
#define OMg _p[30*_STRIDE]
#define w _p[31*_STRIDE]
#define DU _p[32*_STRIDE]
#define DCl _p[33*_STRIDE]
#define DD1 _p[34*_STRIDE]
#define DD2 _p[35*_STRIDE]
#define DO _p[36*_STRIDE]
#define DUMg _p[37*_STRIDE]
#define DClMg _p[38*_STRIDE]
#define DD1Mg _p[39*_STRIDE]
#define DD2Mg _p[40*_STRIDE]
#define DOMg _p[41*_STRIDE]
#define _v_unused _p[42*_STRIDE]
#define _g_unused _p[43*_STRIDE]
#define _tsav _p[44*_STRIDE]
 
#ifndef NRN_PRCELLSTATE
#define NRN_PRCELLSTATE 0
#endif
#if NRN_PRCELLSTATE
#define _PRCELLSTATE_V _v_unused = _v;
#define _PRCELLSTATE_G _g_unused = _g;
#else
#define _PRCELLSTATE_V /**/
#define _PRCELLSTATE_G /**/
#endif
#define _nd_area  _nt_data[_ppvar[0*_STRIDE]]
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 static int hoc_nrnpointerindex =  -1;
 static ThreadDatum* _extcall_thread;
 /* external NEURON variables */
 
#if 0 /*BBCORE*/
 /* declaration of user functions */
 static double _hoc_release();
 
#endif /*BBCORE*/
 static int _mechtype;
 static int _pointtype;
 
#if 0 /*BBCORE*/
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 
#endif /*BBCORE*/
 
#if 0 /*BBCORE*/
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "release", _hoc_release,
 0, 0
};
 
#endif /*BBCORE*/
 /* declare global and static user variables */
#define Rmc2u Rmc2u_NMDA10_1
 double Rmc2u = 0.00504192;
 #pragma acc declare copyin (Rmc2u)
#define Rmc2b Rmc2b_NMDA10_1
 double Rmc2b = 5e-05;
 #pragma acc declare copyin (Rmc2b)
#define Rmc1u Rmc1u_NMDA10_1
 double Rmc1u = 0.00243831;
 #pragma acc declare copyin (Rmc1u)
#define Rmc1b Rmc1b_NMDA10_1
 double Rmc1b = 5e-05;
 #pragma acc declare copyin (Rmc1b)
#define Rmd2u Rmd2u_NMDA10_1
 double Rmd2u = 0.00295341;
 #pragma acc declare copyin (Rmd2u)
#define Rmd2b Rmd2b_NMDA10_1
 double Rmd2b = 5e-05;
 #pragma acc declare copyin (Rmd2b)
#define Rmd1u Rmd1u_NMDA10_1
 double Rmd1u = 0.00298874;
 #pragma acc declare copyin (Rmd1u)
#define Rmd1b Rmd1b_NMDA10_1
 double Rmd1b = 5e-05;
 #pragma acc declare copyin (Rmd1b)
#define RcMg RcMg_NMDA10_1
 double RcMg = 0.548;
 #pragma acc declare copyin (RcMg)
#define RoMg RoMg_NMDA10_1
 double RoMg = 0.01;
 #pragma acc declare copyin (RoMg)
#define Rr2Mg Rr2Mg_NMDA10_1
 double Rr2Mg = 0.00042;
 #pragma acc declare copyin (Rr2Mg)
#define Rd2Mg Rd2Mg_NMDA10_1
 double Rd2Mg = 0.00026;
 #pragma acc declare copyin (Rd2Mg)
#define Rr1Mg Rr1Mg_NMDA10_1
 double Rr1Mg = 0.00087;
 #pragma acc declare copyin (Rr1Mg)
#define Rd1Mg Rd1Mg_NMDA10_1
 double Rd1Mg = 0.0021;
 #pragma acc declare copyin (Rd1Mg)
#define RuMg RuMg_NMDA10_1
 double RuMg = 0.0171;
 #pragma acc declare copyin (RuMg)
#define RbMg RbMg_NMDA10_1
 double RbMg = 10;
 #pragma acc declare copyin (RbMg)
#define Rmu Rmu_NMDA10_1
 double Rmu = 12.8;
 #pragma acc declare copyin (Rmu)
#define Rmb Rmb_NMDA10_1
 double Rmb = 0.05;
 #pragma acc declare copyin (Rmb)
#define Rc Rc_NMDA10_1
 double Rc = 0.273;
 #pragma acc declare copyin (Rc)
#define Ro Ro_NMDA10_1
 double Ro = 0.01;
 #pragma acc declare copyin (Ro)
#define Rr2 Rr2_NMDA10_1
 double Rr2 = 0.0005;
 #pragma acc declare copyin (Rr2)
#define Rd2 Rd2_NMDA10_1
 double Rd2 = 0.00043;
 #pragma acc declare copyin (Rd2)
#define Rr1 Rr1_NMDA10_1
 double Rr1 = 0.0016;
 #pragma acc declare copyin (Rr1)
#define Rd1 Rd1_NMDA10_1
 double Rd1 = 0.0022;
 #pragma acc declare copyin (Rd1)
#define Ru Ru_NMDA10_1
 double Ru = 0.0056;
 #pragma acc declare copyin (Ru)
#define Rb Rb_NMDA10_1
 double Rb = 10;
 #pragma acc declare copyin (Rb)
#define memb_fraction memb_fraction_NMDA10_1
 double memb_fraction = 0.8;
 #pragma acc declare copyin (memb_fraction)
#define mg mg_NMDA10_1
 double mg = 1;
 #pragma acc declare copyin (mg)
#define valence valence_NMDA10_1
 double valence = -2;
 #pragma acc declare copyin (valence)
 
static void _acc_globals_update() {
 #pragma acc update device (Rmc2u) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmc2b) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmc1u) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmc1b) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmd2u) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmd2b) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmd1u) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmd1b) if(nrn_threads->compute_gpu)
 #pragma acc update device (RcMg) if(nrn_threads->compute_gpu)
 #pragma acc update device (RoMg) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rr2Mg) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rd2Mg) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rr1Mg) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rd1Mg) if(nrn_threads->compute_gpu)
 #pragma acc update device (RuMg) if(nrn_threads->compute_gpu)
 #pragma acc update device (RbMg) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmu) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rmb) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rc) if(nrn_threads->compute_gpu)
 #pragma acc update device (Ro) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rr2) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rd2) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rr1) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rd1) if(nrn_threads->compute_gpu)
 #pragma acc update device (Ru) if(nrn_threads->compute_gpu)
 #pragma acc update device (Rb) if(nrn_threads->compute_gpu)
 #pragma acc update device (memb_fraction) if(nrn_threads->compute_gpu)
 #pragma acc update device (mg) if(nrn_threads->compute_gpu)
 #pragma acc update device (valence) if(nrn_threads->compute_gpu)
 }

 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "mg_NMDA10_1", "mM",
 "Rb_NMDA10_1", "/mM",
 "Ru_NMDA10_1", "/ms",
 "Ro_NMDA10_1", "/ms",
 "Rc_NMDA10_1", "/ms",
 "Rd1_NMDA10_1", "/ms",
 "Rr1_NMDA10_1", "/ms",
 "Rd2_NMDA10_1", "/ms",
 "Rr2_NMDA10_1", "/ms",
 "Rmb_NMDA10_1", "/mM",
 "Rmu_NMDA10_1", "/ms",
 "Rmc1b_NMDA10_1", "/mM",
 "Rmc1u_NMDA10_1", "/ms",
 "Rmc2b_NMDA10_1", "/mM",
 "Rmc2u_NMDA10_1", "/ms",
 "Rmd1b_NMDA10_1", "/mM",
 "Rmd1u_NMDA10_1", "/ms",
 "Rmd2b_NMDA10_1", "/mM",
 "Rmd2u_NMDA10_1", "/ms",
 "RbMg_NMDA10_1", "/mM",
 "RuMg_NMDA10_1", "/ms",
 "RoMg_NMDA10_1", "/ms",
 "RcMg_NMDA10_1", "/ms",
 "Rd1Mg_NMDA10_1", "/ms",
 "Rr1Mg_NMDA10_1", "/ms",
 "Rd2Mg_NMDA10_1", "/ms",
 "Rr2Mg_NMDA10_1", "/ms",
 "Erev", "mV",
 "gmax", "pS",
 "tau", "ms",
 "T_max", "mM",
 "i", "nA",
 "g", "uS",
 "T", "mM",
 "tRel", "ms",
 "rb", "/ms",
 "rmb", "/ms",
 "rmu", "/ms",
 "rbMg", "/ms",
 "rmc1b", "/ms",
 "rmc1u", "/ms",
 "rmc2b", "/ms",
 "rmc2u", "/ms",
 "rmd1b", "/ms",
 "rmd1u", "/ms",
 "rmd2b", "/ms",
 "rmd2u", "/ms",
 0,0
};
 
#endif /*BBCORE*/
 static double ClMg0 = 0;
#pragma acc declare copyin(ClMg0)
 static double Cl0 = 0;
#pragma acc declare copyin(Cl0)
 static double D2Mg0 = 0;
#pragma acc declare copyin(D2Mg0)
 static double D1Mg0 = 0;
#pragma acc declare copyin(D1Mg0)
 static double D20 = 0;
#pragma acc declare copyin(D20)
 static double D10 = 0;
#pragma acc declare copyin(D10)
 static double OMg0 = 0;
#pragma acc declare copyin(OMg0)
 static double O0 = 0;
#pragma acc declare copyin(O0)
 static double UMg0 = 0;
#pragma acc declare copyin(UMg0)
 static double U0 = 0;
#pragma acc declare copyin(U0)
 static double delta_t = 1;
#pragma acc declare copyin(delta_t)
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "mg_NMDA10_1", &mg_NMDA10_1,
 "valence_NMDA10_1", &valence_NMDA10_1,
 "memb_fraction_NMDA10_1", &memb_fraction_NMDA10_1,
 "Rb_NMDA10_1", &Rb_NMDA10_1,
 "Ru_NMDA10_1", &Ru_NMDA10_1,
 "Ro_NMDA10_1", &Ro_NMDA10_1,
 "Rc_NMDA10_1", &Rc_NMDA10_1,
 "Rd1_NMDA10_1", &Rd1_NMDA10_1,
 "Rr1_NMDA10_1", &Rr1_NMDA10_1,
 "Rd2_NMDA10_1", &Rd2_NMDA10_1,
 "Rr2_NMDA10_1", &Rr2_NMDA10_1,
 "Rmb_NMDA10_1", &Rmb_NMDA10_1,
 "Rmu_NMDA10_1", &Rmu_NMDA10_1,
 "Rmc1b_NMDA10_1", &Rmc1b_NMDA10_1,
 "Rmc1u_NMDA10_1", &Rmc1u_NMDA10_1,
 "Rmc2b_NMDA10_1", &Rmc2b_NMDA10_1,
 "Rmc2u_NMDA10_1", &Rmc2u_NMDA10_1,
 "Rmd1b_NMDA10_1", &Rmd1b_NMDA10_1,
 "Rmd1u_NMDA10_1", &Rmd1u_NMDA10_1,
 "Rmd2b_NMDA10_1", &Rmd2b_NMDA10_1,
 "Rmd2u_NMDA10_1", &Rmd2u_NMDA10_1,
 "RbMg_NMDA10_1", &RbMg_NMDA10_1,
 "RuMg_NMDA10_1", &RuMg_NMDA10_1,
 "RoMg_NMDA10_1", &RoMg_NMDA10_1,
 "RcMg_NMDA10_1", &RcMg_NMDA10_1,
 "Rd1Mg_NMDA10_1", &Rd1Mg_NMDA10_1,
 "Rr1Mg_NMDA10_1", &Rr1Mg_NMDA10_1,
 "Rd2Mg_NMDA10_1", &Rd2Mg_NMDA10_1,
 "Rr2Mg_NMDA10_1", &Rr2Mg_NMDA10_1,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(double*, Datum*, int);
void nrn_init(NrnThread*, Memb_list*, int);
void nrn_state(NrnThread*, Memb_list*, int);
 void nrn_cur(NrnThread*, Memb_list*, int);
 
#if 0 /*BBCORE*/
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
#endif /*BBCORE*/
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"NMDA10_1",
 "Erev",
 "gmax",
 "tau",
 "T_max",
 0,
 "i",
 "g",
 "T",
 "tRel",
 "synon",
 "rb",
 "rmb",
 "rmu",
 "rbMg",
 "rmc1b",
 "rmc1u",
 "rmc2b",
 "rmc2u",
 "rmd1b",
 "rmd1u",
 "rmd2b",
 "rmd2u",
 0,
 "U",
 "Cl",
 "D1",
 "D2",
 "O",
 "UMg",
 "ClMg",
 "D1Mg",
 "D2Mg",
 "OMg",
 0,
 0};
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	Erev = 0;
 	gmax = 50;
 	tau = 0.3;
 	T_max = 1.5;
 
#endif /* BBCORE */
 
}
 static void _initlists();
 void _net_receive(Point_process*, int, double);
 static void _thread_cleanup(ThreadDatum*);
 
#define _psize 45
#define _ppsize 2
 void _SynNMDA10_1_reg() {
	int _vectorized = 1;
  _initlists();
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 
#if 0 /*BBCORE*/
 
#endif /*BBCORE*/
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init,
	 hoc_nrnpointerindex,
	 NULL/*_hoc_create_pnt*/, NULL/*_hoc_destroy_pnt*/, /*_member_func,*/
	 3);
  _extcall_thread = (ThreadDatum*)ecalloc(2, sizeof(ThreadDatum));
     _nrn_thread_reg0(_mechtype, _thread_cleanup);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
 
#if NET_RECEIVE_BUFFERING
  hoc_register_net_receive_buffering(_net_buf_receive, _mechtype);
#endif
 set_pnt_receive(_mechtype, _net_receive, nullptr, 1);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, NULL);
 }
static const char *modelname = "kinetic NMDA receptor model";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static inline int release(_threadargsprotocomma_ double);
 
#define _MATELM1(_row,_col) _nrn_thread_getelm((SparseObj*)_so, _row + 1, _col + 1, _iml)[_iml]
 
#define _RHS1(_arg) _rhs[(_arg+1)*_STRIDE]
  
#define _linmat1  1
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 
#define _slist1 _slist1_NMDA10_1
int* _slist1;
#pragma acc declare create(_slist1)

#define _dlist1 _dlist1_NMDA10_1
int* _dlist1;
#pragma acc declare create(_dlist1)
 
/* _kinetic_ kstates _NMDA10_1 */
#ifndef INSIDE_NMODL
#define INSIDE_NMODL
#endif
 
struct kstates_NMDA10_1 {
  int operator()(SparseObj* _so, double* _rhs, _threadargsproto_) const;
};
int kstates_NMDA10_1::operator() (SparseObj* _so, double* _rhs, _threadargsproto_) const
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<10;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]*_STRIDE] - _p[_dlist1[_i]*_STRIDE]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 release ( _threadargscomma_ t ) ;
   rb = Rb * T ;
   rbMg = RbMg * T ;
   rmb = Rmb * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
   rmu = Rmu * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
   rmc1b = Rmc1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
   rmc1u = Rmc1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
   rmc2b = Rmc2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
   rmc2u = Rmc2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
   rmd1b = Rmd1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
   rmd1u = Rmd1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
   rmd2b = Rmd2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
   rmd2u = Rmd2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
   /* ~ U <-> Cl ( rb , Ru )*/
 f_flux =  rb * U ;
 b_flux =  Ru * Cl ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  rb ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  Ru ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 f_flux =  Ro * Cl ;
 b_flux =  Rc * O ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  Ro ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 7 ,2)  -= _term;
 _term =  Rc ;
 _MATELM1( 2 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 f_flux =  Rd1 * Cl ;
 b_flux =  Rr1 * D1 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  Rd1 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 6 ,2)  -= _term;
 _term =  Rr1 ;
 _MATELM1( 2 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 f_flux =  Rd2 * D1 ;
 b_flux =  Rr2 * D2 ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  Rd2 ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Rr2 ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( rmb , rmu )*/
 f_flux =  rmb * O ;
 b_flux =  rmu * OMg ;
 _RHS1( 7) -= (f_flux - b_flux);
 
 _term =  rmb ;
 _MATELM1( 7 ,7)  += _term;
 _term =  rmu ;
 _MATELM1( 7 ,0)  -= _term;
 /*REACTION*/
  /* ~ UMg <-> ClMg ( rbMg , RuMg )*/
 f_flux =  rbMg * UMg ;
 b_flux =  RuMg * ClMg ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  rbMg ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 1 ,8)  -= _term;
 _term =  RuMg ;
 _MATELM1( 8 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ ClMg <-> OMg ( RoMg , RcMg )*/
 f_flux =  RoMg * ClMg ;
 b_flux =  RcMg * OMg ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  RoMg ;
 _MATELM1( 1 ,1)  += _term;
 _term =  RcMg ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
  /* ~ ClMg <-> D1Mg ( Rd1Mg , Rr1Mg )*/
 f_flux =  Rd1Mg * ClMg ;
 b_flux =  Rr1Mg * D1Mg ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  Rd1Mg ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 4 ,1)  -= _term;
 _term =  Rr1Mg ;
 _MATELM1( 1 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ D1Mg <-> D2Mg ( Rd2Mg , Rr2Mg )*/
 f_flux =  Rd2Mg * D1Mg ;
 b_flux =  Rr2Mg * D2Mg ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  Rd2Mg ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  Rr2Mg ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ U <-> UMg ( rmc1b , rmc1u )*/
 f_flux =  rmc1b * U ;
 b_flux =  rmc1u * UMg ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  rmc1b ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  rmc1u ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ Cl <-> ClMg ( rmc2b , rmc2u )*/
 f_flux =  rmc2b * Cl ;
 b_flux =  rmc2u * ClMg ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  rmc2b ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  rmc2u ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D1Mg ( rmd1b , rmd1u )*/
 f_flux =  rmd1b * D1 ;
 b_flux =  rmd1u * D1Mg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  rmd1b ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  rmd1u ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ D2 <-> D2Mg ( rmd2b , rmd2u )*/
 f_flux =  rmd2b * D2 ;
 b_flux =  rmd2u * D2Mg ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  rmd2b ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 3 ,5)  -= _term;
 _term =  rmd2u ;
 _MATELM1( 5 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* U + Cl + D1 + D2 + O + UMg + ClMg + D1Mg + D2Mg + OMg = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= OMg ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= D2Mg ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= D1Mg ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= ClMg ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= UMg ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= D2 ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= D1 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= Cl ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= U ;
 /*CONSERVATION*/
   } return _reset;
 }
 
#if NET_RECEIVE_BUFFERING 
#undef t
#define t _nrb_t
static inline void _net_receive_kernel(double, Point_process*, int _weight_index, double _flag);
void _net_buf_receive(NrnThread* _nt) {
  if (!_nt->_ml_list) { return; }
  Memb_list* _ml = _nt->_ml_list[_mechtype];
  if (!_ml) { return; }
  NetReceiveBuffer_t* _nrb = _ml->_net_receive_buffer; 
  int _di;
  int stream_id = _nt->stream_id;
  Point_process* _pnt = _nt->pntprocs;
  int _pnt_length = _nt->n_pntproc - _nrb->_pnt_offset;
  int _displ_cnt = _nrb->_displ_cnt;
  _PRAGMA_FOR_NETRECV_ACC_LOOP_ 
  for (_di = 0; _di < _displ_cnt; ++_di) {
    int _inrb;
    int _di0 = _nrb->_displ[_di];
    int _di1 = _nrb->_displ[_di + 1];
    for (_inrb = _di0; _inrb < _di1; ++_inrb) {
      int _i = _nrb->_nrb_index[_inrb];
      int _j = _nrb->_pnt_index[_i];
      int _k = _nrb->_weight_index[_i];
      double _nrt = _nrb->_nrb_t[_i];
      double _nrflag = _nrb->_nrb_flag[_i];
      _net_receive_kernel(_nrt, _pnt + _j, _k, _nrflag);
    }
  }
  #pragma acc wait(stream_id)
  _nrb->_displ_cnt = 0;
  _nrb->_cnt = 0;
  /*printf("_net_buf_receive__NMDA10_1  %d\n", _nt->_id);*/
 
}
 
void _net_receive (Point_process* _pnt, int _weight_index, double _lflag) {
  NrnThread* _nt = nrn_threads + _pnt->_tid;
  NetReceiveBuffer_t* _nrb = _nt->_ml_list[_mechtype]->_net_receive_buffer;
  if (_nrb->_cnt >= _nrb->_size){
    realloc_net_receive_buffer(_nt, _nt->_ml_list[_mechtype]);
  }
  _nrb->_pnt_index[_nrb->_cnt] = _pnt - _nt->pntprocs;
  _nrb->_weight_index[_nrb->_cnt] = _weight_index;
  _nrb->_nrb_t[_nrb->_cnt] = _nt->_t;
  _nrb->_nrb_flag[_nrb->_cnt] = _lflag;
  ++_nrb->_cnt;
}
 
static void _net_receive_kernel(double _nrb_t, Point_process* _pnt, int _weight_index, double _lflag)
#else
 
void _net_receive (Point_process* _pnt, int _weight_index, double _lflag) 
#endif
 
{  double* _p; Datum* _ppvar; ThreadDatum* _thread; double v = 0;
   Memb_list* _ml; int _cntml_padded, _cntml_actual; int _iml; double* _args;
 
   NrnThread* _nt;
   int _tid = _pnt->_tid; 
   _nt = nrn_threads + _tid;
   _thread = (ThreadDatum*)0; 
   double *_weights = _nt->_weights;
   _args = _weights + _weight_index;
   _ml = _nt->_ml_list[_pnt->_type];
   _cntml_actual = _ml->_nodecount;
   _cntml_padded = _ml->_nodecount_padded;
   _iml = _pnt->_i_instance;
#if LAYOUT == 1 /*AoS*/
   _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#endif
#if LAYOUT == 0 /*SoA*/
   _p = _ml->_data; _ppvar = _ml->_pdata;
#endif
#if LAYOUT > 1 /*AoSoA*/
#error AoSoA not implemented.
#endif
  #if !defined(_OPENACC) 
 assert(_tsav <= t); 
 #endif 
 _tsav = t; {
   if ( _lflag  == 0.0 ) {
     tRel = t ;
     synon = 1.0 ;
     w = _args[0] ;
     }
   } 
#if NET_RECEIVE_BUFFERING
#undef t
#define t _nt->_t
#endif
 }
 
static int  release ( _threadargsprotocomma_ double _lt ) {
   T = T_max * ( _lt - tRel ) / tau * exp ( 1.0 - ( _lt - tRel ) / tau ) * synon ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static double _hoc_release(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 release ( _threadargs_, *getarg(1) );
 return(_r);
}
 
#endif /*BBCORE*/
 
/*CVODE ode begin*/
 static int _ode_spec1(_threadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<10;_i++) _p[_dlist1[_i]] = 0.0;}
 release ( _threadargscomma_ t ) ;
 rb = Rb * T ;
 rbMg = RbMg * T ;
 rmb = Rmb * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmu = Rmu * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmc1b = Rmc1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmc1u = Rmc1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmc2b = Rmc2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmc2u = Rmc2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmd1b = Rmd1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmd1u = Rmd1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmd2b = Rmd2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmd2u = Rmd2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 /* ~ U <-> Cl ( rb , Ru )*/
 f_flux =  rb * U ;
 b_flux =  Ru * Cl ;
 DU -= (f_flux - b_flux);
 DCl += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 f_flux =  Ro * Cl ;
 b_flux =  Rc * O ;
 DCl -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 f_flux =  Rd1 * Cl ;
 b_flux =  Rr1 * D1 ;
 DCl -= (f_flux - b_flux);
 DD1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 f_flux =  Rd2 * D1 ;
 b_flux =  Rr2 * D2 ;
 DD1 -= (f_flux - b_flux);
 DD2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ O <-> OMg ( rmb , rmu )*/
 f_flux =  rmb * O ;
 b_flux =  rmu * OMg ;
 DO -= (f_flux - b_flux);
 DOMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ UMg <-> ClMg ( rbMg , RuMg )*/
 f_flux =  rbMg * UMg ;
 b_flux =  RuMg * ClMg ;
 DUMg -= (f_flux - b_flux);
 DClMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ ClMg <-> OMg ( RoMg , RcMg )*/
 f_flux =  RoMg * ClMg ;
 b_flux =  RcMg * OMg ;
 DClMg -= (f_flux - b_flux);
 DOMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ ClMg <-> D1Mg ( Rd1Mg , Rr1Mg )*/
 f_flux =  Rd1Mg * ClMg ;
 b_flux =  Rr1Mg * D1Mg ;
 DClMg -= (f_flux - b_flux);
 DD1Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ D1Mg <-> D2Mg ( Rd2Mg , Rr2Mg )*/
 f_flux =  Rd2Mg * D1Mg ;
 b_flux =  Rr2Mg * D2Mg ;
 DD1Mg -= (f_flux - b_flux);
 DD2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ U <-> UMg ( rmc1b , rmc1u )*/
 f_flux =  rmc1b * U ;
 b_flux =  rmc1u * UMg ;
 DU -= (f_flux - b_flux);
 DUMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ Cl <-> ClMg ( rmc2b , rmc2u )*/
 f_flux =  rmc2b * Cl ;
 b_flux =  rmc2u * ClMg ;
 DCl -= (f_flux - b_flux);
 DClMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ D1 <-> D1Mg ( rmd1b , rmd1u )*/
 f_flux =  rmd1b * D1 ;
 b_flux =  rmd1u * D1Mg ;
 DD1 -= (f_flux - b_flux);
 DD1Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ D2 <-> D2Mg ( rmd2b , rmd2u )*/
 f_flux =  rmd2b * D2 ;
 b_flux =  rmd2u * D2Mg ;
 DD2 -= (f_flux - b_flux);
 DD2Mg += (f_flux - b_flux);
 
 /*REACTION*/
   /* U + Cl + D1 + D2 + O + UMg + ClMg + D1Mg + D2Mg + OMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, _threadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<10;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 release ( _threadargscomma_ t ) ;
 rb = Rb * T ;
 rbMg = RbMg * T ;
 rmb = Rmb * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmu = Rmu * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmc1b = Rmc1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmc1u = Rmc1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmc2b = Rmc2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmc2u = Rmc2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmd1b = Rmd1b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmd1u = Rmd1u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 rmd2b = Rmd2b * mg * exp ( ( 1.0 * v - 40.0 ) * valence * memb_fraction / 25.0 ) ;
 rmd2u = Rmd2u * exp ( ( - 1.0 ) * ( v - 40.0 ) * valence * ( 1.0 - memb_fraction ) / 25.0 ) ;
 /* ~ U <-> Cl ( rb , Ru )*/
 _term =  rb ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  Ru ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 _term =  Ro ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 7 ,2)  -= _term;
 _term =  Rc ;
 _MATELM1( 2 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 _term =  Rd1 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 6 ,2)  -= _term;
 _term =  Rr1 ;
 _MATELM1( 2 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 _term =  Rd2 ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Rr2 ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( rmb , rmu )*/
 _term =  rmb ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 0 ,7)  -= _term;
 _term =  rmu ;
 _MATELM1( 7 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ UMg <-> ClMg ( rbMg , RuMg )*/
 _term =  rbMg ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 1 ,8)  -= _term;
 _term =  RuMg ;
 _MATELM1( 8 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ ClMg <-> OMg ( RoMg , RcMg )*/
 _term =  RoMg ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  RcMg ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ ClMg <-> D1Mg ( Rd1Mg , Rr1Mg )*/
 _term =  Rd1Mg ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 4 ,1)  -= _term;
 _term =  Rr1Mg ;
 _MATELM1( 1 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ D1Mg <-> D2Mg ( Rd2Mg , Rr2Mg )*/
 _term =  Rd2Mg ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  Rr2Mg ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ U <-> UMg ( rmc1b , rmc1u )*/
 _term =  rmc1b ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  rmc1u ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ Cl <-> ClMg ( rmc2b , rmc2u )*/
 _term =  rmc2b ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  rmc2u ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D1Mg ( rmd1b , rmd1u )*/
 _term =  rmd1b ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  rmd1u ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ D2 <-> D2Mg ( rmd2b , rmd2u )*/
 _term =  rmd2b ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 3 ,5)  -= _term;
 _term =  rmd2u ;
 _MATELM1( 5 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* U + Cl + D1 + D2 + O + UMg + ClMg + D1Mg + D2Mg + OMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static void _thread_cleanup(ThreadDatum* _thread) {
   _nrn_destroy_sparseobj_thread((SparseObj*)_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread((SparseObj*)_thread[_spth1]._pvoid);
 }

static inline void initmodel(_threadargsproto_) {
  int _i; double _save;{
  ClMg = ClMg0;
  Cl = Cl0;
  D2Mg = D2Mg0;
  D1Mg = D1Mg0;
  D2 = D20;
  D1 = D10;
  OMg = OMg0;
  O = O0;
  UMg = UMg0;
  U = U0;
 {
   T = 0.0 ;
   synon = 0.0 ;
   tRel = 0.0 ;
   U = 1.0 ;
   }
 
}
}

void nrn_init(NrnThread* _nt, Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double _v, v; int* _ni; int _iml, _cntml_padded, _cntml_actual;
    _ni = _ml->_nodeindices;
_cntml_actual = _ml->_nodecount;
_cntml_padded = _ml->_nodecount_padded;
_thread = _ml->_thread;
  if (!_thread[_spth1]._pvoid) {
    _thread[_spth1]._pvoid = nrn_cons_sparseobj(kstates_NMDA10_1{}, 10, _ml, _threadargs_);
    #ifdef _OPENACC
    if (_nt->compute_gpu) {
      void* _d_so = (void*) acc_deviceptr(_thread[_spth1]._pvoid);
      ThreadDatum* _d_td = (ThreadDatum*)acc_deviceptr(_thread);
      acc_memcpy_to_device(&(_d_td[_spth1]._pvoid), &_d_so, sizeof(void*));
    }
    #endif
  }
_acc_globals_update();
double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
int stream_id = _nt->stream_id;
  if (_nrn_skip_initmodel == 0) {
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#elif LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
_PRAGMA_FOR_INIT_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
    int _nd_idx = _ni[_iml];
 _tsav = -1e20;
    _v = _vec_v[_nd_idx];
    _PRCELLSTATE_V
 v = _v;
 _PRCELLSTATE_V
 initmodel(_threadargs_);
}
  }
}

static double _nrn_current(_threadargsproto_, double _v){double _current=0.;v=_v;{ {
   g = w * gmax * O ;
   i = g * ( v - Erev ) ;
   }
 _current += i;

} return _current;
}

#if defined(ENABLE_CUDA_INTERFACE) && defined(_OPENACC)
  void nrn_state_launcher(NrnThread*, Memb_list*, int, int);
  void nrn_jacob_launcher(NrnThread*, Memb_list*, int, int);
  void nrn_cur_launcher(NrnThread*, Memb_list*, int, int);
#endif


void nrn_cur(NrnThread* _nt, Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; ThreadDatum* _thread;
int* _ni; double _rhs, _g, _v, v; int _iml, _cntml_padded, _cntml_actual;
    _ni = _ml->_nodeindices;
_cntml_actual = _ml->_nodecount;
_cntml_padded = _ml->_nodecount_padded;
_thread = _ml->_thread;
double * _vec_rhs = _nt->_actual_rhs;
double * _vec_d = _nt->_actual_d;
double * _vec_shadow_rhs = _nt->_shadow_rhs;
double * _vec_shadow_d = _nt->_shadow_d;

#if defined(ENABLE_CUDA_INTERFACE) && defined(_OPENACC) && !defined(DISABLE_OPENACC)
  NrnThread* d_nt = acc_deviceptr(_nt);
  Memb_list* d_ml = acc_deviceptr(_ml);
  nrn_cur_launcher(d_nt, d_ml, _type, _cntml_actual);
  return;
#endif

double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
int stream_id = _nt->stream_id;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#elif LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
_PRAGMA_FOR_CUR_SYN_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
    _PRCELLSTATE_V
 _g = _nrn_current(_threadargs_, _v + .001);
 	{ _rhs = _nrn_current(_threadargs_, _v);
 	}
 _g = (_g - _rhs)/.001;
 double _mfact =  1.e2/(_nd_area);
 _g *=  _mfact;
 _rhs *= _mfact;
 _PRCELLSTATE_G


#ifdef _OPENACC
  if(_nt->compute_gpu) {
    #pragma acc atomic update
    _vec_rhs[_nd_idx] -= _rhs;
    #pragma acc atomic update
    _vec_d[_nd_idx] += _g;
  } else {
    _vec_shadow_rhs[_iml] = _rhs;
    _vec_shadow_d[_iml] = _g;
  }
#else
  _vec_shadow_rhs[_iml] = _rhs;
  _vec_shadow_d[_iml] = _g;
#endif
 }
#ifdef _OPENACC
    if(!(_nt->compute_gpu)) { 
        for (_iml = 0; _iml < _cntml_actual; ++_iml) {
           int _nd_idx = _ni[_iml];
           _vec_rhs[_nd_idx] -= _vec_shadow_rhs[_iml];
           _vec_d[_nd_idx] += _vec_shadow_d[_iml];
        }
#else
 for (_iml = 0; _iml < _cntml_actual; ++_iml) {
   int _nd_idx = _ni[_iml];
   _vec_rhs[_nd_idx] -= _vec_shadow_rhs[_iml];
   _vec_d[_nd_idx] += _vec_shadow_d[_iml];
#endif
 
}
 
}

void nrn_state(NrnThread* _nt, Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double v, _v = 0.0; int* _ni; int _iml, _cntml_padded, _cntml_actual;
    _ni = _ml->_nodeindices;
_cntml_actual = _ml->_nodecount;
_cntml_padded = _ml->_nodecount_padded;
_thread = _ml->_thread;

#if defined(ENABLE_CUDA_INTERFACE) && defined(_OPENACC) && !defined(DISABLE_OPENACC)
  NrnThread* d_nt = acc_deviceptr(_nt);
  Memb_list* d_ml = acc_deviceptr(_ml);
  nrn_state_launcher(d_nt, d_ml, _type, _cntml_actual);
  return;
#endif

double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
int stream_id = _nt->stream_id;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#elif LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
_PRAGMA_FOR_STATE_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
    _PRCELLSTATE_V
 v=_v;
{
 {  
  sparse_thread(static_cast<SparseObj*>(_thread[_spth1]._pvoid), 10, _slist1, _dlist1, &t, dt, kstates_NMDA10_1{}, _linmat1, _threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
 int _cntml_actual=1;
 int _cntml_padded=1;
 int _iml=0;
  if (!_first) return;
 
 _slist1 = (int*)malloc(sizeof(int)*10);
 _dlist1 = (int*)malloc(sizeof(int)*10);
 _slist1[0] = &(OMg) - _p;  _dlist1[0] = &(DOMg) - _p;
 _slist1[1] = &(ClMg) - _p;  _dlist1[1] = &(DClMg) - _p;
 _slist1[2] = &(Cl) - _p;  _dlist1[2] = &(DCl) - _p;
 _slist1[3] = &(D2Mg) - _p;  _dlist1[3] = &(DD2Mg) - _p;
 _slist1[4] = &(D1Mg) - _p;  _dlist1[4] = &(DD1Mg) - _p;
 _slist1[5] = &(D2) - _p;  _dlist1[5] = &(DD2) - _p;
 _slist1[6] = &(D1) - _p;  _dlist1[6] = &(DD1) - _p;
 _slist1[7] = &(O) - _p;  _dlist1[7] = &(DO) - _p;
 _slist1[8] = &(UMg) - _p;  _dlist1[8] = &(DUMg) - _p;
 _slist1[9] = &(U) - _p;  _dlist1[9] = &(DU) - _p;
 #pragma acc enter data copyin(_slist1[0:10])
 #pragma acc enter data copyin(_dlist1[0:10])

_first = 0;
}
} // namespace coreneuron_lib
