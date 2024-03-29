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
 
#define _thread_present_ /**/ , _thread[0:3] 
 
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
 
#define nrn_init _nrn_init__NMDA16
#define nrn_cur _nrn_cur__NMDA16
#define _nrn_current _nrn_current__NMDA16
#define nrn_jacob _nrn_jacob__NMDA16
#define nrn_state _nrn_state__NMDA16
#define initmodel initmodel__NMDA16
#define _net_receive _net_receive__NMDA16
#define _net_init _net_init__NMDA16
#define nrn_state_launcher nrn_state_NMDA16_launcher
#define nrn_cur_launcher nrn_cur_NMDA16_launcher
#define nrn_jacob_launcher nrn_jacob_NMDA16_launcher 
#if NET_RECEIVE_BUFFERING
#define _net_buf_receive _net_buf_receive_NMDA16
void _net_buf_receive(NrnThread*);
#endif
 
#define kstates kstates_NMDA16 
#define rates rates_NMDA16 
 
#undef _threadargscomma_
#undef _threadargsprotocomma_
#undef _threadargs_
#undef _threadargsproto_
 
#define _threadargscomma_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, _ml, v,
#define _threadargsprotocomma_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, NrnThread* _nt, Memb_list* _ml, double v,
#define _threadargs_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, _ml, v
#define _threadargsproto_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, NrnThread* _nt, Memb_list* _ml, double v
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Erev _p[0*_STRIDE]
#define tau _p[1*_STRIDE]
#define T_max _p[2*_STRIDE]
#define kd1F _p[3*_STRIDE]
#define kd1B _p[4*_STRIDE]
#define kd2F _p[5*_STRIDE]
#define kd2B _p[6*_STRIDE]
#define csi _p[7*_STRIDE]
#define i _p[8*_STRIDE]
#define g _p[9*_STRIDE]
#define T _p[10*_STRIDE]
#define tRel _p[11*_STRIDE]
#define synon _p[12*_STRIDE]
#define R _p[13*_STRIDE]
#define RA _p[14*_STRIDE]
#define RA2 _p[15*_STRIDE]
#define RA2d1 _p[16*_STRIDE]
#define RA2d2 _p[17*_STRIDE]
#define RA2f _p[18*_STRIDE]
#define RA2s _p[19*_STRIDE]
#define O _p[20*_STRIDE]
#define OMg _p[21*_STRIDE]
#define RMg _p[22*_STRIDE]
#define RAMg _p[23*_STRIDE]
#define RA2Mg _p[24*_STRIDE]
#define RA2d1Mg _p[25*_STRIDE]
#define RA2d2Mg _p[26*_STRIDE]
#define RA2fMg _p[27*_STRIDE]
#define RA2sMg _p[28*_STRIDE]
#define w _p[29*_STRIDE]
#define nao _p[30*_STRIDE]
#define DR _p[31*_STRIDE]
#define DRA _p[32*_STRIDE]
#define DRA2 _p[33*_STRIDE]
#define DRA2d1 _p[34*_STRIDE]
#define DRA2d2 _p[35*_STRIDE]
#define DRA2f _p[36*_STRIDE]
#define DRA2s _p[37*_STRIDE]
#define DO _p[38*_STRIDE]
#define DOMg _p[39*_STRIDE]
#define DRMg _p[40*_STRIDE]
#define DRAMg _p[41*_STRIDE]
#define DRA2Mg _p[42*_STRIDE]
#define DRA2d1Mg _p[43*_STRIDE]
#define DRA2d2Mg _p[44*_STRIDE]
#define DRA2fMg _p[45*_STRIDE]
#define DRA2sMg _p[46*_STRIDE]
#define _v_unused _p[47*_STRIDE]
#define _g_unused _p[48*_STRIDE]
#define _tsav _p[49*_STRIDE]
 
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
#define _ion_nao		_nt_data[_ppvar[2*_STRIDE]]
 
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
 extern double celsius;
 #define nrn_ghk(v, ci, co, z) nrn_ghk(v, ci, co, z, celsius)
 #if 0 /*BBCORE*/
 /* declaration of user functions */
 static double _hoc_rates();
 
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
 "rates", _hoc_rates,
 0, 0
};
 
#endif /*BBCORE*/
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[10];
#define _gth 2
 static double Kcs0_NMDA16 = 0.27;
 static double Kna_NMDA16 = 34.4;
#define Kcs_NMDA16 _thread1data[0]
#define Kcs _thread[_gth]._pval[0]
 static double Mg_NMDA16 = 1;
 static double V0_NMDA16 = -100;
 static double Vdep_NMDA16 = 175;
 static double a_NMDA16 = -21;
 static double b_NMDA16 = -55;
 static double c_NMDA16 = 52.7;
 static double d_NMDA16 = -50;
 static double gmax_NMDA16 = 50;
 static double kNi0_NMDA16 = 0.0618;
 static double kNo0_NMDA16 = 110;
 static double kP0_NMDA16 = 1100;
 static double kfB0_NMDA16 = 0.175;
 static double kfF0_NMDA16 = 2.836;
 static double ksB0_NMDA16 = 0.23;
 static double ksF0_NMDA16 = 0.048;
 static double koff_NMDA16 = 0.0381;
 static double kon_NMDA16 = 2.83;
#define kfB_NMDA16 _thread1data[1]
#define kfB _thread[_gth]._pval[1]
#define kfF_NMDA16 _thread1data[2]
#define kfF _thread[_gth]._pval[2]
#define ksB_NMDA16 _thread1data[3]
#define ksB _thread[_gth]._pval[3]
#define ksF_NMDA16 _thread1data[4]
#define ksF _thread[_gth]._pval[4]
#define kMgB_NMDA16 _thread1data[5]
#define kMgB _thread[_gth]._pval[5]
#define kMgF_NMDA16 _thread1data[6]
#define kMgF _thread[_gth]._pval[6]
#define kNi_NMDA16 _thread1data[7]
#define kNi _thread[_gth]._pval[7]
#define kNo_NMDA16 _thread1data[8]
#define kNo _thread[_gth]._pval[8]
#define kP_NMDA16 _thread1data[9]
#define kP _thread[_gth]._pval[9]
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gmax_NMDA16", "pS",
 "Mg_NMDA16", "mM",
 "kon_NMDA16", "/ms",
 "koff_NMDA16", "/ms",
 "ksF0_NMDA16", "/ms",
 "ksB0_NMDA16", "/ms",
 "kfF0_NMDA16", "/ms",
 "kfB0_NMDA16", "/ms",
 "Vdep_NMDA16", "mV",
 "V0_NMDA16", "mV",
 "Kna_NMDA16", "mM",
 "Kcs0_NMDA16", "mM",
 "a_NMDA16", "mV",
 "kP0_NMDA16", "/ms",
 "b_NMDA16", "mV",
 "kNo0_NMDA16", "/ms",
 "c_NMDA16", "mV",
 "kNi0_NMDA16", "/ms",
 "d_NMDA16", "mV",
 "ksF_NMDA16", "/ms",
 "ksB_NMDA16", "/ms",
 "kfF_NMDA16", "/ms",
 "kfB_NMDA16", "/ms",
 "kMgF_NMDA16", "/ms /mM",
 "kMgB_NMDA16", "/ms",
 "Kcs_NMDA16", "mM",
 "kP_NMDA16", "/ms /mM",
 "kNo_NMDA16", "/ms",
 "kNi_NMDA16", "/ms",
 "Erev", "mV",
 "tau", "ms",
 "T_max", "mM",
 "kd1F", "/ms",
 "kd1B", "/ms",
 "kd2F", "/ms",
 "kd2B", "/ms",
 "csi", "mM",
 "i", "nA",
 "g", "uS",
 "T", "mM",
 "tRel", "ms",
 0,0
};
 
#endif /*BBCORE*/
 static double OMg0 = 0;
 static double O0 = 0;
 static double RA2sMg0 = 0;
 static double RA2fMg0 = 0;
 static double RA2d2Mg0 = 0;
 static double RA2d1Mg0 = 0;
 static double RA2Mg0 = 0;
 static double RAMg0 = 0;
 static double RMg0 = 0;
 static double RA2s0 = 0;
 static double RA2f0 = 0;
 static double RA2d20 = 0;
 static double RA2d10 = 0;
 static double RA20 = 0;
 static double RA0 = 0;
 static double R0 = 0;
 static double delta_t = 1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "gmax_NMDA16", &gmax_NMDA16,
 "Mg_NMDA16", &Mg_NMDA16,
 "kon_NMDA16", &kon_NMDA16,
 "koff_NMDA16", &koff_NMDA16,
 "ksF0_NMDA16", &ksF0_NMDA16,
 "ksB0_NMDA16", &ksB0_NMDA16,
 "kfF0_NMDA16", &kfF0_NMDA16,
 "kfB0_NMDA16", &kfB0_NMDA16,
 "Vdep_NMDA16", &Vdep_NMDA16,
 "V0_NMDA16", &V0_NMDA16,
 "Kna_NMDA16", &Kna_NMDA16,
 "Kcs0_NMDA16", &Kcs0_NMDA16,
 "a_NMDA16", &a_NMDA16,
 "kP0_NMDA16", &kP0_NMDA16,
 "b_NMDA16", &b_NMDA16,
 "kNo0_NMDA16", &kNo0_NMDA16,
 "c_NMDA16", &c_NMDA16,
 "kNi0_NMDA16", &kNi0_NMDA16,
 "d_NMDA16", &d_NMDA16,
 "ksF_NMDA16", &ksF_NMDA16,
 "ksB_NMDA16", &ksB_NMDA16,
 "kfF_NMDA16", &kfF_NMDA16,
 "kfB_NMDA16", &kfB_NMDA16,
 "kMgF_NMDA16", &kMgF_NMDA16,
 "kMgB_NMDA16", &kMgB_NMDA16,
 "Kcs_NMDA16", &Kcs_NMDA16,
 "kP_NMDA16", &kP_NMDA16,
 "kNo_NMDA16", &kNo_NMDA16,
 "kNi_NMDA16", &kNi_NMDA16,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void _create_global_variables(NrnThread*, Memb_list*, int);
 static void _destroy_global_variables(NrnThread*, Memb_list*, int);
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
"NMDA16",
 "Erev",
 "tau",
 "T_max",
 "kd1F",
 "kd1B",
 "kd2F",
 "kd2B",
 "csi",
 0,
 "i",
 "g",
 "T",
 "tRel",
 "synon",
 0,
 "R",
 "RA",
 "RA2",
 "RA2d1",
 "RA2d2",
 "RA2f",
 "RA2s",
 "O",
 "OMg",
 "RMg",
 "RAMg",
 "RA2Mg",
 "RA2d1Mg",
 "RA2d2Mg",
 "RA2fMg",
 "RA2sMg",
 0,
 0};
 static int _na_type;
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	Erev = 0;
 	tau = 0.3;
 	T_max = 1.5;
 	kd1F = 0.001;
 	kd1B = 0.001;
 	kd2F = 0.001;
 	kd2B = 0.001;
 	csi = 148;
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[2]; /* nao */
 
#endif /* BBCORE */
 
}
 static void _initlists(Memb_list *_ml);
 void _net_receive(Point_process*, int, double);
 static void _thread_mem_init(ThreadDatum*);
 static void _thread_cleanup(ThreadDatum*);
 static void _update_ion_pointer(Datum*);
 
#define _psize 50
#define _ppsize 3
 void _SynNMDA16_reg() {
	int _vectorized = 1;
  _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 _na_type = nrn_get_mechtype("na_ion"); 
#if 0 /*BBCORE*/
 	ion_reg("na", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 
#endif /*BBCORE*/
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init, _create_global_variables, _destroy_global_variables,
	 hoc_nrnpointerindex,
	 NULL/*_hoc_create_pnt*/, NULL/*_hoc_destroy_pnt*/, /*_member_func,*/
	 4);
  _extcall_thread = (ThreadDatum*)ecalloc(3, sizeof(ThreadDatum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
     _nrn_thread_reg1(_mechtype, _thread_mem_init);
     _nrn_thread_reg0(_mechtype, _thread_cleanup);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
 
#if NET_RECEIVE_BUFFERING
  hoc_register_net_receive_buffering(_net_buf_receive, _mechtype);
#endif
 set_pnt_receive(_mechtype, _net_receive, nullptr, 1);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, NULL);
 }
 struct _global_variables_t : public MemoryManaged {
   int _slist1[16];
   int _dlist1[16];
   double celsius;
   double Kcs0;
   double Kna;
   double Mg;
   double V0;
   double Vdep;
   double a;
   double b;
   double c;
   double d;
   double gmax;
   double kNi0;
   double kNo0;
   double kP0;
   double kfB0;
   double kfF0;
   double ksB0;
   double ksF0;
   double koff;
   double kon;
   double OMg0;
   double O0;
   double RA2sMg0;
   double RA2fMg0;
   double RA2d2Mg0;
   double RA2d1Mg0;
   double RA2Mg0;
   double RAMg0;
   double RMg0;
   double RA2s0;
   double RA2f0;
   double RA2d20;
   double RA2d10;
   double RA20;
   double RA0;
   double R0;
   double delta_t;
 };

 
static void _create_global_variables(NrnThread *_nt, Memb_list *_ml, int _type) {
   assert(!_ml->global_variables);
   _ml->global_variables = new _global_variables_t{};
   _ml->global_variables_size = sizeof(_global_variables_t);
 }
 
static void _destroy_global_variables(NrnThread *_nt, Memb_list *_ml, int _type) {
   delete static_cast<_global_variables_t*>(_ml->global_variables);
   _ml->global_variables = nullptr;
   _ml->global_variables_size = 0;
 }
 
static void _update_global_variables(NrnThread *_nt, Memb_list *_ml) {
   if(!_nt || !_ml) {
     return;
   }
   auto* const _global_variables = static_cast<_global_variables_t*>(_ml->global_variables);
   _global_variables->celsius = celsius;
   _global_variables->Kcs0 = Kcs0_NMDA16;
   _global_variables->Kna = Kna_NMDA16;
   _global_variables->Mg = Mg_NMDA16;
   _global_variables->V0 = V0_NMDA16;
   _global_variables->Vdep = Vdep_NMDA16;
   _global_variables->a = a_NMDA16;
   _global_variables->b = b_NMDA16;
   _global_variables->c = c_NMDA16;
   _global_variables->d = d_NMDA16;
   _global_variables->gmax = gmax_NMDA16;
   _global_variables->kNi0 = kNi0_NMDA16;
   _global_variables->kNo0 = kNo0_NMDA16;
   _global_variables->kP0 = kP0_NMDA16;
   _global_variables->kfB0 = kfB0_NMDA16;
   _global_variables->kfF0 = kfF0_NMDA16;
   _global_variables->ksB0 = ksB0_NMDA16;
   _global_variables->ksF0 = ksF0_NMDA16;
   _global_variables->koff = koff_NMDA16;
   _global_variables->kon = kon_NMDA16;
   _global_variables->OMg0 = OMg0;
   _global_variables->O0 = O0;
   _global_variables->RA2sMg0 = RA2sMg0;
   _global_variables->RA2fMg0 = RA2fMg0;
   _global_variables->RA2d2Mg0 = RA2d2Mg0;
   _global_variables->RA2d1Mg0 = RA2d1Mg0;
   _global_variables->RA2Mg0 = RA2Mg0;
   _global_variables->RAMg0 = RAMg0;
   _global_variables->RMg0 = RMg0;
   _global_variables->RA2s0 = RA2s0;
   _global_variables->RA2f0 = RA2f0;
   _global_variables->RA2d20 = RA2d20;
   _global_variables->RA2d10 = RA2d10;
   _global_variables->RA20 = RA20;
   _global_variables->RA0 = RA0;
   _global_variables->R0 = R0;
   _global_variables->delta_t = delta_t;
 #ifdef CORENEURON_ENABLE_GPU
   if (_nt->compute_gpu) {
       cnrn_target_update_on_device(_global_variables);
   }
 #endif
 }

 #define _slist1 static_cast<_global_variables_t*>(_ml->global_variables)->_slist1
 #define _dlist1 static_cast<_global_variables_t*>(_ml->global_variables)->_dlist1
 #define celsius static_cast<_global_variables_t*>(_ml->global_variables)->celsius
 #define Kcs0 static_cast<_global_variables_t*>(_ml->global_variables)->Kcs0
 #define Kna static_cast<_global_variables_t*>(_ml->global_variables)->Kna
 #define Mg static_cast<_global_variables_t*>(_ml->global_variables)->Mg
 #define V0 static_cast<_global_variables_t*>(_ml->global_variables)->V0
 #define Vdep static_cast<_global_variables_t*>(_ml->global_variables)->Vdep
 #define a static_cast<_global_variables_t*>(_ml->global_variables)->a
 #define b static_cast<_global_variables_t*>(_ml->global_variables)->b
 #define c static_cast<_global_variables_t*>(_ml->global_variables)->c
 #define d static_cast<_global_variables_t*>(_ml->global_variables)->d
 #define gmax static_cast<_global_variables_t*>(_ml->global_variables)->gmax
 #define kNi0 static_cast<_global_variables_t*>(_ml->global_variables)->kNi0
 #define kNo0 static_cast<_global_variables_t*>(_ml->global_variables)->kNo0
 #define kP0 static_cast<_global_variables_t*>(_ml->global_variables)->kP0
 #define kfB0 static_cast<_global_variables_t*>(_ml->global_variables)->kfB0
 #define kfF0 static_cast<_global_variables_t*>(_ml->global_variables)->kfF0
 #define ksB0 static_cast<_global_variables_t*>(_ml->global_variables)->ksB0
 #define ksF0 static_cast<_global_variables_t*>(_ml->global_variables)->ksF0
 #define koff static_cast<_global_variables_t*>(_ml->global_variables)->koff
 #define kon static_cast<_global_variables_t*>(_ml->global_variables)->kon
 #define OMg0 static_cast<_global_variables_t*>(_ml->global_variables)->OMg0
 #define O0 static_cast<_global_variables_t*>(_ml->global_variables)->O0
 #define RA2sMg0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2sMg0
 #define RA2fMg0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2fMg0
 #define RA2d2Mg0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2d2Mg0
 #define RA2d1Mg0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2d1Mg0
 #define RA2Mg0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2Mg0
 #define RAMg0 static_cast<_global_variables_t*>(_ml->global_variables)->RAMg0
 #define RMg0 static_cast<_global_variables_t*>(_ml->global_variables)->RMg0
 #define RA2s0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2s0
 #define RA2f0 static_cast<_global_variables_t*>(_ml->global_variables)->RA2f0
 #define RA2d20 static_cast<_global_variables_t*>(_ml->global_variables)->RA2d20
 #define RA2d10 static_cast<_global_variables_t*>(_ml->global_variables)->RA2d10
 #define RA20 static_cast<_global_variables_t*>(_ml->global_variables)->RA20
 #define RA0 static_cast<_global_variables_t*>(_ml->global_variables)->RA0
 #define R0 static_cast<_global_variables_t*>(_ml->global_variables)->R0
 #define delta_t static_cast<_global_variables_t*>(_ml->global_variables)->delta_t
 
static const char *modelname = "Voltage-dependent kinetic model of NMDA receptor";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static inline int rates(_threadargsprotocomma_ double, double);
 
constexpr coreneuron::scopmath::enabled_code code_to_enable{coreneuron::scopmath::enabled_code::all};
#define _MATELM1(_row,_col) coreneuron::scopmath::sparse::thread_getelm<code_to_enable>(static_cast<SparseObj*>(_so), _row + 1, _col + 1, _iml)[_iml]
 
#define _RHS1(_arg) _rhs[(_arg+1)*_STRIDE]
  
#define _linmat1  1
 static constexpr int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 
/* _kinetic_ kstates _NMDA16 */
#ifndef INSIDE_NMODL
#define INSIDE_NMODL
#endif
 
template <coreneuron::scopmath::enabled_code code_to_enable = coreneuron::scopmath::enabled_code::all>
struct kstates_NMDA16 {
  int operator()(SparseObj* _so, double* _rhs, _threadargsproto_) const;
};
template <coreneuron::scopmath::enabled_code code_to_enable>
int kstates_NMDA16<code_to_enable>::operator() (SparseObj* _so, double* _rhs, _threadargsproto_) const
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<16;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]*_STRIDE] - _p[_dlist1[_i]*_STRIDE]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v , t ) ;
   /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 f_flux =  ( 2.0 * kon * T ) * R ;
 b_flux =  koff * RA ;
 _RHS1( 15) -= (f_flux - b_flux);
 _RHS1( 14) += (f_flux - b_flux);
 
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  koff ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 f_flux =  ( kon * T ) * RA ;
 b_flux =  ( 2.0 * koff ) * RA2 ;
 _RHS1( 14) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  ( kon * T ) ;
 _MATELM1( 14 ,14)  += _term;
 _MATELM1( 13 ,14)  -= _term;
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 14 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 f_flux =  kd1F * RA2 ;
 b_flux =  kd1B * RA2d1 ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  kd1F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  kd1B ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 f_flux =  kd2F * RA2 ;
 b_flux =  kd2B * RA2d2 ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 11) += (f_flux - b_flux);
 
 _term =  kd2F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 11 ,13)  -= _term;
 _term =  kd2B ;
 _MATELM1( 13 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 f_flux =  kfF * RA2 ;
 b_flux =  kfB * RA2f ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  kfF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 10 ,13)  -= _term;
 _term =  kfB ;
 _MATELM1( 13 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 f_flux =  ksF * RA2 ;
 b_flux =  ksB * RA2s ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  ksF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 9 ,13)  -= _term;
 _term =  ksB ;
 _MATELM1( 13 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 f_flux =  ksF * RA2f ;
 b_flux =  ksB * O ;
 _RHS1( 10) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  ksF ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 2 ,10)  -= _term;
 _term =  ksB ;
 _MATELM1( 10 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 f_flux =  kfF * RA2s ;
 b_flux =  kfB * O ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  kfF ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  kfB ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 f_flux =  ( kMgF * Mg ) * O ;
 b_flux =  kMgB * OMg ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  ( kMgF * Mg ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  kMgB ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 f_flux =  ksB * OMg ;
 b_flux =  ksF * RA2fMg ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  ksB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  ksF ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 f_flux =  kfB * OMg ;
 b_flux =  kfF * RA2sMg ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  kfB ;
 _MATELM1( 1 ,1)  += _term;
 _term =  kfF ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 f_flux =  kfB * RA2fMg ;
 b_flux =  kfF * RA2Mg ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  kfB ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 6 ,3)  -= _term;
 _term =  kfF ;
 _MATELM1( 3 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 f_flux =  ksB * RA2sMg ;
 b_flux =  ksF * RA2Mg ;
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  ksB ;
 _MATELM1( 6 ,0)  -= _term;
 _term =  ksF ;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 f_flux =  kd1B * RA2Mg ;
 b_flux =  kd1F * RA2d1Mg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  kd1B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  kd1F ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 f_flux =  kd2B * RA2Mg ;
 b_flux =  kd2F * RA2d2Mg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  kd2B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  kd2F ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 f_flux =  ( 2.0 * koff ) * RA2Mg ;
 b_flux =  ( kon * T ) * RAMg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 7 ,6)  -= _term;
 _term =  ( kon * T ) ;
 _MATELM1( 6 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 f_flux =  koff * RAMg ;
 b_flux =  ( 2.0 * kon * T ) * RMg ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  koff ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 8 ,7)  -= _term;
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 7 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= RA2sMg ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= RA2fMg ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= RA2d2Mg ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= RA2d1Mg ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= RA2Mg ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= RAMg ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= RMg ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= OMg ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= RA2s ;
 _MATELM1(0, 10) = 1;
 _RHS1(0) -= RA2f ;
 _MATELM1(0, 11) = 1;
 _RHS1(0) -= RA2d2 ;
 _MATELM1(0, 12) = 1;
 _RHS1(0) -= RA2d1 ;
 _MATELM1(0, 13) = 1;
 _RHS1(0) -= RA2 ;
 _MATELM1(0, 14) = 1;
 _RHS1(0) -= RA ;
 _MATELM1(0, 15) = 1;
 _RHS1(0) -= R ;
 /*CONSERVATION*/
   } return _reset;
 }
 
#if NET_RECEIVE_BUFFERING 
#undef t
#define t _nrb_t
static inline void _net_receive_kernel(NrnThread*, double, Point_process*, int _weight_index, double _flag);
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
      _net_receive_kernel(nrn_threads, _nrt, _pnt + _j, _k, _nrflag);
    }
  }
  #pragma acc wait(stream_id)
  _nrb->_displ_cnt = 0;
  _nrb->_cnt = 0;
  /*printf("_net_buf_receive__NMDA16  %d\n", _nt->_id);*/
 
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
 
static void _net_receive_kernel(NrnThread* nrn_threads, double _nrb_t, Point_process* _pnt, int _weight_index, double _lflag)
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
   _args = _weights ? _weights + _weight_index : nullptr;
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
 
static int  rates ( _threadargsprotocomma_ double _lv , double _lt ) {
   T = T_max * ( _lt - tRel ) / tau * exp ( 1.0 - ( _lt - tRel ) / tau ) * synon ;
   Kcs = Kcs0 * exp ( _lv / a ) ;
   kP = kP0 * exp ( _lv / b ) ;
   kNo = kNo0 * exp ( _lv / c ) ;
   kNi = kNi0 * exp ( _lv / d ) ;
   kMgF = kP / ( ( 1.0 + nao / Kna ) * ( 1.0 + nao / Kna + csi / Kcs ) ) ;
   kMgB = kNo / pow( ( 1.0 + nao / Kna ) , 2.0 ) + kNi ;
   ksF = ksF0 * exp ( ( _lv - V0 ) / Vdep ) ;
   ksB = ksB0 * exp ( ( _lv - V0 ) / Vdep * ( - 1.0 ) ) ;
   kfF = kfF0 * exp ( ( _lv - V0 ) / Vdep ) ;
   kfB = kfB0 * exp ( ( _lv - V0 ) / Vdep * ( - 1.0 ) ) ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static double _hoc_rates(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 rates ( _threadargs_, *getarg(1) , *getarg(2) );
 return(_r);
}
 
#endif /*BBCORE*/
 
/*CVODE ode begin*/
 static int _ode_spec1(_threadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<16;_i++) _p[_dlist1[_i]] = 0.0;}
 rates ( _threadargscomma_ v , t ) ;
 /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 f_flux =  ( 2.0 * kon * T ) * R ;
 b_flux =  koff * RA ;
 DR -= (f_flux - b_flux);
 DRA += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 f_flux =  ( kon * T ) * RA ;
 b_flux =  ( 2.0 * koff ) * RA2 ;
 DRA -= (f_flux - b_flux);
 DRA2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 f_flux =  kd1F * RA2 ;
 b_flux =  kd1B * RA2d1 ;
 DRA2 -= (f_flux - b_flux);
 DRA2d1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 f_flux =  kd2F * RA2 ;
 b_flux =  kd2B * RA2d2 ;
 DRA2 -= (f_flux - b_flux);
 DRA2d2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 f_flux =  kfF * RA2 ;
 b_flux =  kfB * RA2f ;
 DRA2 -= (f_flux - b_flux);
 DRA2f += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 f_flux =  ksF * RA2 ;
 b_flux =  ksB * RA2s ;
 DRA2 -= (f_flux - b_flux);
 DRA2s += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 f_flux =  ksF * RA2f ;
 b_flux =  ksB * O ;
 DRA2f -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 f_flux =  kfF * RA2s ;
 b_flux =  kfB * O ;
 DRA2s -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 f_flux =  ( kMgF * Mg ) * O ;
 b_flux =  kMgB * OMg ;
 DO -= (f_flux - b_flux);
 DOMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 f_flux =  ksB * OMg ;
 b_flux =  ksF * RA2fMg ;
 DOMg -= (f_flux - b_flux);
 DRA2fMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 f_flux =  kfB * OMg ;
 b_flux =  kfF * RA2sMg ;
 DOMg -= (f_flux - b_flux);
 DRA2sMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 f_flux =  kfB * RA2fMg ;
 b_flux =  kfF * RA2Mg ;
 DRA2fMg -= (f_flux - b_flux);
 DRA2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 f_flux =  ksB * RA2sMg ;
 b_flux =  ksF * RA2Mg ;
 DRA2sMg -= (f_flux - b_flux);
 DRA2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 f_flux =  kd1B * RA2Mg ;
 b_flux =  kd1F * RA2d1Mg ;
 DRA2Mg -= (f_flux - b_flux);
 DRA2d1Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 f_flux =  kd2B * RA2Mg ;
 b_flux =  kd2F * RA2d2Mg ;
 DRA2Mg -= (f_flux - b_flux);
 DRA2d2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 f_flux =  ( 2.0 * koff ) * RA2Mg ;
 b_flux =  ( kon * T ) * RAMg ;
 DRA2Mg -= (f_flux - b_flux);
 DRAMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 f_flux =  koff * RAMg ;
 b_flux =  ( 2.0 * kon * T ) * RMg ;
 DRAMg -= (f_flux - b_flux);
 DRMg += (f_flux - b_flux);
 
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, _threadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<16;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v , t ) ;
 /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  koff ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 _term =  ( kon * T ) ;
 _MATELM1( 14 ,14)  += _term;
 _MATELM1( 13 ,14)  -= _term;
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 14 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 _term =  kd1F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  kd1B ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 _term =  kd2F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 11 ,13)  -= _term;
 _term =  kd2B ;
 _MATELM1( 13 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 _term =  kfF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 10 ,13)  -= _term;
 _term =  kfB ;
 _MATELM1( 13 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 _term =  ksF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 9 ,13)  -= _term;
 _term =  ksB ;
 _MATELM1( 13 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 _term =  ksF ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 2 ,10)  -= _term;
 _term =  ksB ;
 _MATELM1( 10 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 _term =  kfF ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  kfB ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 _term =  ( kMgF * Mg ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  kMgB ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 _term =  ksB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  ksF ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 _term =  kfB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  kfF ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 _term =  kfB ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 6 ,3)  -= _term;
 _term =  kfF ;
 _MATELM1( 3 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 _term =  ksB ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 6 ,0)  -= _term;
 _term =  ksF ;
 _MATELM1( 0 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 _term =  kd1B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  kd1F ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 _term =  kd2B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  kd2F ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 7 ,6)  -= _term;
 _term =  ( kon * T ) ;
 _MATELM1( 6 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 _term =  koff ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 8 ,7)  -= _term;
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 7 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static void _thread_mem_init(ThreadDatum* _thread) {
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(10, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(ThreadDatum* _thread) {
   _nrn_destroy_sparseobj_thread((SparseObj*)_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread((SparseObj*)_thread[_spth1]._pvoid);
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 static void _update_ion_pointer(Datum* _ppvar) {
 }

static inline void initmodel(_threadargsproto_) {
  int _i; double _save;{
  OMg = OMg0;
  O = O0;
  RA2sMg = RA2sMg0;
  RA2fMg = RA2fMg0;
  RA2d2Mg = RA2d2Mg0;
  RA2d1Mg = RA2d1Mg0;
  RA2Mg = RA2Mg0;
  RAMg = RAMg0;
  RMg = RMg0;
  RA2s = RA2s0;
  RA2f = RA2f0;
  RA2d2 = RA2d20;
  RA2d1 = RA2d10;
  RA2 = RA20;
  RA = RA0;
  R = R0;
 {
   T = 0.0 ;
   synon = 0.0 ;
   tRel = 0.0 ;
   R = 1.0 ;
   rates ( _threadargscomma_ v , t ) ;
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
  assert(_ml->global_variables);
  assert(_ml->global_variables_size != 0);
  _initlists(_ml);
  _update_global_variables(_nt, _ml);
  if (!_thread[_spth1]._pvoid) {
    _thread[_spth1]._pvoid = nrn_cons_sparseobj(kstates_NMDA16{}, 16, _ml, _threadargs_);
    #ifdef _OPENACC
    if (_nt->compute_gpu) {
      void* _d_so = cnrn_target_deviceptr(_thread[_spth1]._pvoid);
      ThreadDatum* _d_td = cnrn_target_deviceptr(_thread);
      cnrn_target_memcpy_to_device(&(_d_td[_spth1]._pvoid), &_d_so);
    }
    #endif
  }
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
  nao = _ion_nao;
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
  NrnThread* d_nt = cnrn_target_deviceptr(_nt);
  Memb_list* d_ml = cnrn_target_deviceptr(_ml);
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
  nao = _ion_nao;
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
  NrnThread* d_nt = cnrn_target_deviceptr(_nt);
  Memb_list* d_ml = cnrn_target_deviceptr(_ml);
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
  nao = _ion_nao;
 {  
  sparse_thread(static_cast<SparseObj*>(_thread[_spth1]._pvoid), 16, _slist1, _dlist1, &t, dt, kstates_NMDA16<coreneuron::scopmath::enabled_code::compute_only>{}, _linmat1, _threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(Memb_list *_ml){
 double _x; double* _p = &_x;
 int _i;
 int _cntml_actual=1;
 int _cntml_padded=1;
 int _iml=0;
 _slist1[0] = &(RA2sMg) - _p;  _dlist1[0] = &(DRA2sMg) - _p;
 _slist1[1] = &(OMg) - _p;  _dlist1[1] = &(DOMg) - _p;
 _slist1[2] = &(O) - _p;  _dlist1[2] = &(DO) - _p;
 _slist1[3] = &(RA2fMg) - _p;  _dlist1[3] = &(DRA2fMg) - _p;
 _slist1[4] = &(RA2d2Mg) - _p;  _dlist1[4] = &(DRA2d2Mg) - _p;
 _slist1[5] = &(RA2d1Mg) - _p;  _dlist1[5] = &(DRA2d1Mg) - _p;
 _slist1[6] = &(RA2Mg) - _p;  _dlist1[6] = &(DRA2Mg) - _p;
 _slist1[7] = &(RAMg) - _p;  _dlist1[7] = &(DRAMg) - _p;
 _slist1[8] = &(RMg) - _p;  _dlist1[8] = &(DRMg) - _p;
 _slist1[9] = &(RA2s) - _p;  _dlist1[9] = &(DRA2s) - _p;
 _slist1[10] = &(RA2f) - _p;  _dlist1[10] = &(DRA2f) - _p;
 _slist1[11] = &(RA2d2) - _p;  _dlist1[11] = &(DRA2d2) - _p;
 _slist1[12] = &(RA2d1) - _p;  _dlist1[12] = &(DRA2d1) - _p;
 _slist1[13] = &(RA2) - _p;  _dlist1[13] = &(DRA2) - _p;
 _slist1[14] = &(RA) - _p;  _dlist1[14] = &(DRA) - _p;
 _slist1[15] = &(R) - _p;  _dlist1[15] = &(DR) - _p;
 }
} // namespace coreneuron_lib
