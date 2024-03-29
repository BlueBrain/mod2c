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
 
#define _thread_present_ /**/ , _thread[0:2] 
 
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
 
#define nrn_init _nrn_init__NMDA10_2
#define nrn_cur _nrn_cur__NMDA10_2
#define _nrn_current _nrn_current__NMDA10_2
#define nrn_jacob _nrn_jacob__NMDA10_2
#define nrn_state _nrn_state__NMDA10_2
#define initmodel initmodel__NMDA10_2
#define _net_receive _net_receive__NMDA10_2
#define _net_init _net_init__NMDA10_2
#define nrn_state_launcher nrn_state_NMDA10_2_launcher
#define nrn_cur_launcher nrn_cur_NMDA10_2_launcher
#define nrn_jacob_launcher nrn_jacob_NMDA10_2_launcher 
#if NET_RECEIVE_BUFFERING
#define _net_buf_receive _net_buf_receive_NMDA10_2
void _net_buf_receive(NrnThread*);
#endif
 
#define kstates kstates_NMDA10_2 
#define release release_NMDA10_2 
#define rates rates_NMDA10_2 
 
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
#define gmax _p[1*_STRIDE]
#define tau _p[2*_STRIDE]
#define T_max _p[3*_STRIDE]
#define i _p[4*_STRIDE]
#define g _p[5*_STRIDE]
#define rb _p[6*_STRIDE]
#define rbb _p[7*_STRIDE]
#define RMgB _p[8*_STRIDE]
#define RMgU _p[9*_STRIDE]
#define T _p[10*_STRIDE]
#define tRel _p[11*_STRIDE]
#define synon _p[12*_STRIDE]
#define C0 _p[13*_STRIDE]
#define C1 _p[14*_STRIDE]
#define C2 _p[15*_STRIDE]
#define D _p[16*_STRIDE]
#define O _p[17*_STRIDE]
#define CB0 _p[18*_STRIDE]
#define CB1 _p[19*_STRIDE]
#define CB2 _p[20*_STRIDE]
#define DB _p[21*_STRIDE]
#define OB _p[22*_STRIDE]
#define w _p[23*_STRIDE]
#define DC0 _p[24*_STRIDE]
#define DC1 _p[25*_STRIDE]
#define DC2 _p[26*_STRIDE]
#define DD _p[27*_STRIDE]
#define DO _p[28*_STRIDE]
#define DCB0 _p[29*_STRIDE]
#define DCB1 _p[30*_STRIDE]
#define DCB2 _p[31*_STRIDE]
#define DDB _p[32*_STRIDE]
#define DOB _p[33*_STRIDE]
#define _v_unused _p[34*_STRIDE]
#define _g_unused _p[35*_STRIDE]
#define _tsav _p[36*_STRIDE]
 
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
 extern double celsius;
 #define nrn_ghk(v, ci, co, z) nrn_ghk(v, ci, co, z, celsius)
 #if 0 /*BBCORE*/
 /* declaration of user functions */
 static double _hoc_release();
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
 "release", _hoc_release,
 "rates", _hoc_rates,
 0, 0
};
 
#endif /*BBCORE*/
 /* declare global and static user variables */
 static double Rc_NMDA10_2 = 0.0916;
 static double Ro_NMDA10_2 = 0.0465;
 static double Rr_NMDA10_2 = 0.0018;
 static double Rd_NMDA10_2 = 0.0084;
 static double Ru_NMDA10_2 = 0.0055;
 static double Rb_NMDA10_2 = 5;
 static double mg_NMDA10_2 = 1;
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "mg_NMDA10_2", "mM",
 "Rb_NMDA10_2", "/mM",
 "Ru_NMDA10_2", "/ms",
 "Rd_NMDA10_2", "/ms",
 "Rr_NMDA10_2", "/ms",
 "Ro_NMDA10_2", "/ms",
 "Rc_NMDA10_2", "/ms",
 "Erev", "mV",
 "gmax", "pS",
 "tau", "ms",
 "T_max", "mM",
 "i", "nA",
 "g", "uS",
 "rb", "/ms",
 "rbb", "/ms",
 "RMgB", "/ms",
 "RMgU", "/ms",
 "T", "mM",
 "tRel", "ms",
 0,0
};
 
#endif /*BBCORE*/
 static double CB20 = 0;
 static double CB10 = 0;
 static double CB00 = 0;
 static double C20 = 0;
 static double C10 = 0;
 static double C00 = 0;
 static double DB0 = 0;
 static double D0 = 0;
 static double OB0 = 0;
 static double O0 = 0;
 static double delta_t = 1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "mg_NMDA10_2", &mg_NMDA10_2,
 "Rb_NMDA10_2", &Rb_NMDA10_2,
 "Ru_NMDA10_2", &Ru_NMDA10_2,
 "Rd_NMDA10_2", &Rd_NMDA10_2,
 "Rr_NMDA10_2", &Rr_NMDA10_2,
 "Ro_NMDA10_2", &Ro_NMDA10_2,
 "Rc_NMDA10_2", &Rc_NMDA10_2,
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
"NMDA10_2",
 "Erev",
 "gmax",
 "tau",
 "T_max",
 0,
 "i",
 "g",
 "rb",
 "rbb",
 "RMgB",
 "RMgU",
 "T",
 "tRel",
 "synon",
 0,
 "C0",
 "C1",
 "C2",
 "D",
 "O",
 "CB0",
 "CB1",
 "CB2",
 "DB",
 "OB",
 0,
 0};
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	Erev = -0.7;
 	gmax = 50;
 	tau = 0.3;
 	T_max = 1.5;
 
#endif /* BBCORE */
 
}
 static void _initlists(Memb_list *_ml);
 void _net_receive(Point_process*, int, double);
 static void _thread_cleanup(ThreadDatum*);
 
#define _psize 37
#define _ppsize 2
 void _SynNMDA10_2_reg() {
	int _vectorized = 1;
  _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 
#if 0 /*BBCORE*/
 
#endif /*BBCORE*/
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init, _create_global_variables, _destroy_global_variables,
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
 struct _global_variables_t : public MemoryManaged {
   int _slist1[10];
   int _dlist1[10];
   double celsius;
   double Rc;
   double Ro;
   double Rr;
   double Rd;
   double Ru;
   double Rb;
   double mg;
   double CB20;
   double CB10;
   double CB00;
   double C20;
   double C10;
   double C00;
   double DB0;
   double D0;
   double OB0;
   double O0;
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
   _global_variables->Rc = Rc_NMDA10_2;
   _global_variables->Ro = Ro_NMDA10_2;
   _global_variables->Rr = Rr_NMDA10_2;
   _global_variables->Rd = Rd_NMDA10_2;
   _global_variables->Ru = Ru_NMDA10_2;
   _global_variables->Rb = Rb_NMDA10_2;
   _global_variables->mg = mg_NMDA10_2;
   _global_variables->CB20 = CB20;
   _global_variables->CB10 = CB10;
   _global_variables->CB00 = CB00;
   _global_variables->C20 = C20;
   _global_variables->C10 = C10;
   _global_variables->C00 = C00;
   _global_variables->DB0 = DB0;
   _global_variables->D0 = D0;
   _global_variables->OB0 = OB0;
   _global_variables->O0 = O0;
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
 #define Rc static_cast<_global_variables_t*>(_ml->global_variables)->Rc
 #define Ro static_cast<_global_variables_t*>(_ml->global_variables)->Ro
 #define Rr static_cast<_global_variables_t*>(_ml->global_variables)->Rr
 #define Rd static_cast<_global_variables_t*>(_ml->global_variables)->Rd
 #define Ru static_cast<_global_variables_t*>(_ml->global_variables)->Ru
 #define Rb static_cast<_global_variables_t*>(_ml->global_variables)->Rb
 #define mg static_cast<_global_variables_t*>(_ml->global_variables)->mg
 #define CB20 static_cast<_global_variables_t*>(_ml->global_variables)->CB20
 #define CB10 static_cast<_global_variables_t*>(_ml->global_variables)->CB10
 #define CB00 static_cast<_global_variables_t*>(_ml->global_variables)->CB00
 #define C20 static_cast<_global_variables_t*>(_ml->global_variables)->C20
 #define C10 static_cast<_global_variables_t*>(_ml->global_variables)->C10
 #define C00 static_cast<_global_variables_t*>(_ml->global_variables)->C00
 #define DB0 static_cast<_global_variables_t*>(_ml->global_variables)->DB0
 #define D0 static_cast<_global_variables_t*>(_ml->global_variables)->D0
 #define OB0 static_cast<_global_variables_t*>(_ml->global_variables)->OB0
 #define O0 static_cast<_global_variables_t*>(_ml->global_variables)->O0
 #define delta_t static_cast<_global_variables_t*>(_ml->global_variables)->delta_t
 
static const char *modelname = "detailed model of glutamate NMDA receptors";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static inline int release(_threadargsprotocomma_ double);
static inline int rates(_threadargsprotocomma_ double);
 
constexpr coreneuron::scopmath::enabled_code code_to_enable{coreneuron::scopmath::enabled_code::all};
#define _MATELM1(_row,_col) coreneuron::scopmath::sparse::thread_getelm<code_to_enable>(static_cast<SparseObj*>(_so), _row + 1, _col + 1, _iml)[_iml]
 
#define _RHS1(_arg) _rhs[(_arg+1)*_STRIDE]
  
#define _linmat1  1
 static constexpr int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 
/* _kinetic_ kstates _NMDA10_2 */
#ifndef INSIDE_NMODL
#define INSIDE_NMODL
#endif
 
template <coreneuron::scopmath::enabled_code code_to_enable = coreneuron::scopmath::enabled_code::all>
struct kstates_NMDA10_2 {
  int operator()(SparseObj* _so, double* _rhs, _threadargsproto_) const;
};
template <coreneuron::scopmath::enabled_code code_to_enable>
int kstates_NMDA10_2<code_to_enable>::operator() (SparseObj* _so, double* _rhs, _threadargsproto_) const
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
   rates ( _threadargscomma_ v ) ;
   /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 f_flux =  ( 2.0 * rb ) * C0 ;
 b_flux =  Ru * C1 ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Ru ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 f_flux =  rb * C1 ;
 b_flux =  ( 2.0 * Ru ) * C2 ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  rb ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 f_flux =  Rd * C2 ;
 b_flux =  Rr * D ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  Rd ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 8 ,4)  -= _term;
 _term =  Rr ;
 _MATELM1( 4 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 f_flux =  Ro * C2 ;
 b_flux =  Rc * O ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  Ro ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 9 ,4)  -= _term;
 _term =  Rc ;
 _MATELM1( 4 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 f_flux =  RMgB * O ;
 b_flux =  RMgU * OB ;
 _RHS1( 9) -= (f_flux - b_flux);
 
 _term =  RMgB ;
 _MATELM1( 9 ,9)  += _term;
 _term =  RMgU ;
 _MATELM1( 9 ,0)  -= _term;
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 f_flux =  ( 3.0 * Rc ) * OB ;
 b_flux =  Ro * CB2 ;
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  ( 3.0 * Rc ) ;
 _MATELM1( 1 ,0)  -= _term;
 _term =  Ro ;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 f_flux =  Rd * CB2 ;
 b_flux =  Rr * DB ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  Rd ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 7 ,1)  -= _term;
 _term =  Rr ;
 _MATELM1( 1 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 f_flux =  ( 2.0 * Ru ) * CB2 ;
 b_flux =  rb * CB1 ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  rb ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 f_flux =  Ru * CB1 ;
 b_flux =  ( 2.0 * rb ) * CB0 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  Ru ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= OB ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= DB ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= CB2 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= CB1 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= CB0 ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= D ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= C2 ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= C1 ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= C0 ;
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
  /*printf("_net_buf_receive__NMDA10_2  %d\n", _nt->_id);*/
 
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
 
static int  release ( _threadargsprotocomma_ double _lt ) {
   T = T_max * ( _lt - tRel ) / tau * exp ( 1.0 - ( _lt - tRel ) / tau ) * synon ;
   
/*VERBATIM*/
	return 0;
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
 
static int  rates ( _threadargsprotocomma_ double _lv ) {
   RMgB = 610e-3 * exp ( 1.0 * - _lv / 17.0 ) * ( mg / 1.0 ) * 1.0 ;
   RMgU = 5400e-3 * exp ( 1.0 * _lv / 47.0 ) * 1.0 ;
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
 rates ( _threadargs_, *getarg(1) );
 return(_r);
}
 
#endif /*BBCORE*/
 
/*CVODE ode begin*/
 static int _ode_spec1(_threadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<10;_i++) _p[_dlist1[_i]] = 0.0;}
 release ( _threadargscomma_ t ) ;
 rb = Rb * T ;
 rates ( _threadargscomma_ v ) ;
 /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 f_flux =  ( 2.0 * rb ) * C0 ;
 b_flux =  Ru * C1 ;
 DC0 -= (f_flux - b_flux);
 DC1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 f_flux =  rb * C1 ;
 b_flux =  ( 2.0 * Ru ) * C2 ;
 DC1 -= (f_flux - b_flux);
 DC2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 f_flux =  Rd * C2 ;
 b_flux =  Rr * D ;
 DC2 -= (f_flux - b_flux);
 DD += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 f_flux =  Ro * C2 ;
 b_flux =  Rc * O ;
 DC2 -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 f_flux =  RMgB * O ;
 b_flux =  RMgU * OB ;
 DO -= (f_flux - b_flux);
 DOB += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 f_flux =  ( 3.0 * Rc ) * OB ;
 b_flux =  Ro * CB2 ;
 DOB -= (f_flux - b_flux);
 DCB2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 f_flux =  Rd * CB2 ;
 b_flux =  Rr * DB ;
 DCB2 -= (f_flux - b_flux);
 DDB += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 f_flux =  ( 2.0 * Ru ) * CB2 ;
 b_flux =  rb * CB1 ;
 DCB2 -= (f_flux - b_flux);
 DCB1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 f_flux =  Ru * CB1 ;
 b_flux =  ( 2.0 * rb ) * CB0 ;
 DCB1 -= (f_flux - b_flux);
 DCB0 += (f_flux - b_flux);
 
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
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
 rates ( _threadargscomma_ v ) ;
 /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Ru ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 _term =  rb ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 _term =  Rd ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 8 ,4)  -= _term;
 _term =  Rr ;
 _MATELM1( 4 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 _term =  Ro ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 9 ,4)  -= _term;
 _term =  Rc ;
 _MATELM1( 4 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 _term =  RMgB ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 0 ,9)  -= _term;
 _term =  RMgU ;
 _MATELM1( 9 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 _term =  ( 3.0 * Rc ) ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  Ro ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 _term =  Rd ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 7 ,1)  -= _term;
 _term =  Rr ;
 _MATELM1( 1 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  rb ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 _term =  Ru ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
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
  CB2 = CB20;
  CB1 = CB10;
  CB0 = CB00;
  C2 = C20;
  C1 = C10;
  C0 = C00;
  DB = DB0;
  D = D0;
  OB = OB0;
  O = O0;
 {
   T = 0.0 ;
   synon = 0.0 ;
   tRel = 0.0 ;
   rates ( _threadargscomma_ v ) ;
   C0 = 1.0 ;
   C1 = 0.0 ;
   C2 = 0.0 ;
   D = 0.0 ;
   O = 0.0 ;
   CB0 = 0.0 ;
   CB1 = 0.0 ;
   CB2 = 0.0 ;
   DB = 0.0 ;
   OB = 0.0 ;
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
    _thread[_spth1]._pvoid = nrn_cons_sparseobj(kstates_NMDA10_2{}, 10, _ml, _threadargs_);
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
 {  
  sparse_thread(static_cast<SparseObj*>(_thread[_spth1]._pvoid), 10, _slist1, _dlist1, &t, dt, kstates_NMDA10_2<coreneuron::scopmath::enabled_code::compute_only>{}, _linmat1, _threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(Memb_list *_ml){
 double _x; double* _p = &_x;
 int _i;
 int _cntml_actual=1;
 int _cntml_padded=1;
 int _iml=0;
 _slist1[0] = &(OB) - _p;  _dlist1[0] = &(DOB) - _p;
 _slist1[1] = &(CB2) - _p;  _dlist1[1] = &(DCB2) - _p;
 _slist1[2] = &(CB1) - _p;  _dlist1[2] = &(DCB1) - _p;
 _slist1[3] = &(CB0) - _p;  _dlist1[3] = &(DCB0) - _p;
 _slist1[4] = &(C2) - _p;  _dlist1[4] = &(DC2) - _p;
 _slist1[5] = &(C1) - _p;  _dlist1[5] = &(DC1) - _p;
 _slist1[6] = &(C0) - _p;  _dlist1[6] = &(DC0) - _p;
 _slist1[7] = &(DB) - _p;  _dlist1[7] = &(DDB) - _p;
 _slist1[8] = &(D) - _p;  _dlist1[8] = &(DD) - _p;
 _slist1[9] = &(O) - _p;  _dlist1[9] = &(DO) - _p;
 }
} // namespace coreneuron_lib
