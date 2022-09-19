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
 
#define _thread_present_ /**/ , _thread[0:4] 
 
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
 
#define nrn_init _nrn_init__Isi
#define nrn_cur _nrn_cur__Isi
#define _nrn_current _nrn_current__Isi
#define nrn_jacob _nrn_jacob__Isi
#define nrn_state _nrn_state__Isi
#define initmodel initmodel__Isi
#define _net_receive _net_receive__Isi
#define _net_init _net_init__Isi
#define nrn_state_launcher nrn_state_Isi_launcher
#define nrn_cur_launcher nrn_cur_Isi_launcher
#define nrn_jacob_launcher nrn_jacob_Isi_launcher 
#define _f_rate _f_rate_Isi 
#define rate rate_Isi 
#define states states_Isi 
 
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
#define P _p[0*_STRIDE]
#define isi _p[1*_STRIDE]
#define ica _p[2*_STRIDE]
#define ik _p[3*_STRIDE]
#define ina _p[4*_STRIDE]
#define minf _p[5*_STRIDE]
#define hinf _p[6*_STRIDE]
#define mtau _p[7*_STRIDE]
#define htau _p[8*_STRIDE]
#define m _p[9*_STRIDE]
#define n _p[10*_STRIDE]
#define h _p[11*_STRIDE]
#define Dm _p[12*_STRIDE]
#define Dn _p[13*_STRIDE]
#define Dh _p[14*_STRIDE]
#define eca _p[15*_STRIDE]
#define ninf _p[16*_STRIDE]
#define ntau _p[17*_STRIDE]
#define cai _p[18*_STRIDE]
#define cao _p[19*_STRIDE]
#define nai _p[20*_STRIDE]
#define nao _p[21*_STRIDE]
#define ki _p[22*_STRIDE]
#define ko _p[23*_STRIDE]
#define beta_f2 _p[24*_STRIDE]
#define _v_unused _p[25*_STRIDE]
#define _g_unused _p[26*_STRIDE]
 
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
#define _ion_cao		_nt_data[_ppvar[0*_STRIDE]]
#define _ion_cai		_nt_data[_ppvar[1*_STRIDE]]
#define _ion_ica	_nt_data[_ppvar[2*_STRIDE]]
#define _ion_dicadv	_nt_data[_ppvar[3*_STRIDE]]
#define _ion_ko		_nt_data[_ppvar[4*_STRIDE]]
#define _ion_ki		_nt_data[_ppvar[5*_STRIDE]]
#define _ion_ik	_nt_data[_ppvar[6*_STRIDE]]
#define _ion_dikdv	_nt_data[_ppvar[7*_STRIDE]]
#define _ion_nao		_nt_data[_ppvar[8*_STRIDE]]
#define _ion_nai		_nt_data[_ppvar[9*_STRIDE]]
#define _ion_ina	_nt_data[_ppvar[10*_STRIDE]]
#define _ion_dinadv	_nt_data[_ppvar[11*_STRIDE]]
 
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
 static void _hoc_alp(void);
 static void _hoc_bet(void);
 static void _hoc_rate(void);
 
#endif /*BBCORE*/
 static int _mechtype;
 
#if 0 /*BBCORE*/
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_Isi", _hoc_setdata,
 "alp_Isi", _hoc_alp,
 "bet_Isi", _hoc_bet,
 "rate_Isi", _hoc_rate,
 0, 0
};
 
#endif /*BBCORE*/
#define alp alp_Isi
#define bet bet_Isi
 #pragma acc routine seq
 inline double alp( _threadargsprotocomma_ double , double );
 #pragma acc routine seq
 inline double bet( _threadargsprotocomma_ double , double );
 
static void _check_rate(_threadargsproto_); 
static void _check_table_thread(int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, NrnThread* _nt, Memb_list* _ml, int v) {
   _check_rate(_threadargs_);
 }
 #define _zRT _thread[3]._pval[0]
 /* declare global and static user variables */
 static double Kmf2_Isi = 0.001;
 static double S_Isi = 0.063;
 static double Ve_Isi = 0.00157;
 static double Vrel_Isi = 0.0002827;
 static double Vup_Isi = 0.0007069;
 static double Vcell_Isi = 0.0157;
 static double Vi_Isi = 0.014137;
 static double alpha_f2_Isi = 5;
 static double usetable_Isi = 1;
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_Isi", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Vcell_Isi", "mm3",
 "Vi_Isi", "mm3",
 "Ve_Isi", "mm3",
 "Vup_Isi", "mm3",
 "Vrel_Isi", "mm3",
 "S_Isi", "cm2",
 "alpha_f2_Isi", "/s",
 "Kmf2_Isi", "mM",
 "P_Isi", "nA/mM",
 "isi_Isi", "mA/cm2",
 "ica_Isi", "mA/cm2",
 "ik_Isi", "mA/cm2",
 "ina_Isi", "mA/cm2",
 "mtau_Isi", "ms",
 "htau_Isi", "ms",
 0,0
};
 
#endif /*BBCORE*/
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
 static double n0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "Vcell_Isi", &Vcell_Isi,
 "Vi_Isi", &Vi_Isi,
 "Ve_Isi", &Ve_Isi,
 "Vup_Isi", &Vup_Isi,
 "Vrel_Isi", &Vrel_Isi,
 "S_Isi", &S_Isi,
 "alpha_f2_Isi", &alpha_f2_Isi,
 "Kmf2_Isi", &Kmf2_Isi,
 "usetable_Isi", &usetable_Isi,
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
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"Isi",
 "P_Isi",
 0,
 "isi_Isi",
 "ica_Isi",
 "ik_Isi",
 "ina_Isi",
 "minf_Isi",
 "hinf_Isi",
 "mtau_Isi",
 "htau_Isi",
 0,
 "m_Isi",
 "n_Isi",
 "h_Isi",
 0,
 0};
 static int _ca_type;
 static int _k_type;
 static int _na_type;
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	P = 15;
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[2]; /* cao */
 	_ppvar[1]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[4]._pval = &prop_ion->param[2]; /* ko */
 	_ppvar[5]._pval = &prop_ion->param[1]; /* ki */
 	_ppvar[6]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[7]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[8]._pval = &prop_ion->param[2]; /* nao */
 	_ppvar[9]._pval = &prop_ion->param[1]; /* nai */
 	_ppvar[10]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[11]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 
#endif /* BBCORE */
 
}
 static void _initlists(Memb_list *_ml);
 static void _thread_mem_init(ThreadDatum*);
 static void _thread_cleanup(ThreadDatum*);
 static void _update_ion_pointer(Datum*);
 
#define _psize 27
#define _ppsize 12
 void _isi_reg() {
	int _vectorized = 1;
  _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 _ca_type = nrn_get_mechtype("ca_ion"); _k_type = nrn_get_mechtype("k_ion"); _na_type = nrn_get_mechtype("na_ion"); 
#if 0 /*BBCORE*/
 	ion_reg("ca", -10000.);
 	ion_reg("k", -10000.);
 	ion_reg("na", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_k_sym = hoc_lookup("k_ion");
 	_na_sym = hoc_lookup("na_ion");
 
#endif /*BBCORE*/
  register_mech(_mechanism, nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init, _create_global_variables, _destroy_global_variables, hoc_nrnpointerindex, 5);
  _extcall_thread = (ThreadDatum*)ecalloc(4, sizeof(ThreadDatum));
  _thread_mem_init(_extcall_thread);
     _nrn_thread_reg1(_mechtype, _thread_mem_init);
     _nrn_thread_reg0(_mechtype, _thread_cleanup);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 8, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 9, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 10, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 11, "na_ion");
 	hoc_register_var(hoc_scdoub, hoc_vdoub, NULL);
 }
 
static double F = /* 0x1.78e555060882cp+16; */ 96485.3321233100141;
 
static double R = /* 0x1.0a1013e8990bep+3; */ 8.3144626181532395;
 /*Top LOCAL _zRT */
 static double *_t_minf = nullptr;
 static double *_t_mtau = nullptr;
 static double *_t_ninf = nullptr;
 static double *_t_ntau = nullptr;
 struct _global_variables_t : public MemoryManaged {
   double F;
   double R;
   int _slist1[3];
   int _dlist1[3];
   int _slist2[3];
   double celsius;
   double Kmf2;
   double S;
   double Ve;
   double Vrel;
   double Vup;
   double Vcell;
   double Vi;
   double alpha_f2;
   double usetable;
   double delta_t;
   double h0;
   double m0;
   double n0;
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
   _global_variables->F = F;
   _global_variables->R = R;
   _global_variables->celsius = celsius;
   _global_variables->Kmf2 = Kmf2_Isi;
   _global_variables->S = S_Isi;
   _global_variables->Ve = Ve_Isi;
   _global_variables->Vrel = Vrel_Isi;
   _global_variables->Vup = Vup_Isi;
   _global_variables->Vcell = Vcell_Isi;
   _global_variables->Vi = Vi_Isi;
   _global_variables->alpha_f2 = alpha_f2_Isi;
   _global_variables->usetable = usetable_Isi;
   _global_variables->delta_t = delta_t;
   _global_variables->h0 = h0;
   _global_variables->m0 = m0;
   _global_variables->n0 = n0;
 #ifdef CORENEURON_ENABLE_GPU
   if (_nt->compute_gpu) {
       cnrn_target_update_on_device(_global_variables);
   }
 #endif
 }

 #define F static_cast<_global_variables_t*>(_ml->global_variables)->F
 #define R static_cast<_global_variables_t*>(_ml->global_variables)->R
 #define _slist1 static_cast<_global_variables_t*>(_ml->global_variables)->_slist1
 #define _dlist1 static_cast<_global_variables_t*>(_ml->global_variables)->_dlist1
 #define _slist2 static_cast<_global_variables_t*>(_ml->global_variables)->_slist2
 #define celsius static_cast<_global_variables_t*>(_ml->global_variables)->celsius
 #define Kmf2 static_cast<_global_variables_t*>(_ml->global_variables)->Kmf2
 #define S static_cast<_global_variables_t*>(_ml->global_variables)->S
 #define Ve static_cast<_global_variables_t*>(_ml->global_variables)->Ve
 #define Vrel static_cast<_global_variables_t*>(_ml->global_variables)->Vrel
 #define Vup static_cast<_global_variables_t*>(_ml->global_variables)->Vup
 #define Vcell static_cast<_global_variables_t*>(_ml->global_variables)->Vcell
 #define Vi static_cast<_global_variables_t*>(_ml->global_variables)->Vi
 #define alpha_f2 static_cast<_global_variables_t*>(_ml->global_variables)->alpha_f2
 #define usetable static_cast<_global_variables_t*>(_ml->global_variables)->usetable
 #define delta_t static_cast<_global_variables_t*>(_ml->global_variables)->delta_t
 #define h0 static_cast<_global_variables_t*>(_ml->global_variables)->h0
 #define m0 static_cast<_global_variables_t*>(_ml->global_variables)->m0
 #define n0 static_cast<_global_variables_t*>(_ml->global_variables)->n0
 
static const char *modelname = "Cardiac second inward current";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static inline int _f_rate(_threadargsprotocomma_ double);
static inline int rate(_threadargsprotocomma_ double);
 
#define _deriv1_advance _thread[0]._i
#define _dith1 1
#define _newtonspace1 _thread[2]._pvoid
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static void _n_rate(_threadargsprotocomma_ double _lv);
 
/* _derivimplicit_ states _Isi */
#ifndef INSIDE_NMODL
#define INSIDE_NMODL
#endif
 
struct _newton_states_Isi {
  int operator()(_threadargsproto_) const;
};
  
struct states_Isi {
  int operator()(_threadargsproto_) const;
};
 
/*CVODE*/
 static int _ode_spec1 (_threadargsproto_) {int _reset = 0; {
   rate ( _threadargscomma_ v ) ;
   Dm = ( minf - m ) / mtau ;
   Dn = ( ninf - n ) / ntau ;
   Dh = ( 0.001 ) * ( alpha_f2 - h * ( alpha_f2 + beta_f2 ) ) ;
   beta_f2 = cai * alpha_f2 / Kmf2 ;
   }
 return _reset;
}
 static int _ode_matsol1 (_threadargsproto_) {
 rate ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / mtau )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / ntau )) ;
 Dh = Dh  / (1. - dt*( (( 0.001 ))*(( ( - (1.0)*(( alpha_f2 + beta_f2 )) ) )) )) ;
 return 0;
}
 /*END CVODE*/
 
int states ::operator()(_threadargsproto_) const {
 int _reset=0;
 int error = 0;
 { double* _savstate1 = (double*)_thread[_dith1]._pval;
 double* _dlist2 = (double*)(_thread[_dith1]._pval) + (3*_cntml_padded);
 {int _id; for(_id=0; _id < 3; _id++) { _savstate1[_id*_STRIDE] = _p[_slist1[_id]*_STRIDE];}}
 _reset = nrn_newton_thread(static_cast<NewtonSpace*>(_newtonspace1), 3, _slist2, _newton_states_Isi{}, _dlist2, _threadargs_);
 /*if(_reset) {abort_run(_reset);}*/ }
 
  return _reset;
}

int _newton_states_Isi::operator()(_threadargsproto_) const {
  int _reset=0;
 { double* _savstate1 = (double*)_thread[_dith1]._pval;
 double* _dlist2 = (double*)(_thread[_dith1]._pval) + (3*_cntml_padded);
 int _counte = -1;
 {
   rate ( _threadargscomma_ v ) ;
   Dm = ( minf - m ) / mtau ;
   Dn = ( ninf - n ) / ntau ;
   Dh = ( 0.001 ) * ( alpha_f2 - h * ( alpha_f2 + beta_f2 ) ) ;
   beta_f2 = cai * alpha_f2 / Kmf2 ;
   {int _id; for(_id=0; _id < 3; _id++) {
if (_deriv1_advance) {
 _dlist2[(++_counte)*_STRIDE] = _p[_dlist1[_id]*_STRIDE] - (_p[_slist1[_id]*_STRIDE] - _savstate1[_id*_STRIDE])/dt;
 }else{
_dlist2[(++_counte)*_STRIDE] = _p[_slist1[_id]*_STRIDE] - _savstate1[_id*_STRIDE];}}}
 
  } }
 return _reset;}
 
double alp ( _threadargsprotocomma_ double _lv , double _li ) {
   double _lalp;
 double _lEo , _lE1 ;
 if ( _li  == 0.0 ) {
     _lEo = _lv + 24.0 ;
     if ( fabs ( _lEo * 1.0 ) < 1e-5 ) {
       _lalp = ( 0.001 ) * 120.0 ;
       }
     else {
       _lalp = ( 0.001 ) * 30.0 * _lEo / ( 1.0 - exp ( - _lEo / 4.0 ) ) ;
       }
     }
   else if ( _li  == 1.0 ) {
     _lE1 = _lv + 34.0 ;
     if ( fabs ( _lE1 * 1.0 ) < 1e-5 ) {
       _lalp = ( 0.001 ) * 25.0 ;
       }
     else {
       _lalp = ( 0.001 ) * 6.25 * _lE1 / ( exp ( _lE1 / 4.0 ) - 1.0 ) ;
       }
     }
   
return _lalp;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_alp(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  alp ( _threadargs_, *getarg(1) , *getarg(2) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
double bet ( _threadargsprotocomma_ double _lv , double _li ) {
   double _lbet;
 double _lEo ;
 if ( _li  == 0.0 ) {
     _lEo = _lv + 24.0 ;
     if ( fabs ( _lEo * 1.0 ) < 1e-5 ) {
       _lbet = ( 0.001 ) * 120.0 ;
       }
     else {
       _lbet = ( 0.001 ) * 12.0 * _lEo / ( exp ( _lEo / 10.0 ) - 1.0 ) ;
       }
     }
   else if ( _li  == 1.0 ) {
     _lbet = ( 0.001 ) * 50.0 / ( 1.0 + exp ( - ( _lv + 34.0 ) / 4.0 ) ) ;
     }
   
return _lbet;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_bet(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  bet ( _threadargs_, *getarg(1) , *getarg(2) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 static double _mfac_rate, _tmin_rate;
  static void _check_rate(_threadargsproto_) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  if (!usetable) {return;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_rate =  - 100.0 ;
   _tmax =  100.0 ;
   _dx = (_tmax - _tmin_rate)/200.; _mfac_rate = 1./_dx;
   for (_i=0, _x=_tmin_rate; _i < 201; _x += _dx, _i++) {
    _f_rate(_threadargs_, _x);
    _t_minf[_i] = minf;
    _t_mtau[_i] = mtau;
    _t_ninf[_i] = ninf;
    _t_ntau[_i] = ntau;
   }
  }
 }

 static int rate(_threadargsproto_, double _lv) { 
#if 0
_check_rate(_threadargs_);
#endif
 _n_rate(_threadargs_, _lv);
 return 0;
 }

 static void _n_rate(_threadargsproto_, double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_rate(_threadargs_, _lv); return; 
}
 _xi = _mfac_rate * (_lv - _tmin_rate);
 if (isnan(_xi)) {
  minf = _xi;
  mtau = _xi;
  ninf = _xi;
  ntau = _xi;
  return;
 }
 if (_xi <= 0.) {
 minf = _t_minf[0];
 mtau = _t_mtau[0];
 ninf = _t_ninf[0];
 ntau = _t_ntau[0];
 return; }
 if (_xi >= 200.) {
 minf = _t_minf[200];
 mtau = _t_mtau[200];
 ninf = _t_ninf[200];
 ntau = _t_ntau[200];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 minf = _t_minf[_i] + _theta*(_t_minf[_i+1] - _t_minf[_i]);
 mtau = _t_mtau[_i] + _theta*(_t_mtau[_i+1] - _t_mtau[_i]);
 ninf = _t_ninf[_i] + _theta*(_t_ninf[_i+1] - _t_ninf[_i]);
 ntau = _t_ntau[_i] + _theta*(_t_ntau[_i+1] - _t_ntau[_i]);
 }

 
static int  _f_rate ( _threadargsprotocomma_ double _lv ) {
   double _la , _lb ;
 _la = alp ( _threadargscomma_ _lv , 0.0 ) ;
   _lb = bet ( _threadargscomma_ _lv , 0.0 ) ;
   mtau = 1.0 / ( _la + _lb ) ;
   minf = _la * mtau ;
   _la = alp ( _threadargscomma_ _lv , 1.0 ) ;
   _lb = bet ( _threadargscomma_ _lv , 1.0 ) ;
   ntau = 1.0 / ( _la + _lb ) ;
   ninf = _la * ntau ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static void _hoc_rate(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 
#if 1
 _check_rate(_threadargs_);
#endif
 _r = 1.;
 rate ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
static void _thread_mem_init(ThreadDatum* _thread) {
   _thread[_dith1]._pval = NULL;   _thread[3]._pval = (double*)ecalloc(1, sizeof(double));
 }
 
static void _thread_cleanup(ThreadDatum* _thread) {
   free( _thread[_dith1]._pval);
   nrn_destroy_newtonspace((NewtonSpace*) _newtonspace1);
   free((void*)(_thread[3]._pval));
 }
 static void _update_ion_pointer(Datum* _ppvar) {
 }

static inline void initmodel(_threadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
  n = n0;
 {
   _zRT = ( 1000.0 ) * R * ( 273.15 + celsius ) ;
   rate ( _threadargscomma_ v ) ;
   m = minf ;
   n = ninf ;
   h = hinf ;
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
  _deriv1_advance = 0;
  #ifdef _OPENACC
  #pragma acc update device (_deriv1_advance) if (_nt->compute_gpu)
  #endif
  if (!_newtonspace1) {
    _newtonspace1 = nrn_cons_newtonspace(3, _cntml_padded);
    _thread[_dith1]._pval = makevector(2*3*_cntml_padded*sizeof(double));
    #ifdef _OPENACC
    if (_nt->compute_gpu) {
      void* _d_ns = cnrn_target_deviceptr(_newtonspace1);
      double* _d_pd = cnrn_target_copyin(_thread[_dith1]._pval, 2*3*_cntml_padded);
      ThreadDatum* _d_td = cnrn_target_deviceptr(_thread);
      cnrn_target_memcpy_to_device(&(_d_td[2]._pvoid), &_d_ns);
      cnrn_target_memcpy_to_device(&(_d_td[_dith1]._pval), &_d_pd);
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

#if 0
 _check_rate(_threadargs_);
#endif
    _v = _vec_v[_nd_idx];
    _PRCELLSTATE_V
 v = _v;
 _PRCELLSTATE_V
  cao = _ion_cao;
  cai = _ion_cai;
  ko = _ion_ko;
  ki = _ion_ki;
  nao = _ion_nao;
  nai = _ion_nai;
 initmodel(_threadargs_);
   }
  }
  _deriv1_advance = 1;
  #ifdef _OPENACC
  #pragma acc update device (_deriv1_advance) if (_nt->compute_gpu)
  #endif
}

static double _nrn_current(_threadargsproto_, double _v){double _current=0.;v=_v;{ {
   double _lcom , _lcom2 , _lEo , _licat , _linat , _likt ;
 _lEo = v - 50.0 ;
   _lcom = ( 1e-06 ) * P / S * _lEo * F / _zRT * m * n * h ;
   _licat = 4.0 * _lcom / ( 1.0 - exp ( - 2.0 * _lEo * F / _zRT ) ) * ( cai * exp ( 100.0 * F / _zRT ) - cao * exp ( - 2.0 * F * _lEo / _zRT ) ) ;
   ica = _licat ;
   _lcom2 = 0.01 * _lcom / ( 1.0 - exp ( - _lEo * F / _zRT ) ) ;
   _likt = _lcom2 * ( ki * exp ( 50.0 * F / _zRT ) - ko * exp ( - F * _lEo / _zRT ) ) ;
   ik = _likt ;
   _linat = _lcom2 * ( nai * exp ( 50.0 * F / _zRT ) - nao * exp ( - F * _lEo / _zRT ) ) ;
   ina = _linat ;
   isi = _linat + _likt + _licat ;
   }
 _current += ica;
 _current += ik;
 _current += ina;

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
_PRAGMA_FOR_CUR_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
    _PRCELLSTATE_V
  cao = _ion_cao;
  cai = _ion_cai;
  ko = _ion_ko;
  ki = _ion_ki;
  nao = _ion_nao;
  nai = _ion_nai;
 _g = _nrn_current(_threadargs_, _v + .001);
 	{ double _dina;
 double _dik;
 double _dica;
  _dica = ica;
  _dik = ik;
  _dina = ina;
 _rhs = _nrn_current(_threadargs_, _v);
  _ion_dicadv += (_dica - ica)/.001 ;
  _ion_dikdv += (_dik - ik)/.001 ;
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
  _ion_ik += ik ;
  _ion_ina += ina ;
 _PRCELLSTATE_G
	_vec_rhs[_nd_idx] -= _rhs;
	_vec_d[_nd_idx] += _g;
 
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
  cao = _ion_cao;
  cai = _ion_cai;
  ko = _ion_ko;
  ki = _ion_ki;
  nao = _ion_nao;
  nai = _ion_nai;
 {  
  derivimplicit_thread(3, _slist1, _dlist1, states_Isi{}, _threadargs_);
  }   }}

}

static void terminal(){}

static void _initlists(Memb_list *_ml){
 double _x; double* _p = &_x;
 int _i;
 int _cntml_actual=1;
 int _cntml_padded=1;
 int _iml=0;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(n) - _p;  _dlist1[1] = &(Dn) - _p;
 _slist1[2] = &(h) - _p;  _dlist1[2] = &(Dh) - _p;
 _slist2[0] = &(h) - _p;
 _slist2[1] = &(m) - _p;
 _slist2[2] = &(n) - _p;
   if (!_t_minf) { _t_minf = makevector(201*sizeof(double)); }
   if (!_t_mtau) { _t_mtau = makevector(201*sizeof(double)); }
   if (!_t_ninf) { _t_ninf = makevector(201*sizeof(double)); }
   if (!_t_ntau) { _t_ntau = makevector(201*sizeof(double)); }
 }
} // namespace coreneuron_lib
