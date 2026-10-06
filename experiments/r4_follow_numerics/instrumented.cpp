// Research instrumentation of the SAME Follow implementation; never installed.
#include "trace.hpp"
#include <osqp.h>
#include <auxil.h>
#include <polish.h>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
namespace r4_trace {
std::string mode,condition; int cycle{};
struct Record {
  std::string mode,condition,status; int cycle{},iterations{},status_value{},updates{},interval{};
  double primal{},dual{},rho{},estimate{},setup_seconds{},solve_seconds{}; int solver_rows{};
  int pre_status{},polish_status{}; double pre_primal{},pre_dual{};
  std::vector<double> P,A,q,lower,upper,warm,x,y;
};
std::vector<Record> records;
const OSQPData * current{};
std::vector<int> selected;
std::vector<c_int> reduced_columns,reduced_rows;
std::vector<c_float> reduced_values,reduced_lower,reduced_upper;
csc reduced_A{};OSQPData reduced_data{};
std::vector<double> dense(csc * m, bool symmetric=false) {
  std::vector<double> out(m->m*m->n);
  for(int col=0;col<m->n;++col) {for(int k=m->p[col];k<m->p[col+1];++k) {
    out[m->i[k]*m->n+col]=m->x[k];
    if(symmetric) {out[col*m->n+m->i[k]]=m->x[k];}
  }}return out;
}
void array(std::ofstream & out,const std::string & name,const std::vector<double> & a) {
  out<<'"'<<name<<"\":[";for(size_t i=0;i<a.size();++i) {out<<(i?",":"")<<a[i];}out<<']';
}
void save(const std::filesystem::path & output) {
  std::filesystem::create_directories(output);std::ofstream out(output/"trace.csv");out<<std::setprecision(17);
  out<<"mode,condition,cycle,status,status_value,iterations,primal_residual,dual_residual,rho,rho_estimate,rho_updates,adaptive_interval,setup_ms,solve_ms,solver_rows,pre_status,pre_primal,pre_dual,polish_status\n";
  for(const auto & r:records) {
    out<<r.mode<<','<<r.condition<<','<<r.cycle<<','<<r.status<<','<<r.status_value<<','<<r.iterations<<','<<r.primal<<','<<r.dual<<','
      <<r.rho<<','<<r.estimate<<','<<r.updates<<','<<r.interval<<','<<1000*r.setup_seconds<<','<<1000*r.solve_seconds<<','<<r.solver_rows<<','
      <<r.pre_status<<','<<r.pre_primal<<','<<r.pre_dual<<','<<r.polish_status<<'\n';
    if(r.P.empty()) {continue;}
    std::ofstream qp(output/(r.mode+"_"+r.condition+"_"+std::to_string(r.cycle)+".json"));qp<<std::setprecision(17)<<'{';
    array(qp,"P",r.P);qp<<',';array(qp,"A",r.A);qp<<',';array(qp,"q",r.q);qp<<',';
    array(qp,"lower",r.lower);qp<<',';array(qp,"upper",r.upper);qp<<',';array(qp,"warm",r.warm);qp<<',';
    array(qp,"x",r.x);qp<<',';array(qp,"y",r.y);qp<<"}\n";
  }
}
}
c_int traced_setup(OSQPWorkspace ** work,const OSQPData * data,const OSQPSettings * settings) {
  r4_trace::current=data;r4_trace::Record record;
  record.mode=r4_trace::mode;record.condition=r4_trace::condition;record.cycle=r4_trace::cycle;
  r4_trace::records.push_back(record);auto configured=*settings;
  if(r4_trace::mode=="interval10") {configured.adaptive_rho_interval=10;}
  if(r4_trace::mode=="rho_ratio15" || (r4_trace::mode=="rho_at42" && r4_trace::cycle==42 && r4_trace::condition=="clear")) {configured.adaptive_rho_tolerance=1.5;}
  r4_trace::selected.clear();const OSQPData * input=data;
  if(r4_trace::mode=="equivalent_rows" || r4_trace::mode=="reduced_polish" || r4_trace::mode=="reduced_refine") {
    auto & selected=r4_trace::selected;
    for(int i=0;i<75;++i) {selected.push_back(i);}
    for(int base : {75,106}) {for(int k=0;k<=30;k+=2) {selected.push_back(base+k);}}
    selected.push_back(167); // progress is monotone: only its terminal bounds are independent.
    std::vector<int> inverse(data->m,-1);
    for(size_t i=0;i<selected.size();++i) {inverse[selected[i]]=i;}
    auto & cols=r4_trace::reduced_columns;auto & rows=r4_trace::reduced_rows;auto & vals=r4_trace::reduced_values;
    auto & lo=r4_trace::reduced_lower;auto & hi=r4_trace::reduced_upper;
    cols.clear();rows.clear();vals.clear();lo.clear();hi.clear();
    for(int col=0;col<data->n;++col) {cols.push_back(vals.size());
      for(int k=data->A->p[col];k<data->A->p[col+1];++k) {const int row=inverse[data->A->i[k]];
        if(row>=0) {rows.push_back(row);vals.push_back(data->A->x[k]);}}
    }cols.push_back(vals.size());
    for(int row:selected) {lo.push_back(data->l[row]);hi.push_back(data->u[row]);}
    r4_trace::reduced_A={static_cast<c_int>(vals.size()),static_cast<c_int>(selected.size()),data->n,cols.data(),rows.data(),vals.data(),-1};
    r4_trace::reduced_data=*data;r4_trace::reduced_data.m=selected.size();r4_trace::reduced_data.A=&r4_trace::reduced_A;
    r4_trace::reduced_data.l=lo.data();r4_trace::reduced_data.u=hi.data();input=&r4_trace::reduced_data;
  }
  r4_trace::records.back().solver_rows=input->m;
  return osqp_setup(work,input,&configured);
}
c_int traced_warm(OSQPWorkspace * work,const c_float * x) {
  auto & r=r4_trace::records.back();r.warm.assign(x,x+work->data->n);
  if(r4_trace::mode=="zero_primal" && r4_trace::cycle==42 && r4_trace::condition=="clear") {
    std::fill(r.warm.begin(),r.warm.end(),0.);
  }
  return osqp_warm_start_x(work,r.warm.data());
}
c_int traced_solve(OSQPWorkspace * work) {
  const auto code=osqp_solve(work);auto & r=r4_trace::records.back();
  r.pre_status=work->info->status_val;r.pre_primal=work->info->pri_res;r.pre_dual=work->info->dua_res;
  const bool refine=(r4_trace::mode=="strict_polish" || r4_trace::mode=="reduced_polish" || r4_trace::mode=="reduced_refine");
  const bool eligible=work->info->status_val==OSQP_SOLVED_INACCURATE ||
    (r4_trace::mode=="reduced_refine" && work->info->status_val==OSQP_MAX_ITER_REACHED);
  if(refine && code==0 && eligible && work->info->iter==400) {
    // Diagnostic only: reuse OSQP's existing active-set refinement, then ask
    // its original strict termination check, never relabel approximate output.
    if(polish(work)==0 && work->info->status_polish==1) {
      update_info(work,work->info->iter,1,0);
      if(check_termination(work,0) && work->info->status_val==OSQP_SOLVED) {store_solution(work);}
    }
  }
  r.polish_status=work->info->status_polish;const auto & info=*work->info;
  r.status=info.status;r.status_value=info.status_val;r.iterations=info.iter;r.primal=info.pri_res;r.dual=info.dua_res;
  r.rho=work->settings->rho;r.estimate=info.rho_estimate;r.updates=info.rho_updates;r.interval=work->settings->adaptive_rho_interval;
  r.setup_seconds=info.setup_time;r.solve_seconds=info.solve_time;
  if(info.status_val!=OSQP_SOLVED || (r.condition=="clear"&&r.cycle==42)) {
    const auto & d=*r4_trace::current;r.P=r4_trace::dense(d.P,true);r.A=r4_trace::dense(d.A);r.q.assign(d.q,d.q+d.n);
    r.lower.assign(d.l,d.l+d.m);r.upper.assign(d.u,d.u+d.m);
    if(work->solution&&work->solution->x) {r.x.assign(work->solution->x,work->solution->x+d.n);r.y.assign(d.m,0.);
      if(r4_trace::selected.empty()) {std::copy(work->solution->y,work->solution->y+d.m,r.y.begin());}
      else {for(size_t i=0;i<r4_trace::selected.size();++i) {r.y[r4_trace::selected[i]]=work->solution->y[i];}}}
  }
  return code;
}
#define osqp_setup traced_setup
#define osqp_warm_start_x traced_warm
#define osqp_solve traced_solve
#ifndef R4_NUMERICS_BASELINE_SOURCE
#error "Set R4_NUMERICS_BASELINE_SOURCE to the generated A21 Git source"
#endif
#include R4_NUMERICS_BASELINE_SOURCE
