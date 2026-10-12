// Complete production bundle translation unit, including the actual measurement
// callback, real IKD/PCL/plane fit and iterated ESKF. Only main is renamed.
#define main fastlio_backend_main
#include "src/laserMapping_bundle.cpp"
#undef main
#include <cassert>
#include <iomanip>
#include FASTLIO_REFERENCE
int calls=0;
void checked_measurement(state_ikfom& s, esekfom::dyn_share_datastruct<double>& d) {
  h_share_model(s,d); ++calls;
  assert(laserCloudOri->size()==static_cast<size_t>(effect_feat_num));
  assert(corr_normvect->size()==laserCloudOri->size());
  if(d.valid) assert(d.h_x.rows()==effect_feat_num && d.h_x.cols()==12 && d.h.rows()==effect_feat_num);
}
void queries(int count, bool matched) {
  feats_down_body->clear();
  for(int i=0;i<count;++i) { PointType p{}; p.x=matched?1.5f+.01f*i:50.f+i; p.y=1.5f; p.z=1.f; p.intensity=i; feats_down_body->push_back(p); }
  feats_down_size=count; feats_down_world->resize(count); normvec->resize(count); Nearest_Points.resize(count);
}
int main(int argc,char**argv) {
  bool unmatched=argc>1 && std::string(argv[1])=="unmatched";
  PointVector map;
  for(int x=0;x<11;++x) for(int y=0;y<11;++y) { PointType p{}; p.x=1+.1f*x; p.y=1+.1f*y; p.z=1; map.push_back(p); }
  ikdtree.Build(map); extrinsic_est_en=false;
  auto capacity=laserCloudOri->points.capacity(); auto normal_capacity=corr_normvect->points.capacity();
  auto storage=laserCloudOri->points.data(); auto normal_storage=corr_normvect->points.data();
  if(unmatched) { queries(5,false); state_ikfom s; esekfom::dyn_share_datastruct<double> d; d.valid=d.converge=true; checked_measurement(s,d); assert(!d.valid && effect_feat_num==0); std::cout<<"RESULT {\"unmatched\":true,\"features\":0}"<<std::endl; return 0; }
  double max_jacobian_error=0;
  double reference_error=0;
  for(int count:{5,30,1,17,5}) {
    queries(count,true); state_ikfom s;
    esekfom::dyn_share_datastruct<double> reference; reference.valid=reference.converge=true;
    reference_h_share_model(s,reference);
    const auto reference_count=effect_feat_num;
    for(bool search:{true,false,true}) {
      esekfom::dyn_share_datastruct<double> d; d.valid=true; d.converge=search; checked_measurement(s,d);
      assert(d.valid && effect_feat_num==count);
      assert(effect_feat_num==reference_count);
      reference_error=std::max(reference_error,(d.h_x-reference.h_x).cwiseAbs().maxCoeff());
      reference_error=std::max(reference_error,(d.h-reference.h).cwiseAbs().maxCoeff());
      assert(reference_error==0.);
      // Analytic plane z=1 with the production fit's negative normal convention.
      for(int i=0;i<count;++i) {
        Eigen::Matrix<double,1,12> expected=Eigen::Matrix<double,1,12>::Zero();
        expected[2]=-1; expected[3]=-feats_down_body->points[i].y; expected[4]=feats_down_body->points[i].x;
        max_jacobian_error=std::max(max_jacobian_error,(d.h_x.row(i)-expected).cwiseAbs().maxCoeff());
        assert(std::abs(d.h[i])<1e-6); assert(laserCloudOri->points[i].intensity==i);
      }
      assert(max_jacobian_error<1e-6);
    }
    double e[23]; std::fill(e,e+23,1e-7);
    kf.init_dyn_share(get_f,df_dx,df_dw,checked_measurement,4,e);
    kf.change_x(s); auto P=kf.get_P(); P.setIdentity(); kf.change_P(P);
    int before=calls; double solve=0; kf.update_iterated_dyn_share_modified(.001,solve);
    assert(calls-before>=2 && kf.get_P().allFinite());
    assert(laserCloudOri->points.capacity()==capacity && corr_normvect->points.capacity()==normal_capacity);
    assert(laserCloudOri->points.data()==storage && corr_normvect->points.data()==normal_storage);
  }
  // Nonzero residual and both extrinsic Jacobian modes, including real posterior.
  double posterior_error=0;
  for(bool estimate:{false,true}) {
    extrinsic_est_en=estimate; queries(17,true); state_ikfom initial; initial.pos.z()=.02;
    initial.offset_T_L_I.x()=.03;
    esekfom::dyn_share_datastruct<double> r,d; r.valid=d.valid=r.converge=d.converge=true;
    reference_h_share_model(initial,r); checked_measurement(initial,d);
    assert(r.valid && d.valid && (r.h_x-d.h_x).cwiseAbs().maxCoeff()==0. && (r.h-d.h).cwiseAbs().maxCoeff()==0.);
    esekfom::esekf<state_ikfom,12,input_ikfom> original_math, fixed;
    double e[23]; std::fill(e,e+23,1e-7);
    original_math.init_dyn_share(get_f,df_dx,df_dw,reference_h_share_model,4,e);
    fixed.init_dyn_share(get_f,df_dx,df_dw,checked_measurement,4,e);
    original_math.change_x(initial); fixed.change_x(initial);
    double solve=0; original_math.update_iterated_dyn_share_modified(.001,solve);
    fixed.update_iterated_dyn_share_modified(.001,solve);
    auto x=fixed.get_x(); Eigen::Matrix<double,23,1> dx; x.boxminus(dx,original_math.get_x());
    posterior_error=std::max(posterior_error,dx.cwiseAbs().maxCoeff());
    posterior_error=std::max(posterior_error,(fixed.get_P()-original_math.get_P()).cwiseAbs().maxCoeff());
    assert(posterior_error==0.);
  }
  std::cout<<std::setprecision(17)<<"RESULT {\"measurement_calls\":"<<calls<<",\"jacobian_error\":"<<max_jacobian_error<<",\"reference_error\":"<<reference_error<<",\"posterior_error\":"<<posterior_error<<",\"capacity\":"<<capacity<<",\"allocation_reused\":true}"<<std::endl;
}
