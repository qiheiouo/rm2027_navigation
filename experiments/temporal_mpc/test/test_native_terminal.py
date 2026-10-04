from pathlib import Path
import subprocess


def test_native_stopping_reserve_handles_new_held_velocity(tmp_path):
    root=Path(__file__).resolve().parents[1]
    source=tmp_path/'stop.cpp';binary=tmp_path/'stop'
    source.write_text(r'''
#include "stopping.hpp"
#include <cassert>
#include <cmath>
int main() {
  // Reproduces live failure: measured velocity has just increased under held
  // command while the old ramp proposal still terminates at the old velocity.
  for(int sign : {-1,1}) for(double jitter : {0.,.006,.03,.05,.149}) {
    double v=sign*.05;
    for(int k=0;k<30;k++) {
      double a=sign*(k<15?1.:-1.);
      // Extra initial measured velocity cannot simply retain old final controls.
      if(k==0) v+=sign*jitter;
      a=rm_temporal_mpc::stopping_acceleration(v,a,(29-k)*.05);
      assert(std::isfinite(a) && std::abs(a)<=1.);
      v+=.05*a;
      assert(std::abs(v)<=(29-k)*.05+1e-12);
    }
    assert(std::abs(v)<1e-12);
  }
  // Floating point residual at the last actuator limit cannot invert clamp bounds.
  const double a=rm_temporal_mpc::stopping_acceleration(.0500000000000001,0.,0.);
  assert(a==-1. && std::abs(.0500000000000001+.05*a)<1e-12);
}
''')
    subprocess.run(['g++','-std=c++17','-O2','-I',str(root/'ros2/rm_temporal_mpc_controller/include'),str(source),'-o',str(binary)],check=True,capture_output=True)
    subprocess.run([str(binary)],check=True,capture_output=True)


def test_native_velocity_projection_cannot_inherit_old_saturation(tmp_path):
    root=Path(__file__).resolve().parents[1]
    source=tmp_path/'bounds.cpp';binary=tmp_path/'bounds'
    source.write_text(r'''
#include "stopping.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>
int main() {
  using rm_temporal_mpc::bounded_stopping_acceleration;
  // The old saturation is no longer valid after a newer measured velocity.
  assert(bounded_stopping_acceleration(-.499, -1., 1., -.5,.5)>-.021);
  assert(bounded_stopping_acceleration(.799, 1., 1., -.5,.8)<.021);
  for(double upper : {.5,.8}) for(double initial : {-.5,-.499,-.05,0.,.049,.499,.5}) {
    for(int sign : {-1,1}) {
      double v=initial;
      for(int k=0;k<30;k++) {
        double a=bounded_stopping_acceleration(v,sign,(29-k)*.05,-.5,upper);
        assert(std::isfinite(a) && std::abs(a)<=1.);
        v+=.05*a;
        assert(v>=-.5-1e-12 && v<=upper+1e-12);
        assert(std::abs(v)<=(29-k)*.05+1e-12);
      }
      assert(std::abs(v)<1e-12);
    }
  }
}
''')
    subprocess.run(['g++','-std=c++17','-O2','-I',str(root/'ros2/rm_temporal_mpc_controller/include'),str(source),'-o',str(binary)],check=True,capture_output=True)
    subprocess.run([str(binary)],check=True,capture_output=True)
