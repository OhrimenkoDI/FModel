#include "adaptive_controller.hpp"
#include <cmath>
#include <cstdio>
int main(){
 adaptive_flight::AdaptiveController c;
 double yaw=0,prevdt=0;
 for(int i=0;i<600;++i){
  const double dt=0.003+(i%3)*0.001;
  const double gyro=(5+2*std::sin(i*0.03))*3.14159265358979323846/180;
  yaw+=gyro*prevdt;
  const double angle=0.2+0.1*std::sin(i*0.013);
  c.set_relative_target_coordinates(std::cos(yaw+angle),std::sin(yaw+angle));
  c.set_aircraft_sensors(yaw);
  const auto o=c.step(dt);
  std::printf("%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",o.rudder,o.raw_reference,o.filtered_reference,o.sliding_surface,o.identifier_residual,c.parameters()[0],c.parameters()[1],c.parameters()[2],c.parameters()[3]);
  prevdt=dt;
 }
}
