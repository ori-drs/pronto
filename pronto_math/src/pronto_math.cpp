#include "pronto_math/pronto_math.hpp"
#include <iostream>


void scale_quaternion(double r,Eigen::Quaterniond q,Eigen::Quaterniond &q_out){
  if (q.w() ==1){
    // BUG: 15dec2011
    // TODO: check if this is a rigerous solution
    // q.w() ==1 mean no translation. so just return the input
    // if q.w()==1 and r=0.5 ... what is created below will have w=0 ... invalid
    //std::cout << "unit quaternion input:\n";
    q_out = q;
    return;
  }
  double theta = acos(q.w());
  //double sin_theta_inv = 1/ sin(theta);
  q_out.w() = sin((1-r)*theta)  + sin(r*theta) * q.w();
  q_out.x() = sin(r*theta) * q.x();
  q_out.y() = sin(r*theta) * q.y();
  q_out.z() = sin(r*theta) * q.z();
  q_out.normalize();
}


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {
  
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0 
  // Post DRC Trails: replace these with Eigen's own conversions
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }
  
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1301
void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


std::string print_Isometry3d(Eigen::Isometry3d pose){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  double rpy[3];
  quat_to_euler(r, rpy[0], rpy[1], rpy[2]);
  
  std::stringstream ss;
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<", " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() << ", "
       << rpy[0] <<", "<< rpy[1] <<", "<< rpy[2];
  return ss.str();
}


void print_Isometry3d(Eigen::Isometry3d pose, std::stringstream &ss){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  double rpy[3];
  quat_to_euler(r, rpy[0], rpy[1], rpy[2]);
  
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() << " | RPY "
       << rpy[0] <<", "<< rpy[1] <<", "<< rpy[2];
}

void print_Quaterniond(Eigen::Quaterniond r, std::stringstream &ss){
  ss <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << r.str() << "q\n";
}
