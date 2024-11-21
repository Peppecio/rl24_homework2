#include "kdl_planner.h"
#include <cmath>


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
  
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit,Eigen::Vector3d _trajEnd,double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    accDuration_ = _accDuration;
    trajEnd_=_trajEnd;

}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}


void KDLPlanner::trapezoidal_vel (double time, double accDuration_, double & s, double & s_dot, double & s_ddot) {

  double ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

  if ( time <=accDuration_) {
    s = 0.5 * ddot_traj_c * std::pow(time,2);
    s_dot = ddot_traj_c * time; 
    s_ddot = ddot_traj_c; 

  } 
  else if (time < trajDuration_ - accDuration_) {
    s= ddot_traj_c * accDuration_ * (time - accDuration_/2);
    s_dot= ddot_traj_c * accDuration_; 
    s_ddot= 0.0;

  } 
  else {
    s=  1 - 0.5 * ddot_traj_c * (std::pow(trajDuration_ - time,2));
    s_dot =  ddot_traj_c * (trajDuration_ - time);
    s_ddot  = - ddot_traj_c;
  }

}

void KDLPlanner::cubic_polinomial (double time, double &s, double &s_dot, double &s_ddot) {

  double s_fin = 1;
  double t_f = trajDuration_; 

  double a_0 = 0.0;
  double a_1 = 0.0;
  double a_2 = (3 * s_fin  / std::pow(t_f, 2)) ;
  double a_3 = (-2 * s_fin / std::pow(t_f, 3)) ;

  s = a_3 * std::pow(time, 3) + a_2 * std::pow(time, 2) + a_1 * time + a_0;
  s_dot = 3 * a_3 * std::pow(time, 2) + 2 * a_2 * time + a_1;
  s_ddot = 6 * a_3 * time + 2 * a_2;

}

trajectory_point KDLPlanner::compute_linear_trajectory(double time){

  double s, s_dot, s_ddot;
  trajectory_point traj;
  cubic_polinomial(time, s, s_dot, s_ddot);

  traj.pos[0] = trajInit_[0] + s*(trajEnd_[0] - trajInit_[0]);
  traj.pos[1] = trajInit_[1] + s*(trajEnd_[1] - trajInit_[1]);
  traj.pos[2] = trajInit_[2] + s*(trajEnd_[2] - trajInit_[2]);

  traj.vel[0] = s_dot*(trajEnd_[0] - trajInit_[0]);
  traj.vel[1] = s_dot*(trajEnd_[1] - trajInit_[1]);
  traj.vel[2] = s_dot*(trajEnd_[2] - trajInit_[2]);

  traj.acc[0] = s_ddot*(trajEnd_[0] - trajInit_[0]);
  traj.acc[1] = s_ddot*(trajEnd_[1] - trajInit_[1]);
  traj.acc[2] = s_ddot*(trajEnd_[2] - trajInit_[2]);

  return traj;

}


trajectory_point KDLPlanner::compute_circular_trajectory(double time)
{
    double s, s_dot, s_ddot;
    trajectory_point traj;
    cubic_polinomial(time, s, s_dot, s_ddot);
    std::cout<< "ascissa:"<<s<<std::endl;

    traj.pos[0] = trajInit_[0];
    traj.pos[1] = trajInit_[1] + trajRadius_ - trajRadius_ * cos(2 * M_PI * s);
    traj.pos[2] = trajInit_[2]- trajRadius_ * sin(2 * M_PI * s);

    traj.vel[0] = 0;
    traj.vel[1] = trajRadius_ * sin (2 * M_PI * s) * 2 * M_PI * s_dot;
    traj.vel[2] = - trajRadius_ * cos(2 * M_PI * s) * 2 * M_PI * s_dot;

    traj.acc[0] = 0;
    traj.acc[1] = 2 * M_PI * trajRadius_ * (s_ddot * sin (2 * M_PI * s) + 2 * M_PI * std::pow(s_dot, 2) * cos(2*M_PI*s));
    traj.acc[2] = - 2 * M_PI * trajRadius_ * (s_ddot * cos (2 * M_PI * s) - 2 * M_PI * std::pow(s_dot, 2) * sin(2*M_PI*s));
 

  return traj;

}


trajectory_point KDLPlanner::compute_linear_trajectory_trapezoidal(double time){

  double s, s_dot, s_ddot;
  trajectory_point traj;
  trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);

  traj.pos[0] = trajInit_[0] + s*(trajEnd_[0] - trajInit_[0]);
  traj.pos[1] = trajInit_[1] + s*(trajEnd_[1] - trajInit_[1]);
  traj.pos[2] = trajInit_[2] + s*(trajEnd_[2] - trajInit_[2]);

  traj.vel[0] = s_dot*(trajEnd_[0] - trajInit_[0]);
  traj.vel[1] = s_dot*(trajEnd_[1] - trajInit_[1]);
  traj.vel[2] = s_dot*(trajEnd_[2] - trajInit_[2]);

  traj.acc[0] = s_ddot*(trajEnd_[0] - trajInit_[0]);
  traj.acc[1] = s_ddot*(trajEnd_[1] - trajInit_[1]);
  traj.acc[2] = s_ddot*(trajEnd_[2] - trajInit_[2]);

  return traj;

}


trajectory_point KDLPlanner::compute_circular_trajectory_trapezoidal(double time)
{
    double s, s_dot, s_ddot;
    trajectory_point traj;
    trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);

    traj.pos[0] = trajInit_[0];
    traj.pos[1] = trajInit_[1] + trajRadius_ - trajRadius_ * cos(2 * M_PI * s);
    traj.pos[2] = trajInit_[2]- trajRadius_ * sin(2 * M_PI * s);

    traj.vel[0] = 0;
    traj.vel[1] = trajRadius_ * sin (2 * M_PI * s) * 2 * M_PI * s_dot;
    traj.vel[2] = - trajRadius_ * cos(2 * M_PI * s) * 2 * M_PI * s_dot;

    traj.acc[0] = 0;
    traj.acc[1] = 2 * M_PI * trajRadius_ * (s_ddot * sin (2 * M_PI * s) + 2 * M_PI * std::pow(s_dot, 2) * cos(2*M_PI*s));
    traj.acc[2] = - 2 * M_PI * trajRadius_ * (s_ddot * cos (2 * M_PI * s) - 2 * M_PI * std::pow(s_dot, 2) * sin(2*M_PI*s));
 

  return traj;

}