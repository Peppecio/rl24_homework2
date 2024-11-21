#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

KDLController::KDLController()
{
    robot_ = nullptr;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    //read current frame 
    KDL::Frame pos_curr(robot_->getEEFrame());
    //Eigen::Vector3d pos_ang_curr(robot_->getEEFrame().M.data);
    KDL::Twist vel_curr(robot_->getEEVelocity());
    //Eigen::Vector3d vel_ang_curr(robot_->getEEVelocity().rot.data);
    
    //geometric jacobian
    KDL::Jacobian J = robot_->getEEJacobian();
    //pseudoinverse 
    Eigen::Matrix<double,7,6> pseudoJ = pseudoinverse(J.data);

    //jacobian derivate
    Eigen::MatrixXd J_dot;
    J_dot= robot_->getEEJacDot();

    //q
    Eigen::VectorXd q_d= robot_->getJntVelocities();
    
    Vector6d error;
    Vector6d dot_error;

    //compute linear angular error
    computeErrors(_desPos,pos_curr,_desVel,vel_curr,error,dot_error);

    //gain matrix
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();
    
    Eigen::Matrix<double,7,1> y;
    
    y<< pseudoJ*(toEigen(_desAcc) + Kd*dot_error + Kp*error - J_dot*q_d );

    //inertia matrix
    Eigen::MatrixXd B = robot_->getJsim();

    return B*y + robot_->getCoriolis();
}


