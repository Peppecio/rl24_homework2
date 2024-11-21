// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            //scegli la traiettoria 
            std::cout<<"choose a number:\n";
            std::cout<<"1.  circular-cubic-polinomial\n";
            std::cout<<"2.  linear-cubic-polinomial\n";
            std::cout<<"3.  circular-trapezoidal\n";
            std::cout<<"4.  linear-trapezoidale\n";

            if(!(std::cin>>choice)) {
                std::cerr << "Error: Input not valid.\n";
                std::exit(EXIT_FAILURE);
            }

            std::cout<<"choose a controller:\n";
            std::cout<<"1.  joint-space\n";
            std::cout<<"2.  cartesian-space\n";
          
            
            if(!(std::cin>>choice_1)) {
                std::cerr << "Error: Input not valid.\n";
                std::exit(EXIT_FAILURE);
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);

            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_effort_.resize(nj);

            qd.resize(nj); 
            qdd.resize(nj);
            qddd.resize(nj);

            vecchio_joint_velocities_.resize(nj);
            
            controller_= KDLController(*robot_);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, radius = 0.1;

            if (choice==1) {
                planner_ = KDLPlanner(traj_duration, init_position, radius, end_position); 
                p = planner_.compute_circular_trajectory(t);
            }

            else if (choice==2) {
                planner_ = KDLPlanner(traj_duration, init_position, radius, end_position); 
                p = planner_.compute_linear_trajectory(t);
            }

            else if (choice==3) {
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, radius);
                p = planner_.compute_circular_trajectory_trapezoidal(t);
            }

            else if (choice==4) {
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, radius); 
                p = planner_.compute_linear_trajectory_trapezoidal(t);
            }


            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if (cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if (cmd_interface_=="effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // Send joint torques commands
                for (long int i = 0; i < joint_effort_.data.size(); ++i) { 
                    desired_commands_[i] = joint_effort_(i);
                }

            }
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5;  
            int trajectory_len = 150; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            
            t_+=dt/10;

            if (t_ < total_time){

                if (choice==1) {
                    p = planner_.compute_circular_trajectory(t_);
                }

                else if (choice==2) {
                    p = planner_.compute_linear_trajectory(t_);
                }

                else if (choice==3) {
                    p = planner_.compute_circular_trajectory_trapezoidal(t_);
                }

                else if (choice==4) {
                    p = planner_.compute_linear_trajectory_trapezoidal(t_);
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos); 
            
                KDL::Twist des_vel(toKDL(p.vel),KDL::Vector::Zero());
                KDL::Twist des_acc(toKDL(p.acc),KDL::Vector::Zero());

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if (cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
                else if(cmd_interface_ == "effort") {
                
                    KDL::Twist vel (toKDL(p.vel),KDL::Vector::Zero());
                    KDL::Twist acc (toKDL(p.acc),KDL::Vector::Zero());

                    vecchio_joint_velocities_.data = qdd.data;
                    robot_->getInverseKinematics(desFrame, qd);
                    robot_->getInverseKinematicsVel(vel, qdd);
                    qddd.data = (qdd.data - vecchio_joint_velocities_.data)/dt;

                    if(choice_1==1){
                        joint_effort_.data= controller_.KDLController::idCntr(qd,qdd,qddd,Kp,Kd); //point 3
                    }
                    else if(choice_1==2){
                        joint_effort_.data= controller_.KDLController::idCntr(desFrame,des_vel,des_acc,Kpp,Kpo,Kdp,Kdo); //punto 4

                    }
                    
                } 

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if (cmd_interface_ == "velocity") {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if (cmd_interface_ == "effort"){
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) { ///////////////
                        desired_commands_[i] = joint_effort_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

            }

            else{ 

                qdd.data = Eigen::VectorXd::Zero(7,1);
                qddd.data = Eigen::VectorXd::Zero(7,1);

                KDL::Frame stop_pos = robot_->getEEFrame();
                KDL::Twist stop_vel(KDL::Vector::Zero(),KDL::Vector::Zero());
                KDL::Twist stop_acc(KDL::Vector::Zero(),KDL::Vector::Zero());

                if(choice_1==1){
                    joint_effort_.data= controller_.KDLController::idCntr(qd,qdd,qddd,Kp,Kd); //point 3
                }
                else if(choice_1==2){
                    joint_effort_.data= controller_.KDLController::idCntr(stop_pos,stop_vel,stop_acc,Kpp,Kpo,Kdp,Kdo); //punto 4

                }
    

                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands

                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                for (long int i = 0; i < joint_effort_.data.size(); ++i) { 

                    desired_commands_[i] = joint_effort_(i);
                }
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);    
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_effort_;
        KDL::JntArray vecchio_joint_velocities_;
        KDL::JntArray joint_acceleration;
                

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        KDLController controller_;
        KDL::JntArray qd,qdd,qddd;

        int choice;
        int choice_1;
        trajectory_point p;
      
        double Kd =17;
        double Kp = 250;

        double Kpp = 50;
        double Kpo = 5;
        double Kdp = 10;
        double Kdo = 5;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;

};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}