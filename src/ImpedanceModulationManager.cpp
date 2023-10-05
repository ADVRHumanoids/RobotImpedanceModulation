/*
 * Copyright (C) 2023 IIT-HHCM
 * Author: Liana Bertoni
 * email:  liana.bertoni@iit.it
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <ImpedanceModulation/ImpedanceModulationManager.h>

ImpedanceModulationManager::ImpedanceModulationManager ( std::string ns )
{
    // ros2 'init'
    _nh = rclcpp::Node::make_shared(ns);

    //initialization ROS node
    RCLCPP_INFO(_nh->get_logger(), "%s\n","I am initializing the ROS node...");
    initROSNode();

    //initialization ROS node
    RCLCPP_INFO(_nh->get_logger(), "%s\n","I am loading the ROS node params...");
    loadParam();

    //initialization ROS node
    RCLCPP_INFO(_nh->get_logger(), "%s\n","I am initializing the robot impedance...");
    initRobotImpedance();

}

void ImpedanceModulationManager::initROSNode()
{
    // init ROS node
    int rate;
    _nh->declare_parameter("rate", 1000);
    _nh->get_parameter("rate", rate);
    _rate = rate;
    _timer = _nh->create_wall_timer( std::chrono::milliseconds(rate),std::bind(&ImpedanceModulationManager::timer_callback, this));

    _nh->declare_parameter("topic_subscriber_name","task_planner");
    _nh->declare_parameter("topic_publisher_name","robot_IM_planner");
    _nh->get_parameter("topic_subscriber_name",_topicSub);
    _nh->get_parameter("topic_publisher_name",_topicPub);
    _taskSubscriber = _nh->create_subscription<rim::msg::TaskMsg>(_topicSub,1, std::bind(&ImpedanceModulationManager::TaskParamSubscriberCallback, this, _1));
    _robotPublisher = _nh->create_publisher<rim::msg::ImpedanceMsg>(_topicPub,1000);

    // logger
    _nh->declare_parameter("log_path","/tmp/");
    _nh->get_parameter("log_path",_logPath);
    _loggerL.initLogger(_logPath);
    _loggerL.setLogger(false); // false:unlimited true:limited buffer
}

void ImpedanceModulationManager::loadParam()
{
    std::vector<double> q0;
    std::vector<double> k_preset;
    std::vector<double> k_0;
    std::vector<double> k_max;
    std::vector<double> d_preset;
    std::vector<double> d_max;
    std::vector<double> w_ee_d_initial;
    std::vector<double> delta_x_ee_initial;

    // declaring params with default values
    _nh->declare_parameter("verbose",false);
    _nh->declare_parameter("stiffness_preset",k_preset);
    _nh->declare_parameter("stiffness_constant",k_0);
    _nh->declare_parameter("stiffness_maximum",k_max);

    _nh->declare_parameter("damping_preset",d_preset);
    _nh->declare_parameter("damping_maximum",d_max);

    _nh->declare_parameter("wrench_initial",w_ee_d_initial);
    _nh->declare_parameter("precision_initial",delta_x_ee_initial);

    _nh->declare_parameter("robot_initial_config",q0);
    _nh->declare_parameter("robot_urdf_model_path","/tmp/robot.urdf");
    _nh->declare_parameter("robot_base_frame_name","base_link");
    _nh->declare_parameter("robot_tip_frame_name","end_effector");

    _nh->declare_parameter("transition_time",5.0);

    // getting params with their proper values
    _nh->get_parameter("verbose",_verbose);
    _nh->get_parameter("stiffness_preset",k_preset);
    _nh->get_parameter("stiffness_constant",k_0);
    _nh->get_parameter("stiffness_maximum",k_max);

    _nh->get_parameter("damping_preset",d_preset);
    _nh->get_parameter("damping_maximum",d_max);

    _nh->get_parameter("wrench_initial",w_ee_d_initial);
    _nh->get_parameter("precision_initial",delta_x_ee_initial);    

    _nh->get_parameter("robot_initial_config",q0);
    _nh->get_parameter("robot_urdf_model_path",_robotURDFModelPath);
    _nh->get_parameter("robot_base_frame_name",_base);
    _nh->get_parameter("robot_tip_frame_name",_endEffector);

    _nh->get_parameter("transition_time",_ref_time);

    _k_preset = _utils.toEigen(k_preset);
    _k_0 = _utils.toEigen(k_0);
    _k_max = _utils.toEigen(k_max);
    _d_preset = _utils.toEigen(d_preset);
    _d_max = _utils.toEigen(d_max);
    _w_ee_d_initial = _utils.toEigen(w_ee_d_initial);
    _delta_x_ee_initial = _utils.toEigen(delta_x_ee_initial);
    _q = _utils.toEigen(q0);

    _kdl.init(_robotURDFModelPath,_base,_endEffector);
    _kdl.setJointsPositions(_q);

    initVars();
}

void ImpedanceModulationManager::initVars()
{
    // cartesian space
    _w_ee_d = Eigen::VectorXd::Zero(_nc);
    _w_ee_td = Eigen::VectorXd::Zero(_nc);
    _w_ee_g = Eigen::VectorXd::Zero(_nc);
    _delta_x_ee_td = Eigen::VectorXd::Zero(_nc);
    _k_c_d = Eigen::VectorXd::Zero(_nc);
    _K_C_d = Eigen::MatrixXd::Identity(_nc,_nc);

    // joint space
    _nj = _k_0.size();
    _qref = Eigen::VectorXd::Zero(_nj);
    _qerr = Eigen::VectorXd::Zero(_nj);

    _k = Eigen::VectorXd::Zero(_nj);    
    _d = Eigen::VectorXd::Zero(_nj);
    _K_0 = _k_0.asDiagonal();
    _K_J = Eigen::MatrixXd::Identity(_nj,_nj);
    _D_J = Eigen::MatrixXd::Identity(_nj,_nj);
    _K_J_off = Eigen::MatrixXd::Identity(_nj,_nj);

    _tau_g = Eigen::VectorXd::Zero(_nj);
    _tau_ext = Eigen::VectorXd::Zero(_nj);
    _tau_Koff= Eigen::VectorXd::Zero(_nj);
    _tau_ff= Eigen::VectorXd::Zero(_nj);

    // state
    _subscribed = false;
}

void ImpedanceModulationManager::initRobotImpedance()
{
    // smoothing
    double t = 0.0;
    double time = 0.0;
    double it_time = 0.0;

    Eigen::VectorXd taug;
    taug = getGravityCompensation();
    Eigen::MatrixXd _J_invT;
    _J = getJacobian();

    _J_invT = _algebra.pseudoInverse(_J.transpose());

    Eigen::VectorXd k;
    Eigen::VectorXd d;

    // smoothing stiffness and damping and torque
    while( it_time < 1.0 )
    {
        it_time = time/_ref_time;
        t = ((6*it_time - 15)*it_time + 10)*it_time*it_time*it_time;

        _tau_g = ( 1 - t ) * Eigen::VectorXd::Zero(_nj) +  t * taug;
        _w_ee_td = ( 1 - t ) * Eigen::VectorXd::Zero(_nj) +  t * _w_ee_d_initial;
        _w_ee_g = _J_invT * _tau_g;
        
        _w_ee_d = _w_ee_g + _w_ee_td;
        _w_ee_d = _w_ee_d.cwiseAbs();
        _k_c_d = _w_ee_d.cwiseProduct(_delta_x_ee_initial.cwiseInverse());
        _K_C_d = _k_c_d.asDiagonal();
        _K_J = _K_0 + _J.transpose() * _K_C_d * _J;

        k = _K_J.diagonal();
        d = 2*_csi*_k.cwiseAbs().cwiseSqrt();

        Eigen::MatrixXd Kapp;
        Kapp = k.asDiagonal();
        _K_J_off = _K_J - Kapp;

        _tau_Koff = ( 1 - t ) * Eigen::VectorXd::Zero(_nj) +  t * _K_J_off * _qerr;
        _tau_ext = _J.transpose() * _w_ee_td;
        
        _k = ( 1 - t ) * _k_preset + t * k;
        _d = ( 1 - t ) * _d_preset + t * d;
        _tau_ff = _tau_g + _tau_Koff + _tau_ext;

        checkLimits();

        // compose robot msg
        composeRobotMsg();
        _robotPublisher->publish(_robotMsg);

        // iteration step
        time += 1/_rate;
        log();
    }
}

void ImpedanceModulationManager::computeRobotImpedanceModulation()
{
    // cartesian stiffness calculations
    _w_ee_g = getGravityWrench();
    _w_ee_d = _w_ee_g + _w_ee_td;
    _w_ee_d = _w_ee_d.cwiseAbs();
    _k_c_d = _w_ee_d.cwiseProduct(_delta_x_ee_td.cwiseInverse());

    // joints stiffness calculations
    _J = getJacobian();
    _K_C_d = _k_c_d.asDiagonal();
    _K_J = _K_0 + _J.transpose() * _K_C_d * _J;
    _k = _K_J.diagonal();

    Eigen::MatrixXd Kapp;
    Kapp = _k.asDiagonal();
    _K_J_off = _K_J - Kapp;

    // joints damping calculations
    _d =  2*_csi*_k.cwiseAbs().cwiseSqrt();
    _D_J.diagonal() = _d;

    // torques calculations
    _qerr = _qref - _q;
    _tau_g = getGravityCompensation();
    _tau_Koff = _K_J_off * _qerr;
    _tau_ext = _J.transpose() * _w_ee_td;
    _tau_ff =_tau_g + _tau_Koff + _tau_ext;

    // check limits for motors
    checkLimits();
}

void ImpedanceModulationManager::checkLimits()
{
    for (int i = 0; i < _nj; ++i)
    {
        if( _k(i) > _k_max(i) )
            _k(i) = _k_max(i);

        if( _d(i) > _d_max(i) )
            _d(i) = _d_max(i);
    }
}

void ImpedanceModulationManager::composeRobotMsg()
{
    // compose message
    _robotMsg.robot_stiffness = _utils.toStdVector(_k);
    _robotMsg.robot_damping = _utils.toStdVector(_d);
    _robotMsg.robot_feedforward_torque = _utils.toStdVector(_tau_ff);
    _robotMsg.joints_position_reference = _utils.toStdVector(_qref);
}

void ImpedanceModulationManager::timer_callback()
{
    // control loop variable impedance modulation
    if (_subscribed)
    {
        if(_verbose)
        {
            std::cout << "Subscribed data..." << std::endl;
            std::cout << "Joints state: " << _q.transpose() << std::endl;
            std::cout << "Joints reference: " << _qref.transpose() << std::endl;
            std::cout << "Wrench reference: " << _w_ee_td.transpose() << std::endl;
            std::cout << "Precision reference: " << _delta_x_ee_td.transpose() << std::endl;
        }

        // iteration for the planning
        computeRobotImpedanceModulation();
        composeRobotMsg();

         // pub
        _robotPublisher->publish(_robotMsg);

         // log
        log();

        _subscribed = false;
    }
}

void ImpedanceModulationManager::spin()
{
    std::cout << "------------------------------------------------------------------------------" << std::endl;
    std::cout << "Robot Impedance Modulation started! Ready to accept task planning..." << std::endl;
    rclcpp::spin(_nh);
}

void ImpedanceModulationManager::TaskParamSubscriberCallback(const rim::msg::TaskMsg::SharedPtr msg)
{
    // task param : inputs
    if(msg->cartesian_space)
    {
        _kdl.setPose(_utils.toEigen(msg->task_pose_reference));
        _kdl.FK();
        _qref = _kdl.getJointsPositions();
    }
    else
    {   
        _qref = _utils.toEigen(msg->joints_position_reference);
    }

    _q = _utils.toEigen(msg->joints_position);
    _w_ee_td = _utils.toEigen(msg->task_wrench);
    _delta_x_ee_td = _utils.toEigen(msg->task_precision);

    _subscribed = true;
}

Eigen::MatrixXd ImpedanceModulationManager::getJacobian()
{
    // getting jacobian matrix
    _kdl.setJointsPositions(_q);
    return _kdl.getJacobian();
}

Eigen::VectorXd ImpedanceModulationManager::getGravityCompensation()
{
    // gravity wrench
    _kdl.setJointsPositions(_q);
    return _kdl.getGravity();
}

Eigen::VectorXd ImpedanceModulationManager::getGravityWrench()
{
    // gravity wrench
    _J = getJacobian();
    return _algebra.pseudoInverse(_J.transpose()) * getGravityCompensation();
}

void ImpedanceModulationManager::log()
{
    // log data
    _loggerL.logData("EEwrench",_w_ee_d);
    _loggerL.logData("EETaskwrench",_w_ee_td);
    _loggerL.logData("TaskErrorDesired",_delta_x_ee_td);
    _loggerL.logData("CartesianStiffness",_k_c_d);
    _loggerL.logData("JointStiffness",_k);
    _loggerL.logData("JointDamping",_d);
    _loggerL.logData("FeedforwardTorque",_tau_ff);

    _loggerL.logData("JointsPosition",_q);
    _loggerL.logData("JointsPositionReference",_qref);
}

ImpedanceModulationManager::~ImpedanceModulationManager()
{

}