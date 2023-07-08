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

#ifndef IMPEDANCE_MODULATION_
#define IMPEDANCE_MODULATION_

// ROS 
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "rim/msg/task_msg.hpp"
#include "rim/msg/impedance_msg.hpp"

// utils
#include <ImpedanceModulation/utils/KDLhelper.h>
#include <ImpedanceModulation/utils/algebra.h>
#include <ImpedanceModulation/utils/utilities.h>
#include <ImpedanceModulation/utils/logger.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
/**
 * @brief  ImpedanceModulation class is a inherited ControlPlugin Class
 *         implements joint impedance modulation.
 */
class ImpedanceModulationManager
{

public:

    /////////////// COSTRUCTOR ////////////////////
    /* costructor of the class */
    ImpedanceModulationManager( std::string ns = "" );
    /////////// COMPUTATION functions ////////////
    /**
    * callback of the ROS Node
    * @return void
    */
    void timer_callback();
    /**
    * spin function
    * @return void
    */
    void spin();
    /////////////// DISTRUCTOR ////////////////////
    /* distructor of the class */
    ~ImpedanceModulationManager();

private:

    /**
     * init ROS Node
     * @return void
     */
    void initROSNode();
    /**
     * load params
     * @return void
     */
    void loadParam();
    /**
     * init vars
     * @return void
     */
    void initVars();
    /**
     * init robot impedance stiffness and damping
     * @return void
     */
    void initRobotImpedance();
    /**
     * compute robot impedance plus torques [stiffness,damping,torques]
     * @return void
     */
    void computeRobotImpedanceModulation();
    /**
     * @return void
     */
    void checkLimits();
    /**
     *  subscribe task params
     *  @return void
     */
    void TaskParamSubscriberCallback(const rim::msg::TaskMsg::SharedPtr msg);
    /**
     * create robot message
     * @return void
     */
    void composeRobotMsg();
    /**
     * compute jacobian
     */
    Eigen::MatrixXd getJacobian();
    /**
     * compute gravity wrench
     */
    Eigen::VectorXd getGravityWrench();
    /**
     * compute gravity torque
     */
    Eigen::VectorXd getGravityCompensation();
    /**
     * log data
     * @return void
     */
    void log();
    
    //  ROS ----------------------------------------------------------------
    rclcpp::Node::SharedPtr _nh; /* ROSE node handle */
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<rim::msg::TaskMsg>::SharedPtr _taskSubscriber; /* ROS topic object model subscriber */
    rclcpp::Publisher<rim::msg::ImpedanceMsg>::SharedPtr _robotPublisher; /* ROS topic torques publisher */
    std::string _topicSub;
    std::string _topicPub;
    double _rate;

    //  Utilities ----------------------------------------------------------
    Algebra _algebra; // algebra functions container
    Utilities _utils; // utils
    LoggerL _loggerL; // data logger
    std::string _logPath;

    
    KDLHelper _kdl; // kdl
    std::string _robotURDFModelPath;
    std::string _base;
    std::string _endEffector;

    // Data ----------------------------------------------------------------
    // Cartesian Space
    int _nc = 6;
    Eigen::VectorXd _w_ee_d;
    Eigen::VectorXd _w_ee_td;
    Eigen::VectorXd _w_ee_g;
    Eigen::VectorXd _w_ee_d_initial;
    
    Eigen::VectorXd _delta_x_ee_td;
    Eigen::VectorXd _delta_x_ee_initial;

    Eigen::VectorXd _k_c_d;
    Eigen::MatrixXd _K_C_d;
    
    // Joint Space
    int _nj;
    double _csi = 0.7;
    Eigen::MatrixXd _K_0;
    Eigen::MatrixXd _K_J;
    Eigen::MatrixXd _D_J;
    Eigen::MatrixXd _K_J_off;

    Eigen::VectorXd _k;
    Eigen::VectorXd _k_0;
    Eigen::VectorXd _k_preset;
    Eigen::VectorXd _k_max;

    Eigen::VectorXd _d;
    Eigen::VectorXd _d_preset;
    Eigen::VectorXd _d_max;

    Eigen::VectorXd _tau_ff;
    Eigen::VectorXd _tau_g;
    Eigen::VectorXd _tau_ext;
    Eigen::VectorXd _tau_Koff;

    Eigen::MatrixXd _J; // Jacobian
    Eigen::VectorXd _q,_qref,_qerr; // joints variable

    double _ref_time;

    rim::msg::ImpedanceMsg _robotMsg;

    // IM status --------------------------------------------------------
    bool _subscribed;
    bool _verbose;
};

#endif // JOINT_IMPEDANCE_CONTROL_H
