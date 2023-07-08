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

#ifndef KDL_HELPER_
#define KDL_HELPER_

// std library
#include <stdio.h>
#include <iostream>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

// kdl
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
// #include <urdf/model.h>

/**
 * @brief Class KDLHelper
 * 
 */
class KDLHelper
{
	//
public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	KDLHelper();

    // init
    void init(std::string fileURDFpath, std::string base_frame, std::string tip_frame);
	
    // set joints position
    void setJointsPositions(Eigen::VectorXd q);

    // set frame pose
    void setPose(Eigen::VectorXd pose);

    // compute fk
    void FK();

    // compute ik
    void IK();

    // get frame pose
    Eigen::VectorXd getPose();

    // get joints positions
    Eigen::VectorXd getJointsPositions();

    // get jacobian
    Eigen::MatrixXd getJacobian();

    // get gravity torque
    Eigen::VectorXd getGravity();

	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~KDLHelper();

private:

    // Robot data -----------------------------------------------------
    unsigned int _nc;
    unsigned int _nj;
    KDL::Tree _tree;
    KDL::Chain _chain;
    KDL::Frame _pose;
    KDL::JntArray _q;

    // Solvers ---------------------------------------------------------
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> _fksolver;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> _iksolver1v;//Inverse velocity solver
    std::unique_ptr<KDL::ChainIkSolverPos_NR> _iksolver;

    std::unique_ptr<KDL::ChainJntToJacSolver> _jnt_to_jac_solver;
    KDL::Jacobian _jacobian;

    std::unique_ptr<KDL::ChainDynParam> _dyn;

};

#endif // KDL_HELPER_