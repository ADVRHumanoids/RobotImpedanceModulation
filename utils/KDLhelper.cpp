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

#include <ImpedanceModulation/utils/KDLhelper.h>

KDLHelper::KDLHelper()
{

}

void KDLHelper::init(std::string fileURDFpath, std::string base_frame, std::string tip_frame)
{
    // parse URDF
    if (!kdl_parser::treeFromFile(fileURDFpath,_tree))
    {
        std::cout << "Failed to parse URDF -> NO kdl tree" << std::endl;
        return;
    }

    _nc = 6;
    _tree.getChain(base_frame,tip_frame,_chain);
    _nj = _chain.getNrOfJoints();
    _q = KDL::JntArray(_nj);
    _q.resize(_nj);

    _fksolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(_chain);
    _iksolver1v = std::make_unique<KDL::ChainIkSolverVel_pinv>(_chain);
    _iksolver = std::make_unique<KDL::ChainIkSolverPos_NR>(_chain,*_fksolver.get(),*_iksolver1v.get());

    _jnt_to_jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(_chain);
    _jacobian.resize(_nj);

    _dyn = std::make_unique<KDL::ChainDynParam>(_chain,KDL::Vector(0.0,0.0,-9.81));
}

void KDLHelper::setJointsPositions(Eigen::VectorXd q)
{
    for (int i = 0; i < _nj; ++i)
        _q(i) = q(i);
}

void KDLHelper::setPose(Eigen::VectorXd pose)
{
    KDL::Rotation R = KDL::Rotation::RPY(pose(3),pose(4),pose(5));
    KDL::Vector t(pose(0),pose(1),pose(2));

    _pose = KDL::Frame(R,t);
}

void KDLHelper::FK()
{
    bool kinematics_status;
    kinematics_status = _fksolver->JntToCart(_q,_pose);
}

void KDLHelper::IK()
{
    int ret = _iksolver->CartToJnt(_q,_pose,_q);
}

Eigen::VectorXd KDLHelper::getPose()
{
    Eigen::VectorXd pose;
    pose = Eigen::VectorXd::Zero(_nc);

    pose(0) = _pose.p.x();
    pose(1) = _pose.p.y();
    pose(2) = _pose.p.z();
    _pose.M.GetRPY(pose(3),pose(4),pose(5)); 

    return pose;
}

Eigen::VectorXd KDLHelper::getJointsPositions()
{
    Eigen::VectorXd q;
    q = Eigen::VectorXd::Zero(_nj);
    for (int i = 0; i < _nj; ++i)
        q(i) = _q(i);

    return q;
}

Eigen::MatrixXd KDLHelper::getJacobian()
{
    Eigen::MatrixXd J;
    _jnt_to_jac_solver->JntToJac(_q,_jacobian,-1);
    J = Eigen::MatrixXd::Identity(_jacobian.data.rows(),_jacobian.data.cols());

    for (int i = 0; i < J.rows(); ++i)
        for (int j = 0; j < J.cols(); ++j)
            J(i,j) = _jacobian.data(i,j);

    return J;
}

Eigen::VectorXd KDLHelper::getGravity()
{
    Eigen::VectorXd tau_g;
    tau_g = Eigen::VectorXd::Zero(_nj);

    KDL::JntArray jgrav(_nj);
    _dyn->JntToGravity(_q,jgrav);

    for (int i = 0; i < _nj; ++i)
        tau_g(i) = jgrav(i);

    return tau_g;
}

KDLHelper::~KDLHelper()
{

}
