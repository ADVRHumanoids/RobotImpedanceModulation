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

#ifndef ALGEBRA_
#define ALGEBRA_

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Core>

/**
 * @brief Class Algebra
 * 
 */
class Algebra
{
	//
public:
	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	Algebra();
	
	// pseudo inverse
	Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd M_);

    // get skew symmetric matrix
    void getSkewMatrix(Eigen::VectorXd p, Eigen::MatrixXd& Skew);

    // quat to euler angles in degrees
    Eigen::VectorXd quatToEulerDeg(Eigen::VectorXd quat);
    // quat to euler angles in rads
    Eigen::VectorXd quatToEulerRad(Eigen::VectorXd quat);
    // quat to euler 
    Eigen::VectorXd quatToEuler(Eigen::VectorXd quat, bool areDegrees);
    // eulers to quats - rad
    Eigen::VectorXd eulersToQuatsRad(Eigen::VectorXd eulerAngleOrients);
    // eulers to quats - deg
    Eigen::VectorXd eulersToQuatsDeg(Eigen::VectorXd eulerAngleOrients);
    // eulers to quats
    Eigen::VectorXd eulersToQuats(Eigen::VectorXd eulerAngleOrients, bool areDegrees);
    // euler to quat - return vec type
    Eigen::VectorXd EulerToQuatVec(Eigen::VectorXd eulerAngles);
    // euler to quat - return quat type
    Eigen::Quaternionf EulerToQuat(Eigen::VectorXd eulerAngles);
    // rot to quat x y z w
    Eigen::VectorXd RotToQuat(Eigen::Matrix3d rot);

    // get tranform
    void getTransformationRotTrasl(Eigen::Affine3d T, Eigen::Matrix3d& rot, Eigen::VectorXd& trasl);

    // min to T
    void minToT(Eigen::VectorXd position, Eigen::VectorXd orientation, Eigen::Affine3d& T, bool areDegree);
    // T to min
    void TToMin(Eigen::Affine3d T, Eigen::VectorXd& t);
    // T to min
    void TToMinDeg(Eigen::Affine3d T, Eigen::VectorXd& t);
    // T to min
    void TToMinQuat(Eigen::Affine3d T, Eigen::VectorXd& t);

    // deg To rad
    void degTorad(Eigen::VectorXd deg, Eigen::VectorXd& rad);
    // rad to deg
    Eigen::VectorXd radTodeg(Eigen::VectorXd vecRad);

    // T to min
    Eigen::VectorXd TToMin(Eigen::Affine3d T);
    // T to min
    Eigen::VectorXd TToMinDeg(Eigen::Affine3d T);
    // T to min
    Eigen::VectorXd TToMinQuat(Eigen::Affine3d T);

	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~Algebra();
};

#endif // ALGEBRA_