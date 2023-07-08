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

#include <ImpedanceModulation/utils/algebra.h>

Algebra::Algebra()
{

}

Eigen::MatrixXd Algebra::pseudoInverse(Eigen::MatrixXd M_)
{
    // psuedo-inverse of a not-square matrix (SVD based)
    Eigen::BDCSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double epsilon = std::numeric_limits<double>::epsilon();
    svd.setThreshold(epsilon*std::max(M_.cols(), M_.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::MatrixXd tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

void Algebra::getSkewMatrix(Eigen::VectorXd p, Eigen::MatrixXd& Skew)
{
    Skew = Eigen::MatrixXd::Zero(3,3);
    Skew(0,1) = -p(2);
    Skew(0,2) =  p(1);
    Skew(1,0) =  p(2);
    Skew(1,2) = -p(0);
    Skew(2,0) = -p(1);
    Skew(2,1) =  p(0);
}

Eigen::VectorXd Algebra::quatToEulerDeg(Eigen::VectorXd quat) // quat : x y z w 
{
    return radTodeg(quatToEulerRad(quat));
}

Eigen::VectorXd Algebra::quatToEulerRad(Eigen::VectorXd quat)
{
    Eigen::Vector3d eulerAnglesDeg;
    Eigen::Quaterniond q;
    q.x() = quat(0);
    q.y() = quat(1);
    q.z() = quat(2);
    q.w() = quat(3);
    q.normalize();

    Eigen::Matrix3d rot;
    rot = q.toRotationMatrix();
    eulerAnglesDeg = rot.eulerAngles(0,1,2); // roll-x pitch-y yaw-z

    Eigen::VectorXd vecReturn(3);
    vecReturn(0) = eulerAnglesDeg(0);
    vecReturn(1) = eulerAnglesDeg(1);
    vecReturn(2) = eulerAnglesDeg(2);

    return vecReturn;
}

Eigen::VectorXd Algebra::quatToEuler(Eigen::VectorXd quat, bool areDegrees)
{
    if(areDegrees)
        return quatToEulerDeg(quat);
    else
        return quatToEulerRad(quat);
}

Eigen::VectorXd Algebra::eulersToQuatsRad(Eigen::VectorXd eulerAngleOrients)
{
    int dim = (eulerAngleOrients.size()/3)*4;
    Eigen::VectorXd quatOrients(dim);
    int j = 0;
    for (int i = 0; i < dim/4; i+=4)
    {
        quatOrients.segment<4>(i) = EulerToQuatVec(eulerAngleOrients.segment<3>(j));
        j += 3;
    }

    return quatOrients;
}

Eigen::VectorXd Algebra::eulersToQuatsDeg(Eigen::VectorXd eulerAngleOrients)
{
    return radTodeg(eulersToQuatsRad(eulerAngleOrients));
}

Eigen::VectorXd Algebra::eulersToQuats(Eigen::VectorXd eulerAngleOrients, bool areDegrees)
{
    if(areDegrees)
        return eulersToQuatsDeg(eulerAngleOrients);
    else
        return eulersToQuatsRad(eulerAngleOrients);
}

Eigen::VectorXd Algebra::EulerToQuatVec(Eigen::VectorXd eulerAngles)
{
    Eigen::VectorXd quatVec(4);
    Eigen::Quaternionf q;
    q = EulerToQuat(eulerAngles);

    quatVec(0) = q.x();
    quatVec(1) = q.y();
    quatVec(2) = q.z();
    quatVec(3) = q.w();

    return quatVec;
}

Eigen::Quaternionf Algebra::EulerToQuat(Eigen::VectorXd eulerAngles)
{
    // psuedo-inverse of a not-square matrix (SVD based)
    degTorad(eulerAngles,eulerAngles);
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(eulerAngles[2], Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(eulerAngles[1], Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(eulerAngles[0], Eigen::Vector3f::UnitX());
    return q;
}

Eigen::VectorXd Algebra::RotToQuat(Eigen::Matrix3d rot)
{
    Eigen::VectorXd q_vec(4); // x y z w 
    Eigen::Quaterniond q(rot);

    q_vec(0) = q.x();
    q_vec(1) = q.y();
    q_vec(2) = q.z();
    q_vec(3) = q.w();

    return q_vec;
}

void Algebra::getTransformationRotTrasl(Eigen::Affine3d T, Eigen::Matrix3d& rot, Eigen::VectorXd& trasl)
{
    trasl = T.translation();
    rot = T.rotation();
}

void Algebra::minToT(Eigen::VectorXd position, Eigen::VectorXd orientation, Eigen::Affine3d& T, bool areDegree)
{
    // set translation
    T = Eigen::Affine3d::Identity();
    T.translation()[0] = position[0];
    T.translation()[1] = position[1];
    T.translation()[2] = position[2];
    // set orientation
    // set quat
    Eigen::Quaterniond q;
    if(orientation.size() == 3)
    {
        if(areDegree)
            degTorad(orientation,orientation);

        q = Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX());
    }
    else
    {
        q.x() = orientation[0];
        q.y() = orientation[1];
        q.z() = orientation[2];
        q.w() = orientation[3];
    }
    
    T.rotate(q.toRotationMatrix());
}

void Algebra::TToMin(Eigen::Affine3d T, Eigen::VectorXd& t)
{
    t = Eigen::VectorXd::Zero(6);
    t.head(3) = T.translation();
    t.tail<3>() = T.rotation().eulerAngles(0,1,2);
}

void Algebra::TToMinDeg(Eigen::Affine3d T, Eigen::VectorXd& t)
{
    t = Eigen::VectorXd::Zero(6);
    t.head(3) = T.translation();
    t.tail<3>() = T.rotation().eulerAngles(0,1,2);
    t(3) = t(3) * (180.0/M_PI);
    t(4) = t(4) * (180.0/M_PI);
    t(5) = t(5) * (180.0/M_PI);
}

void Algebra::TToMinQuat(Eigen::Affine3d T, Eigen::VectorXd& t)
{
    t = Eigen::VectorXd::Zero(7);
    t.head(3) = T.translation();
    t.tail<4>() = RotToQuat(T.rotation());
}

Eigen::VectorXd Algebra::TToMin(Eigen::Affine3d T)
{
    Eigen::VectorXd t;
    TToMin(T,t);
    return t;
}

Eigen::VectorXd Algebra::TToMinDeg(Eigen::Affine3d T)
{
    Eigen::VectorXd t;
    TToMinDeg(T,t);
    return t;
}

Eigen::VectorXd Algebra::TToMinQuat(Eigen::Affine3d T)
{
    Eigen::VectorXd t;
    TToMinQuat(T,t);
    return t;
}

void Algebra::degTorad(Eigen::VectorXd deg, Eigen::VectorXd& rad)
{
    rad = Eigen::VectorXd::Zero(deg.size());
    for (int i = 0; i < deg.size(); ++i)
        rad[i] = deg[i]*(M_PI/180.0);
}

Eigen::VectorXd Algebra::radTodeg(Eigen::VectorXd vecRad)
{
    Eigen::VectorXd vecDeg(vecRad.size());
    for (int i = 0; i < vecRad.size(); ++i)
        vecDeg[i] = vecRad[i]*(180.0/M_PI);

    return vecDeg;
}

Algebra::~Algebra()
{

}
