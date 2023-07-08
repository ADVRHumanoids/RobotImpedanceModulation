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

#include <ImpedanceModulation/utils/utilities.h>

Utilities::Utilities()
{

}

Eigen::VectorXd Utilities::toEigen(std::vector<double> vec)
{
    // psuedo-inverse of a not-square matrix (SVD based)
    Eigen::VectorXd vecEigen(vec.size());
    for (int i = 0; i < vec.size(); ++i)
        vecEigen[i] = vec[i];
    return vecEigen;
}

std::vector<double> Utilities::toStdVector(Eigen::VectorXd vecEigen)
{
    std::vector<double> vecStd;
    for (int i = 0; i < vecEigen.size(); ++i)
        vecStd.push_back(vecEigen[i]);

    return vecStd;
}

std::vector<double> Utilities::toQuaternionPose(std::vector<double> poses, bool areDegrees)
{
    // to eigen
    Eigen::VectorXd vecEigenPoses;
    vecEigenPoses = toEigen(poses);

    // getting orientation parts
    int nposes = (poses.size()/6);
    int dim = nposes*3;
    Eigen::VectorXd vecEigenAnglesPoses(dim);
    int j = 3;
    for (int i = 0; i < nposes; i+=3)
    {
        vecEigenAnglesPoses.segment<3>(i) = vecEigenPoses.segment<3>(j);
        j+=6;
    }

    // getting the orientations as a quaternions
    Eigen::VectorXd vecEigenQuatOrients;
    vecEigenQuatOrients = _algebra.eulersToQuats(vecEigenAnglesPoses,areDegrees);

    // recombining poses
    dim = nposes*7;
    Eigen::VectorXd vecEigenQuatPoses(dim);
    j = 0;
    int k = 0;
    for (int i = 0; i < nposes; i+=7)
    {
        vecEigenQuatPoses.segment<3>(i) = vecEigenPoses.segment<3>(j);
        vecEigenQuatPoses.segment<4>(i+3) = vecEigenQuatOrients.segment<4>(k);
        j+=6;
        k+=4;
    }

    return toStdVector(vecEigenQuatPoses);
}

Utilities::~Utilities()
{

}
