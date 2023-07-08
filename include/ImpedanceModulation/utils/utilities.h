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

#ifndef UTILITIES_
#define UTILITIES_

// utils
#include <ImpedanceModulation/utils/algebra.h>

// vector
#include <vector>

/**
 * @brief Class Utils
 * 
 */
class Utilities
{
	// utils ---------------------------------------------------
    Algebra _algebra;

public:

	/////////////// COSTRUCTOR ////////////////////
	/* costructor of the class */
	Utilities();
	
	// std to eigen
	Eigen::VectorXd toEigen(std::vector<double> vec);
    // eigen to std
    std::vector<double> toStdVector(Eigen::VectorXd vecEigen);

    // min repre to quat repr for a pose
    std::vector<double> toQuaternionPose(std::vector<double> poses, bool areDegrees);

	/////////////// DISTRUCTOR ////////////////////
	/* distructor of the class */
	~Utilities();
};

#endif // UTILITIES_