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

#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include <ImpedanceModulation/ImpedanceModulationManager.h>

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	ImpedanceModulationManager manager("impedance_modulation");
	manager.spin();

	return 0;
}
