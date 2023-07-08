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

#include <ImpedanceModulation/utils/logger.h>

LoggerL::LoggerL()
{

}

void LoggerL::initLogger(std::string logFilesPath)
{
	// Create matlogger
	_logger = XBot::MatLogger2::MakeLogger(logFilesPath); // date-time automatically appended
}

void LoggerL::setLogger(bool setMode)
{
	if(setMode) // fixed size 10000
		_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
	else
	{
		_appender = XBot::MatAppender::MakeInstance();
		_appender->add_logger(_logger);
		_appender->start_flush_thread();
	}
}

bool LoggerL::logData(std::string dataName, Eigen::VectorXd data)
{
	// log data
	_logger->add(dataName,data);
	return true;
}

bool LoggerL::logData(std::string dataName, double data)
{
	// log data
	_logger->add(dataName,data);
	return true;
}

LoggerL::~LoggerL()
{

}