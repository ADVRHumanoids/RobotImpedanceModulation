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

#ifndef LOGGER_
#define LOGGER_

// matlogger 
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
// Eigen 
#include <Eigen/Dense>

/**
 * @brief Class Data Logger
 *
*/
class LoggerL 
{
    // Logger ---------------------------------------
    XBot::MatLogger2::Ptr _logger; /* mt logger */
    XBot::MatAppender::Ptr _appender; /* mt appender */

    public:
    /* costructor of the class */
    LoggerL();
    /**
     * init plannerLogger
     * @param logFilesPath: string of the path file
     * @return void
     */
    void initLogger(std::string logFilesPath);
    /**
     * set logger mode
     * @param setMode: true: fixed buffer false: unlimited
     * @return void
     */
    void setLogger(bool setMode);
    /**
     * log data eigen vector data
     * @return void
     */
    bool logData(std::string dataName, Eigen::VectorXd data);
    /**
     * log data double data
     * @return void
     */
    bool logData(std::string dataName, double data);

    /* distructor of the class */
    ~LoggerL();

};

#endif // LOGGER_