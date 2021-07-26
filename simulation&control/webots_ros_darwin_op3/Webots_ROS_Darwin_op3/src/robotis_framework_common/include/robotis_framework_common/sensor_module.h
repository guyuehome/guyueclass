/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * sensor_module.h
 *
 *  Created on: 2016. 3. 30.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_


#include <map>
#include <string>

#include "singleton.h"

namespace robotis_framework
{

class SensorModule
{
protected:
  std::string module_name_;

public:
  std::map<std::string, double> result_;

  virtual ~SensorModule() { }

  std::string   getModuleName() { return module_name_; }

  virtual void  initialize(const int control_cycle_msec) = 0;
  virtual void  process( std::map<std::string, double> sensors) = 0;
};

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_SENSOR_MODULE_H_ */
