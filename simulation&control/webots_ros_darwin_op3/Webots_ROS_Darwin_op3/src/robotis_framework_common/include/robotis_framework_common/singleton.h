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
 * singleton.h
 *
 *  Created on: 2016. 5. 17.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_
#define ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_


namespace robotis_framework
{

template <class T>
class Singleton
{
private:
  static T *unique_instance_;

protected:
  Singleton() { }
  Singleton(Singleton const&) { }
  Singleton& operator=(Singleton const&) { return *this; }

public:
  static T* getInstance()
  {
    if(unique_instance_ == NULL)
      unique_instance_ = new T;
    return unique_instance_;
  }

  static void destroyInstance()
  {
    if(unique_instance_)
    {
      delete unique_instance_;
      unique_instance_ = NULL;
    }
  }
};

template <class T> T* Singleton<T>::unique_instance_ = NULL;

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_ */
