// 这个头文件定义符号可见性宏（visibility macros）。
//
// 作用：
// 1. 在 Windows / Linux / GCC 等不同平台下统一处理导出导入符号；
// 2. 让 `rplidar_ros` 编译成共享库时，公开符号和内部符号边界清晰；
// 3. 它不包含业务逻辑，只是跨平台构建层辅助头文件。
//
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISIBILITY_H_
#define VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define RPLIDAR_ROS_EXPORT __attribute__ ((dllexport))
    #define RPLIDAR_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define RPLIDAR_ROS_EXPORT __declspec(dllexport)
    #define RPLIDAR_ROS_IMPORT __declspec(dllimport)
  #endif

  #ifdef RPLIDAR_ROS_DLL
    #define RPLIDAR_ROS_PUBLIC RPLIDAR_ROS_EXPORT
  #else
    #define RPLIDAR_ROS_PUBLIC RPLIDAR_ROS_IMPORT
  #endif

  #define RPLIDAR_ROS_PUBLIC_TYPE RPLIDAR_ROS_PUBLIC

  #define RPLIDAR_ROS_LOCAL

#else

  #define RPLIDAR_ROS_EXPORT __attribute__ ((visibility("default")))
  #define RPLIDAR_ROS_IMPORT

  #if __GNUC__ >= 4
    #define RPLIDAR_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define RPLIDAR_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RPLIDAR_ROS_PUBLIC
    #define RPLIDAR_ROS_LOCAL
  #endif

  #define RPLIDAR_ROS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // VISIBILITY_H_
