#ifndef PID_TUNING__VISIBILITY_CONTROL_H_
#define PID_TUNING__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PID_TUNING_EXPORT __attribute__ ((dllexport))
    #define PID_TUNING_IMPORT __attribute__ ((dllimport))
  #else
    #define PID_TUNING_EXPORT __declspec(dllexport)
    #define PID_TUNING_IMPORT __declspec(dllimport)
  #endif
  #ifdef PID_TUNING_BUILDING_LIBRARY
    #define PID_TUNING_PUBLIC PID_TUNING_EXPORT
  #else
    #define PID_TUNING_PUBLIC PID_TUNING_IMPORT
  #endif
  #define PID_TUNING_PUBLIC_TYPE PID_TUNING_PUBLIC
  #define PID_TUNING_LOCAL
#else
  #define PID_TUNING_EXPORT __attribute__ ((visibility("default")))
  #define PID_TUNING_IMPORT
  #if __GNUC__ >= 4
    #define PID_TUNING_PUBLIC __attribute__ ((visibility("default")))
    #define PID_TUNING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PID_TUNING_PUBLIC
    #define PID_TUNING_LOCAL
  #endif
  #define PID_TUNING_PUBLIC_TYPE
#endif
#endif  // PID_TUNING__VISIBILITY_CONTROL_H_
// Generated 27-Mar-2022 21:04:16
// Copyright 2019-2020 The MathWorks, Inc.
