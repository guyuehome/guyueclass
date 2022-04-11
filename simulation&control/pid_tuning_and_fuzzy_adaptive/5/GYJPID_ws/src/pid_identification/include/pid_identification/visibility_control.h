#ifndef PID_IDENTIFICATION__VISIBILITY_CONTROL_H_
#define PID_IDENTIFICATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PID_IDENTIFICATION_EXPORT __attribute__ ((dllexport))
    #define PID_IDENTIFICATION_IMPORT __attribute__ ((dllimport))
  #else
    #define PID_IDENTIFICATION_EXPORT __declspec(dllexport)
    #define PID_IDENTIFICATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef PID_IDENTIFICATION_BUILDING_LIBRARY
    #define PID_IDENTIFICATION_PUBLIC PID_IDENTIFICATION_EXPORT
  #else
    #define PID_IDENTIFICATION_PUBLIC PID_IDENTIFICATION_IMPORT
  #endif
  #define PID_IDENTIFICATION_PUBLIC_TYPE PID_IDENTIFICATION_PUBLIC
  #define PID_IDENTIFICATION_LOCAL
#else
  #define PID_IDENTIFICATION_EXPORT __attribute__ ((visibility("default")))
  #define PID_IDENTIFICATION_IMPORT
  #if __GNUC__ >= 4
    #define PID_IDENTIFICATION_PUBLIC __attribute__ ((visibility("default")))
    #define PID_IDENTIFICATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PID_IDENTIFICATION_PUBLIC
    #define PID_IDENTIFICATION_LOCAL
  #endif
  #define PID_IDENTIFICATION_PUBLIC_TYPE
#endif
#endif  // PID_IDENTIFICATION__VISIBILITY_CONTROL_H_
// Generated 27-Mar-2022 13:29:00
// Copyright 2019-2020 The MathWorks, Inc.
