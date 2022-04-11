#ifndef FUZZYPID__VISIBILITY_CONTROL_H_
#define FUZZYPID__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FUZZYPID_EXPORT __attribute__ ((dllexport))
    #define FUZZYPID_IMPORT __attribute__ ((dllimport))
  #else
    #define FUZZYPID_EXPORT __declspec(dllexport)
    #define FUZZYPID_IMPORT __declspec(dllimport)
  #endif
  #ifdef FUZZYPID_BUILDING_LIBRARY
    #define FUZZYPID_PUBLIC FUZZYPID_EXPORT
  #else
    #define FUZZYPID_PUBLIC FUZZYPID_IMPORT
  #endif
  #define FUZZYPID_PUBLIC_TYPE FUZZYPID_PUBLIC
  #define FUZZYPID_LOCAL
#else
  #define FUZZYPID_EXPORT __attribute__ ((visibility("default")))
  #define FUZZYPID_IMPORT
  #if __GNUC__ >= 4
    #define FUZZYPID_PUBLIC __attribute__ ((visibility("default")))
    #define FUZZYPID_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FUZZYPID_PUBLIC
    #define FUZZYPID_LOCAL
  #endif
  #define FUZZYPID_PUBLIC_TYPE
#endif
#endif  // FUZZYPID__VISIBILITY_CONTROL_H_
// Generated 30-Mar-2022 17:31:38
// Copyright 2019-2020 The MathWorks, Inc.
