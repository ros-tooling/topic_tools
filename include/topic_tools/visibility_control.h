// Copyright 2022 Melvin Wang
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

#ifndef TOPIC_TOOLS__VISIBILITY_CONTROL_H_
#define TOPIC_TOOLS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TOPIC_TOOLS_EXPORT __attribute__ ((dllexport))
    #define TOPIC_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define TOPIC_TOOLS_EXPORT __declspec(dllexport)
    #define TOPIC_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TOPIC_TOOLS_BUILDING_LIBRARY
    #define TOPIC_TOOLS_PUBLIC TOPIC_TOOLS_EXPORT
  #else
    #define TOPIC_TOOLS_PUBLIC TOPIC_TOOLS_IMPORT
  #endif
  #define TOPIC_TOOLS_PUBLIC_TYPE TOPIC_TOOLS_PUBLIC
  #define TOPIC_TOOLS_LOCAL
#else
  #define TOPIC_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define TOPIC_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define TOPIC_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define TOPIC_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TOPIC_TOOLS_PUBLIC
    #define TOPIC_TOOLS_LOCAL
  #endif
  #define TOPIC_TOOLS_PUBLIC_TYPE
#endif

#endif  // TOPIC_TOOLS__VISIBILITY_CONTROL_H_
