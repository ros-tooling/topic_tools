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
