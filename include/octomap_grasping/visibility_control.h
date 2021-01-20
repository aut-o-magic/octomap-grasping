#ifndef OCTOMAP_GRASPING__VISIBILITY_CONTROL_H_
#define OCTOMAP_GRASPING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OCTOMAP_GRASPING_EXPORT __attribute__ ((dllexport))
    #define OCTOMAP_GRASPING_IMPORT __attribute__ ((dllimport))
  #else
    #define OCTOMAP_GRASPING_EXPORT __declspec(dllexport)
    #define OCTOMAP_GRASPING_IMPORT __declspec(dllimport)
  #endif
  #ifdef OCTOMAP_GRASPING_BUILDING_LIBRARY
    #define OCTOMAP_GRASPING_PUBLIC OCTOMAP_GRASPING_EXPORT
  #else
    #define OCTOMAP_GRASPING_PUBLIC OCTOMAP_GRASPING_IMPORT
  #endif
  #define OCTOMAP_GRASPING_PUBLIC_TYPE OCTOMAP_GRASPING_PUBLIC
  #define OCTOMAP_GRASPING_LOCAL
#else
  #define OCTOMAP_GRASPING_EXPORT __attribute__ ((visibility("default")))
  #define OCTOMAP_GRASPING_IMPORT
  #if __GNUC__ >= 4
    #define OCTOMAP_GRASPING_PUBLIC __attribute__ ((visibility("default")))
    #define OCTOMAP_GRASPING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OCTOMAP_GRASPING_PUBLIC
    #define OCTOMAP_GRASPING_LOCAL
  #endif
  #define OCTOMAP_GRASPING_PUBLIC_TYPE
#endif

#endif  // OCTOMAP_GRASPING__VISIBILITY_CONTROL_H_
